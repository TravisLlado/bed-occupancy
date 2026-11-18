/*
 * Bed Occupancy - four-sensor hysteresis with a web console for tuning
 *
 * Four flat-flex (force-sensitive) sensors, each with its own 10k pullup to
 * 3.3V: high = no contact, low = full contact. Each sensor's raw reading runs
 * through a hysteresis filter with its own low/high cutoffs (see config.h).
 * The onboard LED is on while any sensor is occupied.
 *
 * Commands are accepted from both USB serial and the web console
 * (http://<MDNS_HOSTNAME>.local/); type `help` for the list. Readings print to
 * both once per second, and every state change prints immediately.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <WebConsole.h>
#include <stdarg.h>
#include "config.h"
#include "credentials.h"

// ============================================================================
// State
// ============================================================================

struct Cutoffs {
  uint16_t low;   // occupied below this
  uint16_t high;  // unoccupied above this
};

static Cutoffs  g_cutoffs[NUM_SENSORS];
static bool     g_occupied[NUM_SENSORS] {};
static uint16_t g_raw[NUM_SENSORS] {};

AsyncWebServer    server(SERVER_PORT);
static WebConsole webConsole;
static Preferences prefs;

// The web console's line handler runs on the AsyncTCP task. It only copies the
// line here; loop() runs it, so all state changes, flash writes and output
// happen on one task.
static portMUX_TYPE s_cmdMux = portMUX_INITIALIZER_UNLOCKED;
static char         s_pendingCmd[CMD_MAX_LEN + 1];
static bool         s_hasPendingCmd {false};

// ============================================================================
// Output (USB serial + web console)
// ============================================================================

static void out(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  webConsole.print(buf);
}

// ============================================================================
// Cutoff storage
// ============================================================================

static void keyFor(char* key, size_t size, const char* kind, size_t sensor) {
  snprintf(key, size, "%s%u", kind, (unsigned)sensor);
}

static void loadCutoffs() {
  prefs.begin(PREFS_NAMESPACE, true);
  for (size_t i{0}; i < NUM_SENSORS; i++) {
    char key[8];
    keyFor(key, sizeof(key), "lo", i);
    g_cutoffs[i].low = prefs.getUShort(key, DEFAULT_CUTOFF_LOW);
    keyFor(key, sizeof(key), "hi", i);
    g_cutoffs[i].high = prefs.getUShort(key, DEFAULT_CUTOFF_HIGH);

    // Guard against a corrupt or inconsistent stored pair
    if (g_cutoffs[i].low >= g_cutoffs[i].high || g_cutoffs[i].high > ADC_MAX) {
      g_cutoffs[i] = {DEFAULT_CUTOFF_LOW, DEFAULT_CUTOFF_HIGH};
    }
  }
  prefs.end();
}

static void saveCutoffs(size_t sensor) {
  prefs.begin(PREFS_NAMESPACE, false);
  char key[8];
  keyFor(key, sizeof(key), "lo", sensor);
  prefs.putUShort(key, g_cutoffs[sensor].low);
  keyFor(key, sizeof(key), "hi", sensor);
  prefs.putUShort(key, g_cutoffs[sensor].high);
  prefs.end();
}

// ============================================================================
// Printing
// ============================================================================

static void printReadings() {
  char line[160];
  size_t n{0};
  bool any{false};
  for (size_t i{0}; i < NUM_SENSORS; i++) {
    n += snprintf(line + n, sizeof(line) - n, "S%u %4u %s | ", (unsigned)i,
                  g_raw[i], g_occupied[i] ? "OCC" : "---");
    any |= g_occupied[i];
  }
  snprintf(line + n, sizeof(line) - n, "bed %s\n", any ? "OCCUPIED" : "empty");
  out("%s", line);
}

static void printCutoffs() {
  for (size_t i{0}; i < NUM_SENSORS; i++) {
    out("S%u (GPIO%u): low %4u  high %4u\n", (unsigned)i,
        (unsigned)SENSOR_PINS[i], g_cutoffs[i].low, g_cutoffs[i].high);
  }
}

static void printHelp() {
  out("Commands:\n");
  out("  set <0-%u|all> <low|high> <value>   set a cutoff (raw ADC 0-%u)\n",
      (unsigned)(NUM_SENSORS - 1), (unsigned)ADC_MAX);
  out("  show                               show cutoffs and current readings\n");
  out("  defaults                           restore default cutoffs on all sensors\n");
  out("  help                               this list\n");
  out("Occupied when reading < low, unoccupied when reading > high. Low must be below high.\n");
  out("Example: set 2 low 1400\n");
}

// ============================================================================
// Command handling
// ============================================================================

static void cmdSet(const char* target, const char* which, const char* valueStr) {
  // Sensor index, or all
  size_t first{0}, last{NUM_SENSORS - 1};
  if (strcmp(target, "all") != 0) {
    char* end;
    long idx{strtol(target, &end, 10)};
    if (*end != '\0' || idx < 0 || idx >= (long)NUM_SENSORS) {
      out("Bad sensor '%s'. Use 0-%u or all.\n", target, (unsigned)(NUM_SENSORS - 1));
      return;
    }
    first = last = (size_t)idx;
  }

  const bool isLow{strcmp(which, "low") == 0};
  if (!isLow && strcmp(which, "high") != 0) {
    out("Bad cutoff '%s'. Use low or high.\n", which);
    return;
  }

  char* end;
  long value{strtol(valueStr, &end, 10)};
  if (*end != '\0' || value < 0 || value > ADC_MAX) {
    out("Bad value '%s'. Use 0-%u.\n", valueStr, (unsigned)ADC_MAX);
    return;
  }

  // Validate every target before changing any, so `set all` is all-or-nothing
  for (size_t i{first}; i <= last; i++) {
    const uint16_t lo{isLow ? (uint16_t)value : g_cutoffs[i].low};
    const uint16_t hi{isLow ? g_cutoffs[i].high : (uint16_t)value};
    if (lo >= hi) {
      out("Rejected: S%u would have low %u >= high %u.\n", (unsigned)i, lo, hi);
      return;
    }
  }

  for (size_t i{first}; i <= last; i++) {
    (isLow ? g_cutoffs[i].low : g_cutoffs[i].high) = (uint16_t)value;
    saveCutoffs(i);
    out("S%u: low %u  high %u (saved)\n", (unsigned)i, g_cutoffs[i].low, g_cutoffs[i].high);
  }
}

static void handleCommand(char* line) {
  // Lowercase and tokenize on whitespace
  for (char* p{line}; *p; p++) *p = tolower((unsigned char)*p);

  char* tokens[5];
  size_t count{0};
  for (char* tok{strtok(line, " \t\r\n")}; tok && count < 5; tok = strtok(nullptr, " \t\r\n")) {
    tokens[count++] = tok;
  }
  if (count == 0) return;

  const char* cmd{tokens[0]};
  if (strcmp(cmd, "set") == 0) {
    if (count != 4) {
      out("Usage: set <0-%u|all> <low|high> <value>\n", (unsigned)(NUM_SENSORS - 1));
      return;
    }
    cmdSet(tokens[1], tokens[2], tokens[3]);
  } else if (strcmp(cmd, "show") == 0 || strcmp(cmd, "status") == 0) {
    printCutoffs();
    printReadings();
  } else if (strcmp(cmd, "defaults") == 0) {
    for (size_t i{0}; i < NUM_SENSORS; i++) {
      g_cutoffs[i] = {DEFAULT_CUTOFF_LOW, DEFAULT_CUTOFF_HIGH};
      saveCutoffs(i);
    }
    out("Defaults restored and saved.\n");
    printCutoffs();
  } else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
    printHelp();
  } else {
    out("Unknown command '%s'. Type 'help' for commands.\n", cmd);
  }
}

static void onWebLine(const char* data, size_t len) {
  if (len > CMD_MAX_LEN) len = CMD_MAX_LEN;
  portENTER_CRITICAL(&s_cmdMux);
  if (!s_hasPendingCmd) {  // drop if the previous one hasn't been run yet
    memcpy(s_pendingCmd, data, len);
    s_pendingCmd[len] = '\0';
    s_hasPendingCmd = true;
  }
  portEXIT_CRITICAL(&s_cmdMux);
}

// Run a command queued by the web console, if any
static void pollWebCommand() {
  char line[CMD_MAX_LEN + 1];
  bool have{false};
  portENTER_CRITICAL(&s_cmdMux);
  if (s_hasPendingCmd) {
    memcpy(line, s_pendingCmd, sizeof(line));
    s_hasPendingCmd = false;
    have = true;
  }
  portEXIT_CRITICAL(&s_cmdMux);
  if (have) handleCommand(line);
}

// Accumulate UART characters and run each complete line
static void pollSerialCommand() {
  static char   buf[CMD_MAX_LEN + 1];
  static size_t len{0};
  while (Serial.available()) {
    const char c{(char)Serial.read()};
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handleCommand(buf);
        len = 0;
      }
    } else if (len < CMD_MAX_LEN) {
      buf[len++] = c;
    }
  }
}

// ============================================================================
// Network
// ============================================================================

// Bring up mDNS the first time WiFi connects, and re-join if it drops. Sensing
// never waits on the network.
static void serviceWifi() {
  static bool          announced{false};
  static unsigned long lastCheckMs{0};

  if (!announced && WiFi.status() == WL_CONNECTED) {
    announced = true;
    if (MDNS.begin(MDNS_HOSTNAME)) {
      MDNS.addService("http", "tcp", SERVER_PORT);
    } else {
      Serial.println("mDNS failed");
    }
    out("Web console: http://%s.local/  (http://%s/)\n", MDNS_HOSTNAME,
        WiFi.localIP().toString().c_str());
  }

  const unsigned long now{millis()};
  if (now - lastCheckMs >= WIFI_CHECK_INTERVAL_MS) {
    lastCheckMs = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost, reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
}

// ============================================================================
// Setup / loop
// ============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("\r\n\r\nBed Occupancy");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  analogReadResolution(12);
  for (const uint8_t pin : SENSOR_PINS) {
    analogSetPinAttenuation(pin, ADC_11db);
  }

  loadCutoffs();
  printCutoffs();

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_HOSTNAME);

  // Give the station an IPv6 link-local address once it joins, so mDNS can
  // answer AAAA queries. Without one, macOS waits ~5 s for an AAAA answer on
  // every bedoccupancy.local lookup.
  WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t) { WiFi.enableIpV6(); },
               ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  webConsole.begin(server, "/", onWebLine);
  server.begin();

  Serial.println("Type 'help' for commands.");
}

void loop() {
  static TickType_t    lastWake{xTaskGetTickCount()};
  static unsigned long lastStatusMs{0};

  // Sample and filter
  bool anyOccupied{false};
  for (size_t i{0}; i < NUM_SENSORS; i++) {
    g_raw[i] = analogRead(SENSOR_PINS[i]);

    const bool prev{g_occupied[i]};
    if (!g_occupied[i] && g_raw[i] < g_cutoffs[i].low) {
      g_occupied[i] = true;
    } else if (g_occupied[i] && g_raw[i] > g_cutoffs[i].high) {
      g_occupied[i] = false;
    }
    if (g_occupied[i] != prev) {
      out("S%u -> %s (raw %u)\n", (unsigned)i,
          g_occupied[i] ? "OCCUPIED" : "UNOCCUPIED", g_raw[i]);
    }
    anyOccupied |= g_occupied[i];
  }
  digitalWrite(LED_PIN, anyOccupied ? LED_ON : LED_OFF);

  pollSerialCommand();
  pollWebCommand();
  serviceWifi();

  const unsigned long now{millis()};
  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    printReadings();
  }

  // Send buffered output to the browser
  webConsole.loop();

  vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
}
