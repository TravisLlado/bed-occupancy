/*
 * Bed Occupancy - four-sensor hysteresis with a web console for tuning
 *
 * Four flat-flex (force-sensitive) sensors, each with its own 10k pullup to
 * 3.3V: high = no contact, low = full contact. Each sensor is sampled at 100 Hz
 * and averaged over 1 s windows; each window's mean runs through a hysteresis
 * filter with that sensor's own low/high cutoffs (see config.h). Each side of
 * the bed is occupied while any of its sensors is. The onboard LED is on while
 * either side is occupied.
 *
 * Other nodes read per-side state, the measurement timestamp, and the bed's
 * last-in-bed time (occupied for IN_BED_MIN_MS or longer) as JSON from
 * GET <STATE_PATH>. The clock is set from NTP after WiFi joins and re-synced
 * nightly; sensing never waits on it.
 *
 * Commands are accepted from both USB serial and the web console
 * (http://<MDNS_HOSTNAME>.local/); type `help` for the list. Once per window,
 * one line prints: timestamp, each sensor's average, and each side's state.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <WebConsole.h>
#include <esp_sntp.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <stdarg.h>
#include <time.h>
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
static uint32_t g_sum[NUM_SENSORS] {};    // window in progress
static uint32_t g_accumCount {0};          // samples in the window in progress
static uint16_t g_mean[NUM_SENSORS] {};    // last completed window's mean
static bool     g_sideOccupied[2] {};      // indexed by Side
static time_t   g_measuredAt {0};          // last window's end, 0 if clock unset

// Consecutive windows with either side occupied, and the last time the bed had
// been occupied for IN_BED_MIN_WINDOWS or longer, in seconds since boot. 0 at
// boot. lastInBedTime() moves it onto the real clock once that's set.
static uint32_t g_occupiedWindows {0};
static int64_t  g_lastInBedUptime {0};

// Latest state as served at STATE_PATH. Written by loop(), read by the HTTP
// handler on the AsyncTCP task, so it's copied under a lock.
struct StateSnapshot {
  bool   ready;     // false until the first window completes
  bool   left;
  bool   right;
  time_t measuredAt;
  time_t lastInBed;  // see lastInBedTime()
};
static portMUX_TYPE  s_stateMux = portMUX_INITIALIZER_UNLOCKED;
static StateSnapshot s_state {};

// Incremented by the SNTP task each time the clock is set from NTP
static volatile uint32_t s_syncCount {0};

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
    g_cutoffs[i].low = prefs.getUShort(key, DEFAULT_CUTOFF_LOW[i]);
    keyFor(key, sizeof(key), "hi", i);
    g_cutoffs[i].high = prefs.getUShort(key, DEFAULT_CUTOFF_HIGH[i]);

    // Guard against a corrupt or inconsistent stored pair
    if (g_cutoffs[i].low >= g_cutoffs[i].high || g_cutoffs[i].high > ADC_MAX) {
      g_cutoffs[i] = {DEFAULT_CUTOFF_LOW[i], DEFAULT_CUTOFF_HIGH[i]};
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

// Local time as "HH:MM:SS", or "--:--:--" before the clock is set
static void formatClock(char* buf, size_t size, time_t t) {
  if (t == 0) {
    snprintf(buf, size, "--:--:--");
    return;
  }
  struct tm lt;
  localtime_r(&t, &lt);
  strftime(buf, size, "%H:%M:%S", &lt);
}

// One line: measurement time, each sensor's average reading for the last
// window, then each side's state
static void printReadings() {
  char line[96];
  char clock[16];
  formatClock(clock, sizeof(clock), g_measuredAt);
  size_t n{(size_t)snprintf(line, sizeof(line), "%s |", clock)};
  for (size_t i{0}; i < NUM_SENSORS; i++) {
    n += snprintf(line + n, sizeof(line) - n, " %4u", g_mean[i]);
  }
  snprintf(line + n, sizeof(line) - n, " | L %s  R %s\n",
           g_sideOccupied[SIDE_LEFT] ? "OCC" : "---",
           g_sideOccupied[SIDE_RIGHT] ? "OCC" : "---");
  out("%s", line);
}

// Defined with the clock code below
static bool    clockValid();
static int64_t uptimeS();
static time_t  lastInBedTime();

static void printLastInBed() {
  char buf[40];
  const time_t t{lastInBedTime()};
  if (t == 0) {
    snprintf(buf, sizeof(buf), "0 (not since boot)");
  } else if (!clockValid()) {
    snprintf(buf, sizeof(buf), "%lld s after boot", (long long)t);
  } else {
    struct tm lt;
    localtime_r(&t, &lt);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &lt);
  }
  out("Last in bed (%u+ min): %s\n", (unsigned)(IN_BED_MIN_MS / 60000), buf);
}

// Why the chip last reset, for spotting unexpected reboots
static const char* resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_SW:       return "software restart";
    case ESP_RST_PANIC:    return "crash";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    case ESP_RST_EXT:      return "reset pin";
    default:               return "other";
  }
}

static void printUptime() {
  const int64_t up{uptimeS()};
  out("Uptime %lldh %02lldm %02llds (last reset: %s). Occupied streak %u / %u s\n",
      (long long)(up / 3600), (long long)(up / 60 % 60), (long long)(up % 60),
      resetReasonName(), (unsigned)g_occupiedWindows, (unsigned)IN_BED_MIN_WINDOWS);
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
  out("  show                               show cutoffs, last-in-bed time, latest readings\n");
  out("  defaults                           restore default cutoffs on all sensors\n");
  out("  help                               this list\n");
  out("Cutoffs apply to each sensor's %u ms average reading.\n", (unsigned)AVERAGE_PERIOD_MS);
  out("Occupied when average < low, unoccupied when average > high. Low must be below high.\n");
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
    printLastInBed();
    printUptime();
    printReadings();
  } else if (strcmp(cmd, "defaults") == 0) {
    for (size_t i{0}; i < NUM_SENSORS; i++) {
      g_cutoffs[i] = {DEFAULT_CUTOFF_LOW[i], DEFAULT_CUTOFF_HIGH[i]};
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
// Clock
// ============================================================================

// Runs on the SNTP task
static void onTimeSync(struct timeval*) { s_syncCount = s_syncCount + 1; }

// True once the clock has been set from NTP at least once since boot. There's
// no battery-backed RTC, so until then time() is meaningless.
static bool clockValid() { return s_syncCount > 0; }

// Seconds since boot. 64-bit, so it doesn't wrap like millis().
static int64_t uptimeS() { return esp_timer_get_time() / 1000000; }

// Last-in-bed time as published: 0 if not since boot, seconds since boot until
// the clock is set, then Unix seconds UTC. The offset between the two clocks is
// taken fresh each call, so NTP corrections carry through.
static time_t lastInBedTime() {
  if (g_lastInBedUptime == 0 || !clockValid()) return (time_t)g_lastInBedUptime;
  const int64_t offsetS{(int64_t)time(nullptr) - uptimeS()};
  return (time_t)(g_lastInBedUptime + offsetS);
}

// Start SNTP with the local timezone. SNTP retries on its own until the first
// sync succeeds, then re-syncs in the background every few hours.
static void startClock() {
  sntp_set_time_sync_notification_cb(onTimeSync);
  configTzTime(TIMEZONE_POSIX, NTP_SERVER_1, NTP_SERVER_2);
}

// Report the first sync, and force an NTP re-sync once a night at or after
// NIGHTLY_SYNC_HOUR. Never blocks: a requested sync is checked on later calls.
// A failed sync isn't fatal. The clock keeps running from its last sync, and
// the TZ rules keep DST correct.
static void serviceClock() {
  static uint32_t seenSyncs{0};
  static int      lastNightlyDay{-1};  // tm_yday of the last nightly sync
  static bool     pending{false};
  static uint32_t pendingSinceMs{0};
  static uint32_t pendingBase{0};

  const uint32_t syncs{s_syncCount};
  if (syncs == 0) return;

  const time_t now{time(nullptr)};
  struct tm lt;
  localtime_r(&now, &lt);

  if (seenSyncs == 0) {
    char buf[40];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &lt);
    out("Clock set from NTP: %s\n", buf);

    // A first sync after tonight's sync hour stands in for tonight's sync
    if (lt.tm_hour >= NIGHTLY_SYNC_HOUR) lastNightlyDay = lt.tm_yday;
  }
  seenSyncs = syncs;

  if (pending) {
    if (syncs != pendingBase) {
      pending = false;
      out("Clock re-synced from NTP.\n");
    } else if (millis() - pendingSinceMs >= NTP_SYNC_TIMEOUT_MS) {
      pending = false;
      out("NTP sync failed, using onboard clock.\n");
    }
  } else if (lt.tm_hour >= NIGHTLY_SYNC_HOUR && lt.tm_yday != lastNightlyDay) {
    lastNightlyDay = lt.tm_yday;
    pending = true;
    pendingBase = syncs;
    pendingSinceMs = millis();
    sntp_restart();  // send a request now rather than at the next poll interval
  }
}

// ============================================================================
// HTTP state interface
// ============================================================================

// Unix seconds as a JSON number, or null for 0
static void formatJsonTime(char* buf, size_t size, time_t t) {
  if (t != 0) {
    snprintf(buf, size, "%lld", (long long)t);
  } else {
    snprintf(buf, size, "null");
  }
}

// GET STATE_PATH. Runs on the AsyncTCP task.
//   200 {"left":false,"right":true,"time":1763512345,"lastInBed":1763512345}
//       time: end of the measurement window, Unix seconds UTC; null until the
//       clock has been set from NTP
//       lastInBed: last time the bed had been occupied (either side) for
//       IN_BED_MIN_MS or longer. 0 at boot; seconds since boot until the clock
//       is set from NTP, then Unix seconds UTC
//   503 {"error":"starting"} until the first measurement window completes
static void handleState(AsyncWebServerRequest* request) {
  StateSnapshot s;
  portENTER_CRITICAL(&s_stateMux);
  s = s_state;
  portEXIT_CRITICAL(&s_stateMux);

  AsyncWebServerResponse* response;
  if (!s.ready) {
    response = request->beginResponse(503, "application/json", "{\"error\":\"starting\"}");
  } else {
    char stamp[24];
    formatJsonTime(stamp, sizeof(stamp), s.measuredAt);
    char body[128];
    snprintf(body, sizeof(body),
             "{\"left\":%s,\"right\":%s,\"time\":%s,\"lastInBed\":%lld}",
             s.left ? "true" : "false", s.right ? "true" : "false", stamp,
             (long long)s.lastInBed);
    response = request->beginResponse(200, "application/json", body);
  }
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

// ============================================================================
// Network
// ============================================================================

// Bring up mDNS and the clock the first time WiFi connects, and re-join if it
// drops. Sensing never waits on the network.
static void serviceWifi() {
  static bool          announced{false};
  static unsigned long lastCheckMs{0};

  if (!announced && WiFi.status() == WL_CONNECTED) {
    announced = true;
    startClock();
    if (MDNS.begin(MDNS_HOSTNAME)) {
      MDNS.addService("http", "tcp", SERVER_PORT);
    } else {
      Serial.println("mDNS failed");
    }
    const String ip{WiFi.localIP().toString()};
    out("Web console: http://%s.local/  (http://%s/)\n", MDNS_HOSTNAME, ip.c_str());
    out("State:       http://%s.local%s  (http://%s%s)\n", MDNS_HOSTNAME, STATE_PATH,
        ip.c_str(), STATE_PATH);
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
// Sampling
// ============================================================================

static void resetAccum() {
  for (uint32_t& sum : g_sum) sum = 0;
  g_accumCount = 0;
}

// Add one sample per sensor to the window in progress
static void sampleSensors() {
  for (size_t i{0}; i < NUM_SENSORS; i++) {
    g_sum[i] += analogRead(SENSOR_PINS[i]);
  }
  g_accumCount++;
}

// If the bed (either side) has been occupied for IN_BED_MIN_WINDOWS since it
// was last empty, set the last-in-bed time to now (seconds since boot)
static void updateLastInBed() {
  if (g_sideOccupied[SIDE_LEFT] || g_sideOccupied[SIDE_RIGHT]) {
    if (g_occupiedWindows < IN_BED_MIN_WINDOWS) g_occupiedWindows++;
    if (g_occupiedWindows >= IN_BED_MIN_WINDOWS) g_lastInBedUptime = uptimeS();
  } else {
    g_occupiedWindows = 0;
  }
}

// Close the window: compute means, run hysteresis, combine sensors into sides,
// timestamp, publish, update LED, print
static void finishWindow() {
  g_measuredAt = clockValid() ? time(nullptr) : 0;

  bool side[2] {};
  for (size_t i{0}; i < NUM_SENSORS; i++) {
    g_mean[i] = (uint16_t)((g_sum[i] + g_accumCount / 2) / g_accumCount);

    if (!g_occupied[i] && g_mean[i] < g_cutoffs[i].low) {
      g_occupied[i] = true;
    } else if (g_occupied[i] && g_mean[i] > g_cutoffs[i].high) {
      g_occupied[i] = false;
    }
    side[SENSOR_SIDES[i]] |= g_occupied[i];
  }
  g_sideOccupied[SIDE_LEFT]  = side[SIDE_LEFT];
  g_sideOccupied[SIDE_RIGHT] = side[SIDE_RIGHT];
  updateLastInBed();

  portENTER_CRITICAL(&s_stateMux);
  s_state = {true, side[SIDE_LEFT], side[SIDE_RIGHT], g_measuredAt, lastInBedTime()};
  portEXIT_CRITICAL(&s_stateMux);

  digitalWrite(LED_PIN, (side[SIDE_LEFT] || side[SIDE_RIGHT]) ? LED_ON : LED_OFF);
  printReadings();
  resetAccum();
}

// ============================================================================
// Setup / loop
// ============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("\r\n\r\nBed Occupancy");
  Serial.printf("Reset reason: %s\r\n", resetReasonName());

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  analogReadResolution(12);
  for (const uint8_t pin : SENSOR_PINS) {
    analogSetPinAttenuation(pin, ADC_11db);
  }

  loadCutoffs();
  printCutoffs();
  resetAccum();

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_HOSTNAME);

  // Give the station an IPv6 link-local address once it joins, so mDNS can
  // answer AAAA queries. Without one, macOS waits ~5 s for an AAAA answer on
  // every bedoccupancy.local lookup.
  WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t) { WiFi.enableIpV6(); },
               ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  server.on(STATE_PATH, HTTP_GET, handleState);
  webConsole.begin(server, "/", onWebLine);
  server.begin();

  Serial.println("Type 'help' for commands.");
}

void loop() {
  static TickType_t lastWake{xTaskGetTickCount()};

  sampleSensors();
  if (g_accumCount >= SAMPLES_PER_AVERAGE) finishWindow();

  pollSerialCommand();
  pollWebCommand();
  serviceWifi();
  serviceClock();

  // Send buffered output to the browser
  webConsole.loop();

  vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
}
