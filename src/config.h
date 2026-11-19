// config.h ////////////////////////////////////////////////////////////////////

#pragma once

#include <Arduino.h>

// Sensors /////////////////////////////////////////////////////////////////////

// GPIO0, 1, 3, 4 are ADC1 and not strapping pins (GPIO2 is a strapping pin,
// GPIO5 is on the unreliable ADC2).
constexpr uint8_t SENSOR_PINS[] {0, 1, 3, 4};
constexpr size_t  NUM_SENSORS {sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0])};

// Which side of the bed each sensor is under. A side is occupied while any of
// its sensors is.
enum Side : uint8_t { SIDE_LEFT, SIDE_RIGHT };
constexpr Side SENSOR_SIDES[] {SIDE_RIGHT, SIDE_RIGHT, SIDE_LEFT, SIDE_LEFT};
static_assert(sizeof(SENSOR_SIDES) / sizeof(SENSOR_SIDES[0]) == NUM_SENSORS,
              "SENSOR_SIDES needs one entry per sensor");

// Full-scale raw reading (12-bit ADC)
constexpr uint16_t ADC_MAX {4095};

// Default hysteresis cutoffs per sensor, raw ADC counts. A sensor flips to
// OCCUPIED when its 1 s average reading drops below `low`, and back to
// UNOCCUPIED when it rises above `high`. Tune at runtime with the `set`
// command; tuned values are saved to flash and override these.
//
// Chosen from in-bed logs. S0 and S2 are each side's primary sensor: their
// readings stay well below `low` all night. S1 and S3 are backups that can
// spike above 3000 while occupied, so their `high` favors releasing reliably
// when empty. S2's `high` sits below the ~2250-2600 it reads while only the
// right side is occupied, so a false trigger clears instead of latching.
//                                              S0    S1    S2    S3
constexpr uint16_t DEFAULT_CUTOFF_LOW[]  {    1300, 1400, 1000, 1700};
constexpr uint16_t DEFAULT_CUTOFF_HIGH[] {    2700, 2700, 2100, 2600};
static_assert(sizeof(DEFAULT_CUTOFF_LOW) / sizeof(DEFAULT_CUTOFF_LOW[0]) == NUM_SENSORS &&
              sizeof(DEFAULT_CUTOFF_HIGH) / sizeof(DEFAULT_CUTOFF_HIGH[0]) == NUM_SENSORS,
              "Default cutoffs need one entry per sensor");

// LED /////////////////////////////////////////////////////////////////////////

// Onboard LED, active low. On while any sensor is occupied.
constexpr uint8_t LED_PIN {8};

enum LedState : uint8_t {
  LED_ON  = LOW,
  LED_OFF = HIGH,
};

// Timing //////////////////////////////////////////////////////////////////////

constexpr uint32_t LOOP_PERIOD_MS         {10};     // 100 Hz sensor sampling
constexpr uint32_t AVERAGE_PERIOD_MS      {1000};   // averaging window, 1 Hz output
constexpr uint32_t WIFI_CHECK_INTERVAL_MS {60000};  // re-join if disconnected

// Samples averaged per output. Occupancy decisions and the readings printout
// both run once per window, on the window's mean. A 1 s window spans a whole
// number of 50 Hz and 60 Hz mains cycles, so hum pickup averages out.
constexpr uint32_t SAMPLES_PER_AVERAGE {AVERAGE_PERIOD_MS / LOOP_PERIOD_MS};
static_assert(AVERAGE_PERIOD_MS % LOOP_PERIOD_MS == 0,
              "Averaging window must be a whole number of sample periods");

// Once the bed (either side) has been occupied this long since it was last
// empty, the last-in-bed time updates to now every window.
constexpr uint32_t IN_BED_MIN_MS {5UL * 60UL * 1000UL};  // 5 minutes
constexpr uint32_t IN_BED_MIN_WINDOWS {IN_BED_MIN_MS / AVERAGE_PERIOD_MS};
static_assert(IN_BED_MIN_MS % AVERAGE_PERIOD_MS == 0,
              "In-bed minimum must be a whole number of averaging windows");

// Clock ///////////////////////////////////////////////////////////////////////

// Local timezone as a POSIX TZ string. The ESP32 applies the DST rules in it
// on its own, so local time stays correct across DST changes even if NTP is
// unreachable. US Pacific: standard time UTC-8, DST UTC-7 from the 2nd Sunday
// of March to the 1st Sunday of November.
constexpr const char* TIMEZONE_POSIX {"PST8PDT,M3.2.0,M11.1.0"};

// NTP servers used to set the clock and correct drift
constexpr const char* NTP_SERVER_1 {"pool.ntp.org"};
constexpr const char* NTP_SERVER_2 {"time.nist.gov"};

// Local hour (24-hour clock) of the nightly NTP re-sync. Runs once per day at
// or after this hour.
constexpr int NIGHTLY_SYNC_HOUR {2};

// How long (ms) to wait for an NTP response before reporting a failed sync.
// The wait never blocks sensing.
constexpr uint32_t NTP_SYNC_TIMEOUT_MS {10000};

// Network /////////////////////////////////////////////////////////////////////

// Web console: http://<MDNS_HOSTNAME>.local/
// State for other nodes: http://<MDNS_HOSTNAME>.local<STATE_PATH>
constexpr const char* MDNS_HOSTNAME {"bedoccupancy"};
constexpr const char* STATE_PATH    {"/state"};
constexpr uint16_t    SERVER_PORT   {80};

// Serial //////////////////////////////////////////////////////////////////////

constexpr uint32_t SERIAL_BAUD_RATE {115200};

// Longest command line accepted from UART or the web console
constexpr size_t CMD_MAX_LEN {64};

// Flash storage ///////////////////////////////////////////////////////////////

// Preferences (NVS) namespace for the tuned cutoffs
constexpr const char* PREFS_NAMESPACE {"bedocc"};

// End of file /////////////////////////////////////////////////////////////////
