// config.h ////////////////////////////////////////////////////////////////////

#pragma once

#include <Arduino.h>

// Sensors /////////////////////////////////////////////////////////////////////

// GPIO0, 1, 3, 4 are ADC1 and not strapping pins (GPIO2 is a strapping pin,
// GPIO5 is on the unreliable ADC2).
constexpr uint8_t SENSOR_PINS[] {0, 1, 3, 4};
constexpr size_t  NUM_SENSORS {sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0])};

// Full-scale raw reading (12-bit ADC)
constexpr uint16_t ADC_MAX {4095};

// Default hysteresis cutoffs, raw ADC counts. A sensor flips to OCCUPIED when
// its reading drops below `low`, and back to UNOCCUPIED when it rises above
// `high`. Defaults are 30% / 70% of the tabletop span (~325 pressed, 4095
// unpressed). Tune per sensor at runtime with the `set` command; tuned values
// are saved to flash and override these.
constexpr uint16_t DEFAULT_CUTOFF_LOW  {1456};
constexpr uint16_t DEFAULT_CUTOFF_HIGH {2964};

// LED /////////////////////////////////////////////////////////////////////////

// Onboard LED, active low. On while any sensor is occupied.
constexpr uint8_t LED_PIN {8};

enum LedState : uint8_t {
  LED_ON  = LOW,
  LED_OFF = HIGH,
};

// Timing //////////////////////////////////////////////////////////////////////

constexpr uint32_t LOOP_PERIOD_MS         {10};     // 100 Hz sensor sampling
constexpr uint32_t STATUS_INTERVAL_MS     {1000};   // readings printout (1 Hz)
constexpr uint32_t WIFI_CHECK_INTERVAL_MS {60000};  // re-join if disconnected

// Network /////////////////////////////////////////////////////////////////////

// Web console: http://<MDNS_HOSTNAME>.local/
constexpr const char* MDNS_HOSTNAME {"bedtest"};
constexpr uint16_t    SERVER_PORT   {80};

// Serial //////////////////////////////////////////////////////////////////////

constexpr uint32_t SERIAL_BAUD_RATE {115200};

// Longest command line accepted from UART or the web console
constexpr size_t CMD_MAX_LEN {64};

// Flash storage ///////////////////////////////////////////////////////////////

// Preferences (NVS) namespace for the tuned cutoffs
constexpr const char* PREFS_NAMESPACE {"bedocc"};

// End of file /////////////////////////////////////////////////////////////////
