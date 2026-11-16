// Bed Occupancy - four-sensor hysteresis demo
//
// Four flat-flex (force-sensitive) sensors, each with its own 10k pullup to 3.3V:
//   high = no contact, low = full contact.
// Each sensor's raw reading is normalized to 0-100% of its calibrated span
// (0% = fully pressed, 100% = unpressed) and run through a hysteresis filter:
//   - flips to OCCUPIED   when it drops below OCCUPIED_BELOW_PCT
//   - flips to UNOCCUPIED when it rises above UNOCCUPIED_ABOVE_PCT
// The onboard LED is on while any sensor is occupied.
// Loop runs at 100 Hz; status prints over USB serial at 10 Hz, and every
// state change prints immediately.

#include <Arduino.h>

// --- Sensors ---
// Pins: GPIO0, 1, 3, 4 are ADC1 and not strapping pins (GPIO2 is a strapping
// pin, GPIO5 is on the unreliable ADC2).
// Calibration: raw 12-bit ADC counts when fully pressed / unpressed.
struct SensorConfig {
    uint8_t  pin;
    uint16_t rawPressed;
    uint16_t rawUnpressed;
};

constexpr SensorConfig SENSORS[] = {
    {0, 325, 4095},
    {1, 325, 4095},
    {3, 325, 4095},
    {4, 325, 4095},
};
constexpr size_t NUM_SENSORS = sizeof(SENSORS) / sizeof(SENSORS[0]);

// --- Hysteresis thresholds (percent of calibrated span) ---
constexpr int32_t OCCUPIED_BELOW_PCT   = 30;
constexpr int32_t UNOCCUPIED_ABOVE_PCT = 70;
static_assert(OCCUPIED_BELOW_PCT < UNOCCUPIED_ABOVE_PCT,
              "occupied threshold must be below unoccupied threshold");

// --- LED ---
constexpr uint8_t PIN_LED = 8;  // Onboard LED, active low

// --- Timing ---
constexpr uint32_t LOOP_PERIOD_MS = 10;  // 100 Hz
constexpr uint32_t PRINT_EVERY_N  = 10;  // print at 10 Hz

static bool occupied[NUM_SENSORS] = {};

// Map a raw reading to 0-100% of the sensor's calibrated span.
static int32_t toPercent(const SensorConfig& cfg, int32_t raw) {
    int32_t span = int32_t(cfg.rawUnpressed) - int32_t(cfg.rawPressed);
    if (span <= 0) return 100;
    int32_t pct = (raw - int32_t(cfg.rawPressed)) * 100 / span;
    return constrain(pct, 0, 100);
}

void setup() {
    Serial.begin(115200);

    analogReadResolution(12);
    for (const auto& s : SENSORS) {
        analogSetPinAttenuation(s.pin, ADC_11db);
    }

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);  // off
}

void loop() {
    static TickType_t lastWake  = xTaskGetTickCount();
    static uint32_t   iteration = 0;

    int32_t raw[NUM_SENSORS];
    int32_t pct[NUM_SENSORS];
    bool    anyOccupied = false;

    for (size_t i = 0; i < NUM_SENSORS; ++i) {
        raw[i] = analogRead(SENSORS[i].pin);
        pct[i] = toPercent(SENSORS[i], raw[i]);

        bool prev = occupied[i];
        if (!occupied[i] && pct[i] < OCCUPIED_BELOW_PCT) {
            occupied[i] = true;
        } else if (occupied[i] && pct[i] > UNOCCUPIED_ABOVE_PCT) {
            occupied[i] = false;
        }

        if (occupied[i] != prev) {
            Serial.printf("S%u -> %s (raw %ld, %ld%%)\n", (unsigned)i,
                          occupied[i] ? "OCCUPIED" : "UNOCCUPIED",
                          (long)raw[i], (long)pct[i]);
        }
        anyOccupied |= occupied[i];
    }

    digitalWrite(PIN_LED, anyOccupied ? LOW : HIGH);

    if (++iteration % PRINT_EVERY_N == 0) {
        for (size_t i = 0; i < NUM_SENSORS; ++i) {
            Serial.printf("S%u %4ld %3ld%% %s   ", (unsigned)i, (long)raw[i],
                          (long)pct[i], occupied[i] ? "OCC" : "---");
        }
        Serial.println();
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
}
