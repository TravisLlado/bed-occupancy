// Bed Occupancy - single-sensor demo
//
// Flat-flex (force-sensitive) sensor on GPIO0 with a 10k pullup to 3.3V:
//   3.3V = no contact, 0V = full contact.
// Runs at 100 Hz and sets the onboard LED brightness proportional to the
// measured voltage (3.3V -> full brightness, 0V -> off).

#include <Arduino.h>

// --- Pins ---
constexpr uint8_t PIN_SENSOR = 0;  // GPIO0 = ADC1_CH0
constexpr uint8_t PIN_LED    = 8;  // Onboard LED, active low

// --- LED PWM ---
constexpr uint8_t  LED_PWM_CHANNEL    = 0;
constexpr uint32_t LED_PWM_FREQ_HZ    = 1000;
constexpr uint8_t  LED_PWM_RESOLUTION = 12;  // bits
constexpr uint32_t LED_PWM_MAX        = (1u << LED_PWM_RESOLUTION) - 1;

// --- Timing ---
constexpr uint32_t LOOP_PERIOD_MS  = 10;   // 100 Hz
constexpr uint32_t PRINT_EVERY_N   = 10;   // print at 10 Hz

// --- Sensor ---
constexpr uint32_t SUPPLY_MV = 3300;

static void setLedBrightness(uint32_t level) {
    if (level > LED_PWM_MAX) level = LED_PWM_MAX;
    // Inverted logic: LOW = on, so full brightness = 0 duty.
    ledcWrite(LED_PWM_CHANNEL, LED_PWM_MAX - level);
}

void setup() {
    Serial.begin(115200);

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_SENSOR, ADC_11db);  // widest range (~0-2.5V linear, saturates near ~2.9V+)

    ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ_HZ, LED_PWM_RESOLUTION);
    ledcAttachPin(PIN_LED, LED_PWM_CHANNEL);
    setLedBrightness(0);
}

void loop() {
    static TickType_t lastWake = xTaskGetTickCount();
    static uint32_t   iteration = 0;

    uint32_t mv = analogReadMilliVolts(PIN_SENSOR);  // eFuse-calibrated
    if (mv > SUPPLY_MV) mv = SUPPLY_MV;

    uint32_t level = (mv * LED_PWM_MAX) / SUPPLY_MV;
    setLedBrightness(level);

    if (++iteration % PRINT_EVERY_N == 0) {
        Serial.printf("sensor: %4u mV  led: %3u%%\n", mv, (level * 100) / LED_PWM_MAX);
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
}
