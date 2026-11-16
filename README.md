# Bed Occupancy

Detects whether a bed is occupied using flat-flex (force-sensitive) sensors read by an ESP32-C3.

## Current demo

Four sensors, one LED:

| Signal | Pin | Notes |
|---|---|---|
| Sensor 0 | GPIO0 (ADC1_CH0) | Each sensor has its own 10k pullup to 3.3V. High = no contact, low = full contact |
| Sensor 1 | GPIO1 (ADC1_CH1) | |
| Sensor 2 | GPIO3 (ADC1_CH3) | |
| Sensor 3 | GPIO4 (ADC1_CH4) | |
| Onboard LED | GPIO8 | Active low. On while any sensor is occupied |

GPIO2 is skipped because it's a strapping pin (pulling it low at boot can stop the board from booting). GPIO5 is skipped because it's on ADC2, which is unreliable on the C3.

The main loop runs at 100 Hz. Each raw reading is scaled to 0–100% of that sensor's calibrated span (0% = fully pressed, 100% = unpressed), then passed through a hysteresis filter:

- below 30% → **occupied**
- above 70% → **unoccupied**
- in between → keeps its previous state

Calibration (`SENSORS[]`) and thresholds (`OCCUPIED_BELOW_PCT`, `UNOCCUPIED_ABOVE_PCT`) are at the top of `src/main.cpp`. Status for all four sensors prints over USB serial at 10 Hz, and each state change prints immediately.

### ADC note

With 11 dB attenuation the ESP32-C3 ADC is accurate to about 2.5V and saturates around 2.9V or higher, so readings near the top of the range (sensor barely touched) all read about the same. That's fine for detecting occupancy, which happens toward the low end. If the top end matters, add a divider or a lower-value pullup.

## Build & flash

```sh
pio run -t upload
pio device monitor
```
