# Bed Occupancy

Detects whether a bed is occupied using flat-flex (force-sensitive) sensors read by an ESP32-C3.

## Current demo

One sensor, one LED:

| Signal | Pin | Notes |
|---|---|---|
| Flex sensor | GPIO0 (ADC1_CH0) | 10k pullup to 3.3V. 3.3V = no contact, 0V = full contact |
| Onboard LED | GPIO8 | Active low. 1 kHz PWM, 12-bit |

The main loop runs at 100 Hz, reads the sensor voltage (calibrated mV), and sets the LED brightness proportional to it (3.3V → full, 0V → off). Readings print over USB serial at 10 Hz.

### ADC note

With 11 dB attenuation the ESP32-C3 ADC is accurate to about 2.5V and saturates around 2.9V or higher, so readings near the top of the range (sensor barely touched) all read about the same. That's fine for detecting occupancy, which happens toward the low end. If the top end matters, add a divider or a lower-value pullup.

## Build & flash

```sh
pio run -t upload
pio device monitor
```
