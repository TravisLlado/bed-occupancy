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

The main loop samples all four sensors at 100 Hz. Each raw reading (12-bit, 0–4095) goes through a hysteresis filter with its own pair of cutoffs:

- below `low` → **occupied**
- above `high` → **unoccupied**
- in between → keeps its previous state

Defaults are low 1456 and high 2964 (30% / 70% of the tabletop range, about 325 pressed and 4095 unpressed). They're set in `src/config.h`, and you can change them at runtime, per sensor (see below).

## Console (USB serial + web console)

The same commands work over USB serial (`pio device monitor`) and in the browser at **http://bedoccupancy.local/** (or the IP address printed at boot). The web console is a plain terminal page from [WebConsole](https://github.com/TravisLlado/WebConsole): output on a dark background and a command line at the bottom, with Up/Down to recall earlier commands. Both show all four readings once per second, and print each state change as it happens:

```
S0 4095 --- | S1  410 OCC | S2 4095 --- | S3 4095 --- | bed OCCUPIED
S1 -> UNOCCUPIED (raw 3120)
```

| Command | What it does |
|---|---|
| `set <0-3\|all> <low\|high> <value>` | Set a cutoff in raw ADC counts, e.g. `set 2 low 1400`. `low` must stay below `high` |
| `show` | Print every sensor's cutoffs and current reading |
| `defaults` | Restore the default cutoffs on all sensors |
| `help` | List commands |

Cutoffs you set are saved to flash (NVS) and survive reboots and reflashing. Use `defaults` to clear them.

Wi-Fi credentials come from the shared `credentials.h`, found through the `CREDENTIALS_INCLUDE` environment variable (same setup as the blinds and thermostat projects). Sensing doesn't depend on Wi-Fi: if the network is down, the sensors and LED keep working, and the board retries the connection every 60 s.

### ADC note

With 11 dB attenuation the ESP32-C3 ADC is accurate to about 2.5V and saturates around 2.9V or higher, so readings near the top of the range (sensor barely touched) all read about the same. That's fine for detecting occupancy, which happens toward the low end. If the top end matters, add a divider or a lower-value pullup.

## Build & flash

```sh
pio run -t upload
pio device monitor
```
