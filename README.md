# Bed Occupancy

Detects whether a bed is occupied using flat-flex (force-sensitive) sensors read by an ESP32-C3.

## Current demo

Four sensors, one LED:

| Signal | Pin | Notes |
|---|---|---|
| Sensor 0 | GPIO0 (ADC1_CH0) | Right side. Each sensor has its own 10k pullup to 3.3V. High = no contact, low = full contact |
| Sensor 1 | GPIO1 (ADC1_CH1) | Right side |
| Sensor 2 | GPIO3 (ADC1_CH3) | Left side |
| Sensor 3 | GPIO4 (ADC1_CH4) | Left side |
| Onboard LED | GPIO8 | Active low. On while either side is occupied |

GPIO2 is skipped because it's a strapping pin (pulling it low at boot can stop the board from booting). GPIO5 is skipped because it's on ADC2, which is unreliable on the C3.

The main loop samples all four sensors at 100 Hz and averages each sensor over 1 s windows (100 samples). Averaging cuts ADC and supply-rail noise, and a 1 s window spans a whole number of 50 Hz and 60 Hz mains cycles, so hum pickup cancels. Once per window, each sensor's average (12-bit, 0–4095) goes through a hysteresis filter with its own pair of cutoffs:

- below `low` → **occupied**
- above `high` → **unoccupied**
- in between → keeps its previous state

Each side of the bed is occupied while either of its two sensors is (sides are assigned in `SENSOR_SIDES` in `src/config.h`). The LED is on while either side is occupied.

Each sensor has its own default cutoffs, chosen from in-bed logs and set in `src/config.h`:

| Sensor | Side | `low` | `high` |
|---|---|---|---|
| S0 | right (primary) | 1300 | 2700 |
| S1 | right (backup) | 1400 | 2700 |
| S2 | left (primary) | 1000 | 2100 |
| S3 | left (backup) | 1700 | 2600 |

You can change them at runtime, per sensor (see below).

## Console (USB serial + web console)

The same commands work over USB serial (`pio device monitor`) and in the browser at **http://bedoccupancy.local/** (or the IP address printed at boot). The web console is a plain terminal page from [WebConsole](https://github.com/TravisLlado/WebConsole): output on a dark background and a command line at the bottom, with Up/Down to recall earlier commands. Once per second, both print one line: the local time the window finished (`--:--:--` until the clock has been set), the average reading of S0–S3, and each side's state:

```
23:14:07 | 4095 1120 3702 4095 | L ---  R OCC
```

| Command | What it does |
|---|---|
| `set <0-3\|all> <low\|high> <value>` | Set a cutoff in raw ADC counts, e.g. `set 2 low 1400`. `low` must stay below `high` |
| `show` | Print every sensor's cutoffs and latest readings |
| `defaults` | Restore the default cutoffs on all sensors |
| `help` | List commands |

Cutoffs you set are saved to flash (NVS) and survive reboots and reflashing. Use `defaults` to clear them.

Wi-Fi credentials come from the shared `credentials.h`, found through the `CREDENTIALS_INCLUDE` environment variable (same setup as the blinds and thermostat projects). Sensing doesn't depend on Wi-Fi: if the network is down, the sensors and LED keep working, and the board retries the connection every 60 s.

### ADC note

With 11 dB attenuation the ESP32-C3 ADC is accurate to about 2.5V and saturates around 2.9V or higher, so readings near the top of the range (sensor barely touched) all read about the same. That's fine for detecting occupancy, which happens toward the low end. If the top end matters, add a divider or a lower-value pullup.

## HTTP interface (for other nodes)

Other boards read the current state with a plain HTTP GET, up to once per second:

```
GET http://bedoccupancy.local/state
```

```json
{"left":false,"right":true,"time":1763512345}
```

| Field | Meaning |
|---|---|
| `left`, `right` | Whether that side of the bed is occupied |
| `time` | When the measurement was taken (end of its 1 s averaging window), Unix seconds UTC. `null` until the clock has been set from NTP after boot |

- Status 200 with `Content-Type: application/json` and `Cache-Control: no-store`.
- Status 503 with `{"error":"starting"}` for the first second after boot, before the first measurement window completes.
- A fresh measurement is published every second, so `time` should never be more than about a second behind the requester's clock. An older `time` means the board has stalled, and the reading shouldn't be trusted.
- Clients that can't resolve mDNS `.local` names can use the IP address printed at boot. A DHCP reservation keeps it fixed.

## Clock

There's no battery-backed clock, so the board sets its time from NTP (`pool.ntp.org`, `time.nist.gov`) once Wi-Fi joins. That's the same approach as the blinds controller, with the same `PST8PDT` timezone rules, so DST is handled onboard. It forces a re-sync every night at 02:00 local, on top of the background re-sync SNTP does every few hours. Unlike the blinds controller, nothing waits on NTP: sensing starts at boot, and `time` is `null` until the first sync. A failed nightly sync is logged and the clock keeps running from its last sync.

## Build & flash

```sh
pio run -t upload
pio device monitor
```
