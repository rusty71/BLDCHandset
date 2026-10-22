# BLDCHandset

A CRSF peripheral for ELRS radios. It reads an **absolute angle from an AS5048A
magnetic encoder** and **injects it into the ELRS link as an RC channel**, while
keeping its own transmissions **phase-locked to the radio's uplink** so the two
never collide on the shared CRSF bus.

Runs on a **Waveshare RP2040 Zero** (Raspberry Pi Pico) with the Arduino/mbed
framework, using [SimpleFOC](https://simplefoc.dev/) for the motor side and a
local fork of the CRSF protocol library.

---

## How it works

```
   ELRS radio  ─────── CRSF 420 kbaud ───────►  BLDCHandset (RP2040)
      │  addr 0xEA                                  │  addr 0xC8
      │  0x08 battery      @  8 ms                  │  0x16 RC channels @ 4 ms
      │  0x14 link stats   @ 240 ms                 │  (injects the encoder angle
      │  0x3A radio id     @ 200 ms  ── sync ──►   │   as a stick axis)
      └─────────────────────────────────────────────┴──────► ELRS receiver
```

The handset impersonates the **flight controller** (`0xC8`) so the radio's CRSF
output to the ELRS receiver carries the injected axis, and it consumes the
radio's own frames (`0xEA`) for telemetry and timing.

### The ELRS / CRSF phase lock

The radio publishes its uplink period and a phase request inside the `0x3A`
radio-ID frame (`crsf_opentx_sync_t`, both fields in units of 0.1 µs):

| field           | typical value | meaning                       |
| --------------- | ------------- | ----------------------------- |
| `packet_inteval` | 40000       | uplink period = 4000 µs       |
| `phase_shift`    | ~20000      | requested phase = 2000 µs     |

The handset servos its **own transmit period** so that its frames land in the
middle of the radio's uplink slots:

```cpp
packet_rate = htonl(TXsync->packet_inteval) / 10;   // 40000 -> 4000 µs
phase_shift = htonl(TXsync->phase_shift) / 10;       // raw 0.1 µs
Input = (packet_rate / 2) - phase_shift;             // error vs. half period
syncPID.Compute();                                   // P-on-measurement PID
packet_rate_adj = Output;                            // +/- 100 µs
// effective TX period = packet_rate + packet_rate_adj
```

Targeting `phase_shift == packet_rate / 2` places the handset's frames exactly
half an uplink period away from the radio's — the maximum possible clearance.

---

## Hardware

| Function                | RP2040 pin | Notes                              |
| ----------------------- | ---------- | ---------------------------------- |
| CRSF UART (Serial1)     | GP4 TX / GP5 RX | 420 000 baud, shared CRSF bus |
| Encoder SPI (MISO/MOSI/SCK) | 12 / 11 / 10 | AS5048A, SPI_MODE1 @ 10 MHz   |
| Encoder CS              | 13         |                                    |
| Motor driver 3-PWM      | 3, 2, 6, 7 |                                   |
| Debug serial (USB)      | —          | 115 200 baud                       |

Motor: 7 pole pairs. Supply: 12 V. Driver PWM: 24 kHz.

> The CRSF bus is half-duplex: TX and RX are tied to the same wire. The radio
> side needs a proper CRSF transceiver/level shifting — see the
> [CRSF hardware notes](https://github.com/ExpressLRS/crsf_protocol).

---

## Build

```bash
pio run                # build
pio run -t upload      # flash
pio device monitor     # 115200 baud debug output
```

Set `upload_port` / `monitor_port` in `platformio.ini` if you want to pin the
port to a specific board (the file ships with them commented out).

---

## Repository layout

```
src/main.cpp                    active sketch
variants/                       version snapshots of earlier sketches
lib/CrsfSerial/                 CRSF protocol library (local fork)
lib/crc8/                       CRC-8 (poly 0xD5) used by CRSF
serplot.py                      captures the debug serial stream to test.csv
elrs.ipynb                      CRSF timing analysis notebook
test_crsfshot*.csv              captured CRSF timing traces (see below)
```

### Captured traces

| file                    | duration | TX frames | radio `phase_shift` (median) | vs. target |
| ----------------------- | -------- | --------- | ---------------------------- | ---------- |
| `test_crsfshot.csv`     | 327 s    | 81 822    | 20002 (= 2000.2 µs)          | +0.2 µs ✅ |
| `test_crsfshot_500.csv` | 232 s    | 57 957    | 4976 (= 497.6 µs)            | −1502 µs   |
| `test_crsfshot_2500.csv`| 210 s    | 52 624    | 24927 (= 2492.7 µs)          | +493 µs    |
| `test_NOcrsfshot.csv`   | 281 s    | 70 162    | 17331, range −3020…38980     | unstable   |

Column format: `timestamp_us, direction, addr, type, length[, value]`, where
direction `<` = received and `>` = transmitted.

Measured transmit quality across all traces: **4000 µs period, jitter σ ≈ 3–5 µs**
(p1–p99 = 3992–4008 µs), with **≥ 1.23 ms clearance** to the radio's frames and
**zero phase drift over 330 s**.

---

## Known issues

These are real, verified from the code and the captured traces. They are
**not** fixed here because each needs hardware validation.

1. **The PID never influenced the transmit period in any capture.** The TX
   period is exactly 4000 µs in *all four* traces, i.e. `packet_rate_adj == 0`,
   even in the trace where the radio reported a 1502 µs phase error. The
   captured firmware predates the current `src/main.cpp` (captures: 2024-11-01,
   source: 2024-11-04), so the sync loop has **not yet been validated on
   hardware**. Re-capture with the current firmware and confirm
   `packet_rate_adj` becomes non-zero.

2. **Channel values are written in the wrong scale.** `channels.ch1 = target`
   puts **microseconds** (1000–2000) into an 11-bit field that CRSF defines as
   a *raw* value (172 = 988 µs … 1811 = 2012 µs). The captured traces show the
   field correctly carrying raw values 172…1811, so the current source
   regressed. Fix with
   `map(target, 988, 2012, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX)`.

3. **All 16 channels are overwritten.** `ch0` is never assigned (so it is 0 =
   stick low) and `ch2…ch15` are set to 1 (= stick low). The radio's real
   stick inputs are destroyed. The fix is to parse the radio's incoming
   `0x16` frame — note `CrsfSerial::processPacketIn` only dispatches `0x16`
   for address `0xC8`, but the radio sends channel frames as `0xEA` — then
   merge and override only the one channel you own.

4. **`motor.loopFOC()` / `motor.move()` are never called.** The motor is only
   initialised and aligned (`initFOC()`); the sketch is an encoder *reader*.

5. **`motor.sensor_offset = motor.shaft_angle;` is a no-op** — it runs before
   `motor.init()`, so `shaft_angle` is still 0. SimpleFOC 2.3.x uses
   `zero_electric_angle` instead.

6. **The debug port is flooded.** `vbatMessage()` prints `.` for every received
   battery frame (125 Hz) onto the same 115200-baud port used for the
   angle/target trace.

7. **PID tuning is misleading.** `br3ttb/PID` defaults `SampleTime` to 100 ms,
   so `ki = Ki × 0.1 = 0.01` — the effective integral gain is 10× lower than
   `Ki = 0.1` suggests. `Compute()` is only called from the sync callback
   (~200 ms), so `dInput` spans 200 ms.

8. **No catch-up clamp on the TX scheduler.** `channelstime += period`
   accumulates nominal rather than elapsed time, so an overrun makes the
   sketch fire frames back-to-back to catch up.

---

## Analysis environment

The notebooks need pandas/numpy/seaborn:

```bash
python3 -m venv .venv
.venv/bin/pip install pandas numpy seaborn jupyterlab pyserial
.venv/bin/jupyter lab
```

`.venv/` is git-ignored — it must never live inside PlatformIO's `lib/`
directory, which is scanned by the library dependency finder.

## Capturing a new trace

1. Remove the `#undef CRSF_PERFORMANCE_CSV` line at the top of
   `lib/CrsfSerial/CrsfSerial.cpp` (it disables the CSV logger).
2. Flash and run `serplot.py` to capture the debug serial stream to `test.csv`.
3. Restore the `#undef`.

Note that the CSV-instrumented build is **not** behaviourally identical to
production — in the Servo fork the channel→µs scaling is wrapped in
`#ifndef CRSF_PERFORMANCE_CSV`.

## License

Public domain / unlicensed — add your preferred license.
