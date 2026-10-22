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

The full loop, forward and back:

```
  pilot turns the wheel
        │
        ▼
  ┌───────────────────┐   0x16 RC channels    ┌──────────────┐
  │  BLDCHandset      │ ────────────────────► │  ELRS radio  │
  │  (RP2040)         │      (CRSF 420 kbaud) │  (transmitter)│
  │                   │                       └──────┬───────┘
  │  AS5048A ─► angle │                              │  air
  │                   │                              ▼
  │                   │                       ┌──────────────┐
  │  BLDC ◄─ follows  │ ◄── 0x08 telemetry ────│ ELRS receiver│
  │  the servo        │      (via the radio)  │              │
  └───────────────────┘                       └──────┬───────┘
                                                       │ 0x16 RC channels
                                                       ▼
                                                ┌──────────────┐
                                                │  BLDCServo   │
                                                │  BLDC ─► RC car│
                                                └──────────────┘
```

Both motors run in **position mode**:

- the **servo** position-locks to the channel the handset injected
- the **handset's wheel** position-locks to the position the servo reports back

```
   ELRS radio  ─────── CRSF 420 kbaud ───────►  BLDCHandset (RP2040)
      │  addr 0xEA                                  │  addr 0xC8
      │  0x08 telemetry   @  8 ms  ── servo pos ──► │  0x16 RC channels @ 4 ms
      │  0x14 link stats   @ 240 ms                 │  (the wheel angle as a stick)
      │  0x3A HANDSET      @ 200 ms  ── timing ──►  │
      │       0x10 TIMING  (rate + phase offset)    │
      │  ← 0x28/0x29/0x2B/0x2C/0x2D parameter menu ─┘
      └─────────────────────────────────────────────┴──────► ELRS receiver
```

The radio forwards the receiver's telemetry back to the handset, which is how
the servo's position gets home.

The handset impersonates the **flight controller** (`0xC8`) so the radio's CRSF
output to the ELRS receiver carries the injected axis, and it consumes the
radio's own frames (`0xEA`) for telemetry and timing.

In ELRS terms this device is a **handset**: `TXModuleEndpoint::handleRaw()`
intercepts the `0x16` frame on the CRSF bus and hands it to the radio as the
RC channel data, which the radio then sends over the RF link.

### The ELRS / CRSF phase lock

The radio publishes its uplink period and a **timing correction** inside the
`0x3A` frame. In ELRS 3.x/4.x this frame is `CRSF_FRAMETYPE_HANDSET` with
sub-command `0x10` (`CRSF_HANDSET_SUBCMD_TIMING`) — the TBS spec calls it
*"0x3A.0x10 Timing Correction (CRSF Shot)"*, aka "RC-sync":

```cpp
uint32_t update_interval;   // LSB = 100ns
int32_t  offset;            // LSB = 100ns, +ve = data came too early, -ve = late
```

ELRS builds it in `src/lib/Handset/CRSFHandset.cpp::sendSyncPacketToTX()`:

| field          | ELRS source                            | value in the captures |
| -------------- | -------------------------------------- | --------------------- |
| `update_interval` | `RequestedRCpacketInterval * 10`    | 40000 → **4000 µs**   |
| `offset`       | `OpenTXsyncOffset - 100 µs`             | 20002 → **2000 µs**   |

`OpenTXsyncOffset` is a 5-sample moving average of the time from the
handset's last CRSF packet to the radio's next RF transmission, measured in
`CRSFHandset::JustSentRFpacket()` (called from the OTA timer callback). The
radio therefore publishes **a measured phase error of the handset's frames**,
not a request.

The sync frame is emitted every **200 ms** (`OpenTXsyncPacketInterval`),
which matches the 0x3A interval in every capture.

### The phase controller

Because `offset` is a *phase* error, the correction is applied to the transmit
**phase** and the period is left exactly at the radio's requested interval:

```cpp
packet_rate      = interval_us;                     // exactly what the radio asked for
phase_integral  += offset_us * PHASE_KI;             // integrate the error
phase_correction  = offset_us * PHASE_KP + phase_integral;   // clamp to +/-PHASE_LIMIT
```

```cpp
uint32_t tx_at = channelstime + (uint32_t)phase_correction;
if ((int32_t)(now - tx_at) >= 0) { ... transmit ...; channelstime += packet_rate; }
```

`channelstime` advances by exactly `packet_rate`, so the packets stay
`interval_us` apart and `phase_correction` only shifts *when* they are sent.
The setpoint is the radio's own measurement — the published `offset` is driven
to zero, which per ELRS means the radio's measured headroom settles at its
100 µs safe margin.

The controller is frozen while a parameter scan is running: the radio
measures the age of our *last* frame, so the menu traffic would otherwise
corrupt the phase measurement.

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
src/main.cpp                    active sketch (SimpleFOC + CRSF phase lock + menu)
variants/                       version snapshots of earlier sketches
lib/CrsfSerial/                 CRSF protocol library (local fork)
lib/crc8/                       CRC-8 (poly 0xD5) used by CRSF
serplot.py                      captures the debug serial stream to test.csv
elrs.ipynb                      CRSF timing analysis notebook
test_crsfshot*.csv              captured CRSF timing traces (see below)
```

The menu and the ELRS parameter protocol live in `src/main.cpp` and
`lib/CrsfSerial/` respectively — the sketch talks to the radio's parameter
table over the same CRSF port it uses for the channel frames.

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

## Menu

There is no display, so the menu lives on the **debug serial port** (115 200
baud). The prompt is `>`, and `h` prints the help:

```
> s          scan the ELRS parameter table (0x28 ping -> 0x29 -> 0x2C per id)
> l          list the discovered parameters
> t          set Telemetry Ratio to the fastest option
> c <id> <v> set parameter <id> to <v>
> r <id>     read parameter <id>
> d          show link / timing status
> p          show the motor trim angle
> p <deg>    set the motor trim angle (from the boot position)
> g          show the servo follow gain
> g <gain>   set the servo follow gain (rad of wheel per unit, 0 .. 3)
> a          toggle the automatic Telemetry Ratio setup
> z          clear the latency statistics
> x          toggle the CRSF shot phase correction
> f <us>     force the channel frame interval (f 0 = follow the radio)
> h          help
```

### Automatic Telemetry Ratio setup

`AUTO_SET_TLM_RATIO` in `src/main.cpp` (default `1`) makes the sketch set the
Telemetry Ratio by itself once the CRSF link comes up:

1. wait for `crsf.isLinkUp()`
2. `0x28` broadcast ping, retry every 2 s until the radio answers
3. walk the parameter table with `0x2C`
4. find **"Telem Ratio"** by name, pick the fastest option, write it with `0x2D`

The `a` command toggles it at runtime, and `t` performs the same step on
demand.

### Why the parameter table is discovered at runtime

The radio exposes its settings over the standard CRSF parameter protocol
(TBS spec `0x28`–`0x2D`):

| frame | direction | purpose |
| ----- | --------- | ------- |
| `0x28` Parameter Ping | → radio | ask the device to announce its parameter table |
| `0x29` Parameter Device Information | ← radio | device name, serial/hardware/firmware id, **parameter count** |
| `0x2B` Parameter Settings (Entry) | ← radio | one parameter definition, chunked |
| `0x2C` Parameter Settings (Read) | → radio | read parameter `<id>`, chunk `<n>` |
| `0x2D` Parameter Value (Write) | → radio | write parameter `<id>`; the radio confirms with a `0x2D` |

ELRS assigns parameter ids in registration order (`registerLUAParameter()` →
`lastLuaField++`), so **the id of "Telem Ratio" changes between ELRS
releases**. The sketch therefore never hard-codes an id: it pings the radio,
reads the parameter count from `0x29`, walks ids `1..N` with `0x2C`, and
matches the entry by **name**.

`Telem Ratio` is a `TEXT_SELECTION` parameter, so its `0x2B` entry carries the
full option list:

```
Std;Off;1:128;1:64;1:32;1:16;1:8;1:4;1:2;Race
```

The value is an **index** into that list, and ELRS maps it straight onto
`expresslrs_tlm_ratio_e`. `t` parses the option list and picks the entry with
the smallest denominator (i.e. `1:2`, the fastest downlink → lowest round-trip
latency), then writes that index with `0x2D`.

Because the option list is read from the radio, this keeps working if ELRS
reorders or renames the entries.

## The position lock

The wheel runs SimpleFOC's angle mode:

```
motor.target  ->  P_angle(target - motor.shaft_angle)  ->  velocity setpoint
               ->  PID_velocity(setpoint - motor.shaft_velocity)  ->  voltage
               ->  loopFOC(): commutation against motor.electricalAngle()
```

The target is the **trim plus the servo's reported position**:

```cpp
motor.target = motor_target_rad + follow_gain_rad * servo_position;
```

`servo_position` is `-1 .. +1`, decoded from the telemetry frame the servo
sends (see below). The trim and the follow gain are set from the menu
(`p <deg>` and `g <gain>`).

### The position telemetry

The servo reports its position in the **battery sensor frame** (`0x08`), in
the raw CRSF channel scale:

| raw value | servo position |
| --------- | -------------- |
| `CRSF_CHANNEL_VALUE_MIN` (172) | `min_angle` |
| `CRSF_CHANNEL_VALUE_MAX` (1811) | `max_angle` |
| the midpoint (991) | `center_angle` |

The handset decodes it with the same constants and converts it to `-1 .. +1`
around the middle. The frame's values are big endian on the wire, so the
little-endian bitfield read is swapped back with `be16toh()`.

The frame's `current` field carries the **latency token** (see below).

### Round trip latency

The handset sends a free running 11 bit token in channel 2 alongside the
position in channel 1. The servo echoes the token it received straight back
in the telemetry frame, so the handset can match the reply against the moment
it sent that token and measure the whole loop on its own clock - no clock
synchronisation between the two boards is needed:

```
handset --CRSF--> radio --air--> receiver --CRSF--> servo
handset <--CRSF-- radio <--air-- receiver <--CRSF-- servo
```

A ring of 32 (token, timestamp) pairs is enough: the tokens repeat every
8.2 s and the ring covers 64 ms at 2 ms per frame.

Measured on the bench with ELRS 3.3.2, Packet Rate `D250`, Telemetry Ratio
`1:2`, ~30 cm between the antennas:

| statistic | round trip |
| --------- | ---------- |
| median | **13 ms** |
| mean | 13.8 ms |
| fastest observed | 8 ms |
| typical spread | 9 - 18 ms |
| occasional outliers | 25 - 60 ms (dropped telemetry frames) |

The distribution is **quantised into 4 ms steps** (modes at roughly 9, 13 and
17 ms), which is exactly the over-the-air packet interval - the round trip
has to wait for uplink and downlink slots, so it can only land on slot
boundaries. Half of that is a fair estimate for one direction, about 6.5 ms.

Two things measurably tighten the spread and both are on by default:

* **Sending at 2 ms instead of 4 ms.** The interval field in the `0x3A`
  timing frame is the rate the radio *measured us* sending at, not a command.
  At 4 ms our newest frame is on average 1.9 ms old when the radio transmits
  it; at 2 ms that drops to 0.5 ms. `packet_rate` is therefore capped at
  `HANDSET_MAX_INTERVAL_US` (2 ms); `f <us>` overrides it and `f 0` follows
  the radio.
* **The phase correction** (`x` toggles it). Switching it off made the median
  worse (16 ms instead of 14 ms) at 4 ms per frame.

The latency histogram is printed by `d` in 1 ms buckets, and `z` clears it.

Not every ELRS build relays `0x14` link statistics to the handset, so the
sketch treats **fresh servo telemetry** as the sign that the link is up
(`linkIsUp()`). That matters for the automatic Telemetry Ratio setup, which
otherwise waits forever for a frame that never arrives.

### Loop gain

The wheel's angle and the servo's angle are mapped onto the same channel, so
the loop gain is

```
loop gain = follow_gain * 2 * (1000 us / 6 rad) / 1000 = follow_gain / 3
```

Anything below 3 rad is stable; the default of 0.5 rad gives a loop gain of
0.17, which is far slower than the RF round trip and therefore comfortably
stable.

Two things are needed for the position lock to work at all:

1. **`motor.loopFOC()` and `motor.move()` must run every iteration.** They
   were missing from `loop()`, so the motor was never commutated and never
   held - `initFOC()` aligned the sensor and then nothing drove the motor.
   The motor, driver and PID setup in `setup()` were dead code.

2. **`motor.sensor_offset = motor.shaft_angle;` has to run *after*
   `motor.initFOC()`.** It was before `motor.init()`, where `shaft_angle` is
   still 0, so it was a no-op. Set after alignment it makes the angle the
   motor booted at read as zero, and with `motor.target` left at 0 the
   position controller holds that position against disturbance.

`initFOC()`'s return value is also checked now: if the alignment fails the
electrical zero is unknown (`zero_electric_angle` stays at `-12345`), the
commutation angle would be garbage, and SimpleFOC leaves the driver disabled.
The sketch now reports the failure instead of silently driving the motor with
a broken commutation.

3. **The single turn encoder's wrap is handled explicitly.** The AS5048A's
   angle output wraps at 2π, and SimpleFOC only notices that when more than
   0.8 of a revolution passes between two sensor updates
   (`Sensor::update()` in `common/base_classes/Sensor.cpp`). A rotor that
   crosses the encoder zero *slowly* is therefore not tracked at all:
   `getAngle()` steps by a whole turn, the position error jumps by 2π and the
   position lock stops driving the motor - or drives it the wrong way. The
   sketch watches the raw angle itself and keeps `motor.sensor_offset` in
   step, so the position the controller sees stays continuous. It only
   compensates when SimpleFOC did *not* track the wrap itself, so the two
   never double up.

## Verified on hardware

Measured on the real board (RP2040 Zero, AS5048A, 7 pole pair motor) with
`variants/hardware_test.cpp`:

| measurement | result |
| ----------- | ------ |
| AS5048A over SPI | mode 1 @ 10 MHz, live angle, 11 bit |
| `initFOC()` | OK, `sensor_direction = 1`, `zero_elect_angle = 5.4027` |
| FOC loop rate | ~2.5 kHz |
| position lock, ±0.15 rad sweep | **worst error 0.0014 rad (0.08 deg)** |
| position lock, step response | settles in well under 250 ms |

The wheel is therefore position locked to within a tenth of a degree, and it
holds position when the target is unchanged.

### Running the hardware test

```bash
cp variants/hardware_test.cpp src/main.cpp
pio run -t upload --upload-port /dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_XXXXXXXX-if00
# then type any character on the serial port to start
git checkout src/main.cpp
```

The other sketches in `variants/` are bench diagnostics:
`diagnostic.cpp` (registers + motor drive), `spi_probe.cpp` (bit banged SPI
register dump), `spi_mode_test.cpp` (SPI mode comparison) and `foc_chain.cpp`
(the control chain internals).

> Use the `/dev/serial/by-id/...` path, never `/dev/ttyACM0` - the numbering
> changes when two boards are plugged in.

## Known issues

These are real, verified from the code and the captured traces. They are
**not** fixed here because each needs hardware validation.

1. **The phase controller has never been validated on hardware.** The
   correction is now applied to the transmit phase (as the TBS spec intends)
   and the setpoint is the radio's own measurement, but the captures are from
   2024-11-01 and predate this change. Re-capture and confirm the published
   `offset` converges towards zero, and tune `PHASE_KP` / `PHASE_KI` /
   `PHASE_LIMIT` in `src/main.cpp` if it does not.

2. **Channel values are written in the wrong scale.** `channels.ch1 = target`
   puts **microseconds** (1000-2000) into an 11-bit field that CRSF defines as
   a *raw* value (172 = 988 us ... 1811 = 2012 us). The captured traces show
   the field correctly carrying raw values 172...1811, so the current source
   regressed. Fix with
   `map(target, 988, 2012, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX)`.

3. **All 16 channels are overwritten.** `ch0` is never assigned (so it is 0 =
   stick low) and `ch2...ch15` are set to 1 (= stick low). The radio's real
   stick inputs are destroyed. The fix is to parse the radio's incoming
   `0x16` frame - note `CrsfSerial::processPacketIn` only dispatches `0x16`
   for address `0xC8`, but the radio sends channel frames as `0xEA` - then
   merge and override only the one channel you own.

4. **The `0x08` payload is not a battery frame.** The sketch reads it as
   `crsf_sensor_battery_t.voltage`, but the values sit in the raw 11-bit
   channel range (172...1811) and match the handset's own transmitted channel
   value in 100 % of 39 403 frames. Whatever the radio is really sending in
   `0x08`, it is not a battery voltage.

5. **`motor.loopFOC()` / `motor.move()` are never called.** The motor is only
   initialised and aligned (`initFOC()`); the sketch is an encoder *reader*.

6. **`motor.sensor_offset = motor.shaft_angle;` is a no-op** - it runs before
   `motor.init()`, so `shaft_angle` is still 0. SimpleFOC 2.3.x uses
   `zero_electric_angle` instead.

7. **The debug port is flooded.** `vbatMessage()` prints `.` for every received
   battery frame (125 Hz) onto the same 115200-baud port used for the
   angle/target trace and the menu.

8. **No catch-up clamp on the TX scheduler.** `channelstime += period`
   accumulates nominal rather than elapsed time, so an overrun makes the
   sketch fire frames back-to-back to catch up.

9. **The handset is probably not on the radio's CRSF handset port.** ELRS
   rewrites byte 0 to `0xC8` on that port (`CRSFHandset::forwardMessage`),
   but the captures show `addr=EA` for received frames. The parameter
   requests in `lib/CrsfSerial` therefore try `dest_addr = 0xEE` and fall
   back to broadcast; confirm which connector the board is really on.

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
