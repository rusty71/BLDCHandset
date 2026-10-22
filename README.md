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
  │  BLDC ── holds    │ ◄── 0x08 telemetry ────│ ELRS receiver│
  │  the wheel angle  │      (via the radio)  │              │
  └───────────────────┘                       └──────┬───────┘
                                                       │ 0x16 RC channels
                                                       ▼
                                                ┌──────────────┐
                                                │  BLDCServo   │
                                                │  BLDC ─► RC car│
                                                └──────────────┘
```

Both motors run in **position mode**, coupled both ways:

- the **handset's wheel** carries its angle as the channel. One full turn of
  the wheel spans the servo's whole calibrated travel, with the wheel's boot
  position in the middle of that travel (see
  [The channel mapping](#the-channel-mapping)).
- the **servo** position-locks to that channel, over the travel it measured at
  its own boot
- the servo's reported position is added to the wheel's target, so the wheel
  follows the servo too: turning either one turns the other, like a shaft
  between them. See [Two-way coupling](#two-way-coupling).

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
> p <deg>    set the motor trim angle (use g 0 first - see 'g')
> o          re-measure the servo position offset and couple the two
> v <volts>  motor torque limit (0.5 .. 12, default 4 = light)
> q <P>      position gain (default 15: higher follows faster)
> g          show the servo follow gain
> g <gain>   set the servo follow gain (rad of wheel per unit, 0 .. 6.28)
>            (0 = uncoupled, 3.14 = one-to-one)
> s <us>     force the channel to <us> to park the servo (0 = off)
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

The target is the trim, optionally plus the servo's reported position:

```cpp
motor.target = motor_target_rad +
               follow_gain_rad * (servo_position - servo_position_zero);
```

`servo_position` is `-1 .. +1`, decoded from the telemetry frame the servo
sends (see below), and `servo_position_zero` is the offset measured at boot
(see [Two-way coupling](#two-way-coupling)). `follow_gain_rad` defaults to
**PI**, which makes the wheel follow the servo one-to-one over its calibrated
travel, while the servo follows the wheel's angle. The trim and the gain are
set from the menu (`p <deg>` and `g <gain>`).

So each motor's setpoint is the other board's sensor reading, relayed over
ELRS: the servo's target is `its travel scaled by the wheel's encoder angle`
and the wheel's target is `the servo's encoder angle scaled by PI`. Both
motors run the same SimpleFOC angle loop; only the source of the setpoint
differs.

### The channel mapping

The channel carries the wheel's angle **measured from where it booted**, not
the raw encoder angle (whose zero is arbitrary and differs from board to
board). One full turn of the wheel is the servo's whole travel, with the boot
position in the *middle* of that travel:

```cpp
float angle = motor.shaft_angle;      // radians, 0 at boot
if (angle >  PI) angle =  PI;         // clamp at +-180 deg
if (angle < -PI) angle = -PI;
target = 1500 + (angle / PI) * 500;   // 1000 .. 2000 us
```

| wheel (from boot) | channel | servo |
| ----------------- | ------- | ----- |
| -180 deg | 1000 us | `min_angle` (stop) |
|  -90 deg | 1250 us | 25 % of travel |
|    0 deg | 1500 us | `center_angle` |
|  +90 deg | 1750 us | 75 % of travel |
| +180 deg | 2000 us | `max_angle` (stop) |

Measured on hardware against the bench servo (travel 104.6 deg): 1500 us gave
2.1120 rad, 1599 us gave 2.2948 rad and 1399 us gave 1.9297 rad - the table
above, to within the channel quantisation.

The mapping is **clamped, not wrapped**. A circle cannot be mapped
continuously onto a limited travel, so a wrapped channel has one seam where it
jumps from 2000 back to 1000 and the servo has to cross its entire travel in
one step. With the wheel following the servo's position back, that step is fed
straight into the wheel as a 180 deg jolt. Clamping puts the same limit at
+-180 deg, but there the servo simply sits on its stop and the wheel is pulled
back, so the pair has smooth virtual end stops instead of a seam.

### The position telemetry

The servo reports its position in the **battery sensor frame** (`0x08`), using
the whole 16 bit voltage field:

| raw value | servo position |
| --------- | -------------- |
| `0` | `min_angle` |
| `65535` | `max_angle` |
| `32767` / `32768` | `center_angle` |

The handset decodes it as `(raw - 32767.5) / 32767.5` to `-1 .. +1`. The
frame's values are big endian on the wire, so the little-endian bitfield read
is swapped back with `be16toh()`.

The full 16 bits are used because the coupling loop is **unity gain**: every
quantisation step left in this number becomes a permanent position error,
which the loop integrates. At the 11 bit CRSF channel scale one step was
0.0018 of the travel = 0.006 rad of wheel, and the wheel crawled away from
where it was left within seconds (measured: 0 to -0.40 rad in 20 s, and 1.5
rad in 35 s on another boot). At 16 bit a step is 0.00003 of the travel,
which is below the encoder's own resolution.

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

### Two-way coupling

The wheel commands the servo (its angle is the channel) and the servo's
reported position commands the wheel back. The two paths close a loop around
the RF link:

```
wheel angle -> channel -> servo target -> servo position -> wheel target
```

One full turn of the wheel spans the servo's whole travel, so the servo turns
`travel / 2*PI` rad per radian of wheel motion, and it reports that as a
`-1 .. +1` position over its own travel. Per radian of wheel motion the wheel
target therefore comes back by

```
d(target)/d(wheel) = (travel / 2*PI) * (2 / travel) * follow_gain = follow_gain / PI
```

so the loop gain is `follow_gain / PI`. At `follow_gain = PI` that is exactly
1: each motor holds the position the other one reports, and the pair behaves
like a shaft between them - **neutral**, so it stays wherever it is put.

That equality has to hold to much better than a percent, because at unity the
loop integrates any error it is given. Two things break the equality, and both
are handled:

1. **A constant offset.** The servo is commanded 1500 us, which is 991.5
   channel counts, and it settles wherever its own loop leaves it - a fraction
   of a count away from the middle of its travel. So the position it reports
   when the wheel is on its boot mark is not exactly `0`. At unity gain a
   constant offset is integrated: the wheel creeps away from where it was left
   and only stops when something else pins it. The handset therefore measures
   the offset at boot - the wheel is parked on its zero, so the servo is being
   commanded its centre and whatever it reports is the offset - and subtracts
   it. The coupling only starts once the offset is known, because until then
   the offset itself is what would move the wheel. `o` re-measures it.

2. **Quantisation in the feedback.** See
   [The position telemetry](#the-position-telemetry): the position report is
   16 bit for exactly this reason. At 11 bit the residual after the offset
   measurement was still one step, which is a permanent 0.006 rad error of
   wheel - the loop walked it to a stop.

3. **A seam in the channel.** See
   [The channel mapping](#the-channel-mapping): clamped, not wrapped.

Measured on the pair after those three fixes:

| test | result |
| ---- | ------ |
| servo parked at 1600 us (+20 % travel) | wheel to **+0.63 rad**, in ~0.3 s, then held |
| servo parked at 1400 us (-20 % travel) | wheel to **-0.62 rad** |
| servo parked at 1500 us (centre) | wheel to **0.00 rad** |
| wheel left alone for 40 s, coupled | **0.00 rad, no drift** |
| wheel left alone for 30 s at -2.92 rad | **-2.92 rad, no drift** |
| `p 10` pulsed for 0.8 s (a push, then release) | wheel **+0.80 rad**, servo followed 1574 -> 1701 us, wheel **stayed** at the new position after the release |
| `s 2000 / 1000 / 1500` sweep | +3.14 / -3.14 / 0.00 rad, no seam |

The handset's own angle is the one transmitted, so the forward direction is
the direct one; the reverse direction is the servo's telemetry closing the
loop. A 20 % servo travel maps to a 20 % wheel turn (72 deg of wheel), i.e.
the servo's full 104 deg stroke is the wheel's full turn.

The two directions do not have the same bandwidth, and that is what the
servo's stiffness controls (its menu `v` and `q`):

* Turning the **servo** by hand: its sensor moves, the wheel's target moves,
  and the wheel (light, 4 V) follows in ~0.3 s.
* Turning the **wheel** by hand: its sensor moves, the servo's target moves,
  and the servo has to get there. If it is too soft (6 V / P_angle 8 was
  tried) the wheel's own position loop pulls back before the servo arrives and
  the pair does not appear to move at all. 12 V / P_angle 20 (the default) is
  fast enough.

`g 0` uncouples the two (the wheel then holds its trim, `p <deg>`); `g` values
between 0 and PI make the wheel spring back towards its boot position instead
of tracking the servo, so the default is PI.

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

The **pair** (handset -> radio -> servo) was then checked by commanding the
wheel with the `p` trim and reading back the servo's own report (bench servo,
travel 103.9 deg):

| wheel command | channel sent | servo target | servo position |
| ------------- | ------------ | ------------ | -------------- |
| `p 0`     | 1500 us | 2.1228 rad (centre) | 2.1230 |
| `p 90`    | 1750 us | 2.5760 rad (75 %)   | 2.5771 |
| `p -90`   | 1250 us | 1.6713 rad (25 %)   | 1.6720 |
| `p 179`   | 1996 us | 3.0239 rad (max)    | 3.0250 |
| `p -179`  | 1001 us | 1.2198 rad (min)    | 1.2210 |

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

1. **The battery telemetry is hijacked.** The `0x08` frame carries the servo
   position in its voltage field, so the radio's battery readout shows a
   nonsense voltage. Reporting both would need a second telemetry frame.

2. **All channels except 1 and 2 are forced to "stick low".** `ch3...ch15` are
   set to 1 and `ch0` is left at 0, so any other input on those channels is
   lost. This device is the only CRSF source on its bus, so in practice
   nothing else is lost, but it would not work alongside real stick inputs.

3. **`down_lq` reads 0 %.** This ELRS module does not report a downlink LQ, so
   the link statistics frame carries zeros there. The uplink LQ and RSSI are
   real.

4. **The coupling is neutral, not stiff.** At `follow_gain = PI` the wheel
   holds whatever position the pair is in; nothing pulls it back to centre,
   which is how a real shaft behaves. It also has no friction: a small gain
   error creeps over time. The boot offset measurement cancels the systematic
   part, and the position telemetry is 16 bit so that its quantisation is
   below the encoder's resolution (measured: 0.00 rad over 40 s). If the creep
   comes back after a change in the mechanics, `o` re-measures the offset, and
   `g` slightly below PI makes the pair self-centre.

5. **The trim and the coupling fight each other.** `p <deg>` offsets the
   wheel's target, and at unity loop gain a constant offset is integrated, so
   the wheel runs to a stop. Use `g 0` first if the wheel has to be moved from
   the menu. A *pulsed* `p` (set it, wait, set it back to 0) is a convenient
   way to push the wheel from the console for tests: it is a torque pulse.

6. **The two directions have different bandwidth.** The reverse direction
   (turning the wheel, so the servo has to chase the channel) needs the servo
   to reach its new setpoint before the wheel's own position loop pulls the
   wheel back. That is why the servo runs at 12 V / P_angle 20 by default; its
   `v` and `q` commands trade that against how hard it is to back-drive.

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
