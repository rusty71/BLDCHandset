#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"
#include <CrsfSerial.h>

#define SPI_2_MISO  12
#define SPI_2_MOSI  11
#define SPI_2_SCK   10


#define SENSOR1_CS  13 // some digital pin that you're using as the nCS pin
#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

MagneticSensorSPIConfig_s myAS5048_SPI = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 10000000,
  .bit_resolution = 11,   //Too high causes rumbling? Maybe 10 better than 11. ADC inaccuracy?
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15
};
MagneticSensorSPI sensor1 = MagneticSensorSPI(myAS5048_SPI, SENSOR1_CS);

// MagneticSensorAS5048A sensor1(SENSOR1_CS, true);

MbedSPI spi2(SPI_2_MISO, SPI_2_MOSI, SPI_2_SCK);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 2, 6, 7);
CrsfSerial crsf(Serial1, 420000);

// ---------------------------------------------------------------------------
// Transmit timing
//
// The radio publishes a timing correction frame (TBS spec "0x3A.0x10 Timing
// Correction (CRSF Shot)"), every 200 ms:
//
//   update_interval : the interval it wants us to transmit at
//   offset          : signed phase error, +ve means our data came too early
//
// The radio measures the age of our last frame at the moment of its own RF
// transmission and publishes the error. Correcting it therefore means
// shifting our transmit *phase* - the period stays exactly at the requested
// interval. A PI controller drives the published offset back to zero.
// ---------------------------------------------------------------------------
#define PHASE_KP       0.25f    // proportional gain per correction frame
#define PHASE_KI       0.05f    // integral gain per correction frame
#define PHASE_LIMIT    2500     // max phase correction in us

int packet_rate = 4000;         // us, from the radio's update_interval
int phase_correction = 0;       // us, +ve = transmit later
int phase_integral = 0;         // us, integrator of the phase controller
uint32_t channelstime = 0;      // nominal transmit schedule (micros() domain)

// Set this to 1 to have the sketch set the radio's Telemetry Ratio to the
// fastest option as soon as the link comes up.
#define AUTO_SET_TLM_RATIO  1

// ---------------------------------------------------------------------------
// Encoder wrap handling
//
// The AS5048A is a single turn sensor: its angle output wraps at 2*pi.
// SimpleFOC only notices that wrap when more than 0.8 of a revolution passes
// between two sensor updates (Sensor::update()), so a rotor that crosses the
// encoder zero *slowly* is not tracked at all - getAngle() then steps by a
// whole turn, the position error jumps by 2*pi and the position lock stops
// driving the motor (it would even drive it the wrong way).
//
// Watch the raw angle ourselves and keep motor.sensor_offset in step, so the
// position the controller sees stays continuous across the wrap. If SimpleFOC
// did track the wrap itself we must not compensate twice.
// ---------------------------------------------------------------------------
static float encoder_last_raw = 0.0f;
static int   encoder_last_turns = 0;
static bool  encoder_valid = false;

static void encoderReset()
{
  encoder_last_raw = sensor1.getSensorAngle();
  encoder_last_turns = sensor1.getFullRotations();
  encoder_valid = true;
}

static void encoderCheckWrap()
{
  float raw = sensor1.getSensorAngle();
  int turns = sensor1.getFullRotations();
  if (!encoder_valid) { encoderReset(); return; }

  if (turns == encoder_last_turns)
  {
    float d = raw - encoder_last_raw;
    if (d < -PI)
      motor.sensor_offset -= (float)motor.sensor_direction * (2.0f * PI);
    else if (d > PI)
      motor.sensor_offset += (float)motor.sensor_direction * (2.0f * PI);
  }
  encoder_last_raw = raw;
  encoder_last_turns = turns;
}

// ---------------------------------------------------------------------------
// Round trip latency measurement
//
// The handset sends the sensor position in channel 1 and a free running
// token in channel 2. The servo echoes the token back in its telemetry
// frame, so the handset can match it against the moment it sent it and
// measure the full ELRS round trip on its own clock - no clock sync needed.
//
//   handset --CRSF--> radio --air--> receiver --CRSF--> servo
//   handset <--CRSF-- radio <--air-- receiver <--CRSF-- servo
// ---------------------------------------------------------------------------
#define LAT_RING 32

static uint16_t lat_token[LAT_RING];      // token that was sent
static uint32_t lat_sent_us[LAT_RING];    // when it was sent
static uint16_t tx_token = 0;
static uint32_t lat_samples = 0;
static uint32_t lat_min = 0xFFFFFFFF, lat_max = 0;
static double   lat_sum = 0;
// Round trip histogram in 1 ms buckets, so the shape of the distribution is
// visible without logging every single sample.
#define LAT_HIST 64
static uint16_t lat_hist[LAT_HIST];

// ---------------------------------------------------------------------------
// The closed loop
//
//   pilot turns the wheel  ->  handset angle -> ELRS channel -> servo
//   servo position         ->  ELRS telemetry -> handset -> wheel follows
//
// The wheel is position locked to the servo's reported position, so the
// follow gain must keep the loop stable. With the servo's range mapped 1:1
// onto the channel and the wheel's angle mapped onto the same channel, the
// loop gain is FOLLOW_GAIN_RAD * 2 * (1000/6) / 1000 = FOLLOW_GAIN_RAD/3,
// so anything below 3 rad is stable. The default is a long way below that.
//
// motor.sensor_offset was set to the angle the motor booted at, so 0 means
// "stay where we booted" and the trim is an offset from there.
// ---------------------------------------------------------------------------
#define MOTOR_TARGET_LIMIT_DEG  180.0f
// The servo reports its position over its own calibrated travel as -1 .. +1.
// A gain of PI therefore sweeps the wheel through 2*PI rad, i.e. exactly one
// full turn, for the servo's whole travel.
#define FOLLOW_GAIN_FULL_TURN   (2.0f * (float)PI)
#define FOLLOW_GAIN_LIMIT       FOLLOW_GAIN_FULL_TURN

float motor_target_rad = 0.0f;      // manual trim, from the menu
// rad of wheel per unit of servo position. Zero by default: the wheel is the
// master and the servo follows it. A gain of PI makes the wheel follow the
// servo one-to-one, but the servo follows the wheel too, so that is a
// positive feedback loop with unity gain and it runs away.
float follow_gain_rad = 0.0f;
float servo_position = 0.0f;        // -1 .. +1, from the servo's telemetry
uint32_t servo_position_ts = 0;

static int32_t last_offset_us = 0;  // measured CRSF shot phase error
static bool    phase_enabled = true;      // runtime switch, menu 'x'
static int32_t forced_interval_us = 0;    // 0 = follow the radio, menu 'f'

// Defined with the automatic Telemetry Ratio logic below.
static bool linkIsUp();

// ---------------------------------------------------------------------------
// ELRS parameter / menu support
//
// The radio exposes its settings over the CRSF parameter protocol (0x28..0x2D).
// Parameter ids are assigned in registration order and therefore change
// between ELRS releases, so the table is discovered at runtime instead of
// being hard coded.
// ---------------------------------------------------------------------------

#define MAX_PARAMS 64

struct ParamSlot {
    bool     used;
    uint8_t  id;
    uint8_t  type;
    bool     hidden;
    char     name[CRSF_PARAM_NAME_MAX];
    char     options[CRSF_PARAM_OPTIONS_MAX];
    uint8_t  value;
    uint8_t  min;
    uint8_t  max;
    uint8_t  def;
};

static ParamSlot params[MAX_PARAMS];
static uint8_t   paramCount = 0;
static bool      deviceInfoSeen = false;
static crsf_device_info_t deviceInfo;

static bool  scanInProgress = false;
static uint8_t scanNextId = 1;
static uint8_t scanLastId = 0;
static uint32_t scanTimer = 0;

enum {
    AUTO_WAIT_LINK = 0,   // waiting for the CRSF link to come up
    AUTO_WAIT_INFO,       // pinged, waiting for the device information
    AUTO_SCANNING,        // walking the parameter table
    AUTO_DONE
};

static bool     autoTlmEnabled = AUTO_SET_TLM_RATIO;
static uint8_t  autoTlmState = AUTO_WAIT_LINK;
static uint32_t autoTlmTimer = 0;

static void paramsClear()
{
    memset(params, 0, sizeof(params));
    paramCount = 0;
}

// Copy the option string of a semicolon separated list by index into out.
// Returns false when the index does not exist.
static bool optionAt(const char *options, uint8_t index, char *out, size_t outLen)
{
    if (outLen == 0)
        return false;
    const char *p = options;
    for (uint8_t i = 0; i < index; i++)
    {
        p = strchr(p, ';');
        if (!p)
        {
            out[0] = 0;
            return false;
        }
        p++;
    }
    const char *end = strchr(p, ';');
    size_t n = end ? (size_t)(end - p) : strlen(p);
    if (n >= outLen)
        n = outLen - 1;
    memcpy(out, p, n);
    out[n] = 0;
    return true;
}

// Number of options in a semicolon separated list
static uint8_t optionCount(const char *options)
{
    uint8_t n = 1;
    for (const char *p = options; *p; p++)
        if (*p == ';')
            n++;
    return n;
}

// Parse "1:N" into N. Returns 0 when the option is not a ratio.
static uint16_t ratioDenominator(const char *option)
{
    if (option[0] != '1' || option[1] != ':')
        return 0;
    uint16_t n = 0;
    for (const char *p = option + 2; *p >= '0' && *p <= '9'; p++)
        n = (uint16_t)(n * 10 + (*p - '0'));
    return n;
}

static ParamSlot *paramFind(const char *name)
{
    for (uint8_t i = 0; i < paramCount; i++)
        if (params[i].used && strcmp(params[i].name, name) == 0)
            return &params[i];
    return NULL;
}

static ParamSlot *paramById(uint8_t id)
{
    for (uint8_t i = 0; i < paramCount; i++)
        if (params[i].used && params[i].id == id)
            return &params[i];
    return NULL;
}

// ---------------------------------------------------------------------------
// CRSF callbacks
// ---------------------------------------------------------------------------

// The servo reports its position in the battery sensor frame's voltage field,
// in the raw CRSF channel scale: 172 = servo at min_angle, 1811 = servo at
// max_angle, the midpoint = servo at centre_angle.
// The servo answers in the battery sensor frame:
//   voltage : the servo's own position, raw CRSF channel scale
//   current : the token it received, echoed back
// The radio sends link statistics (0x14), so this tells us whether the ELRS
// link to the receiver is actually up.
static void linkStats(crsfLinkStatistics_t *ls)
{
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  Serial.print("  crsf: frames=");
  Serial.print(crsf.framesReceived());
  Serial.print(" bytes=");
  Serial.print(crsf.bytesReceived());
  Serial.print("  link: up_lq=");
  Serial.print(ls->uplink_Link_quality);
  Serial.print("%  down_lq=");
  Serial.print(ls->downlink_Link_quality);
  Serial.print("%  up_snr=");
  Serial.print(ls->uplink_SNR);
  Serial.print("  down_snr=");
  Serial.print(ls->downlink_SNR);
  Serial.print("  rf_mode=");
  Serial.println(ls->rf_Mode);
}

// Dump every frame we receive, whatever it is, so an unexpected frame type
// can be identified instead of silently ignored.
static void anyMessage(uint8_t addr, uint8_t type, uint8_t len, const uint8_t *buf)
{

  static uint8_t seen_addr[256], seen_type[256];
  // Print the first few of each addr/type pair, then stay quiet.
  (void)len;
  if (seen_type[type] < 3)
  {
    seen_type[type]++;
    seen_addr[addr] = 1;
    Serial.print("  RX addr=0x");
    Serial.print(addr, HEX);
    Serial.print(" type=0x");
    Serial.print(type, HEX);
    Serial.print(" len=");
    Serial.print(len);
    Serial.print(" hdr=");
    for (uint8_t i = 0; i < 6; i++)
    {
      Serial.print(buf[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
}

void vbatMessage(crsf_sensor_battery_t *vbatSensor){
  // The battery frame carries its values big endian on the wire, so the
  // little endian bitfield read has to be swapped back.
  uint16_t pos_raw = be16toh((uint16_t)vbatSensor->voltage);
  uint16_t token   = be16toh((uint16_t)vbatSensor->current);

  float mid = (CRSF_CHANNEL_VALUE_MIN + CRSF_CHANNEL_VALUE_MAX) / 2.0f;
  float half = (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN) / 2.0f;
  servo_position = ((float)pos_raw - mid) / half;      // -1 .. +1
  servo_position_ts = millis();

  // Match the token against the moment we sent it
  uint8_t idx = (uint8_t)(token % LAT_RING);
  if (lat_token[idx] == token && lat_sent_us[idx] != 0)
  {
    uint32_t rtt = micros() - lat_sent_us[idx];
    lat_samples++;
    lat_sum += rtt;
    if (rtt < lat_min) lat_min = rtt;
    if (rtt > lat_max) lat_max = rtt;
    if (rtt / 1000 < LAT_HIST)
      lat_hist[rtt / 1000]++;
    if (lat_samples % 10 == 0)
    {
      Serial.print("  rtt=");
      Serial.print(rtt / 1000.0f, 2);
      Serial.print(" ms  token=");
      Serial.print(token);
      Serial.print("  servo_pos=");
      Serial.print(servo_position, 3);
      Serial.print("  [min ");
      Serial.print(lat_min / 1000.0f, 2);
      Serial.print("  avg ");
      Serial.print(lat_sum / lat_samples / 1000.0f, 2);
      Serial.print("  max ");
      Serial.print(lat_max / 1000.0f, 2);
      Serial.println(" ms]");
    }
  }
}

static void deviceInfoMessage(crsf_device_info_t *info)
{
    deviceInfo = *info;
    deviceInfoSeen = true;

    Serial.println();
    Serial.println("--- ELRS device information ---");
    Serial.print("  serial number : "); Serial.println(info->serial_number, HEX);
    Serial.print("  hardware id   : "); Serial.println(info->hardware_id, HEX);
    Serial.print("  firmware id   : "); Serial.println(info->firmware_id, HEX);
    Serial.print("  parameters    : "); Serial.println(info->parameters_total);
    Serial.print("  param version : "); Serial.println(info->parameter_version);
    Serial.println();

    // Start walking the parameter table from 1. Parameter 0 is the optional
    // root folder, ELRS does not implement it.
    paramsClear();
    scanInProgress = true;
    scanNextId = 1;
    scanLastId = info->parameters_total;
    scanTimer = millis();
}

static void paramEntryMessage(crsf_param_t *param)
{
    if (param->type == CRSF_PARAM_OUT_OF_RANGE)
    {
        // End of the parameter list
        scanInProgress = false;
        Serial.println("  <end of parameter list>");
        Serial.println();
        return;
    }

    if (paramCount < MAX_PARAMS)
    {
        ParamSlot *slot = &params[paramCount++];
        slot->used = true;
        slot->id = param->id;
        slot->type = param->type;
        slot->hidden = param->hidden;
        strncpy(slot->name, param->name, CRSF_PARAM_NAME_MAX - 1);
        slot->name[CRSF_PARAM_NAME_MAX - 1] = 0;
        strncpy(slot->options, param->options, CRSF_PARAM_OPTIONS_MAX - 1);
        slot->options[CRSF_PARAM_OPTIONS_MAX - 1] = 0;
        slot->value = param->value;
        slot->min = param->min;
        slot->max = param->max;
        slot->def = param->def;
    }
}

static void paramWriteAck(uint8_t id, const uint8_t *value, uint8_t len)
{
    Serial.print("  write ack: id="); Serial.print(id);
    Serial.print(" value=");
    if (len == 1)
        Serial.print(value[0]);
    Serial.println();

    ParamSlot *slot = paramById(id);
    if (slot && len == 1)
        slot->value = value[0];
}

// ---------------------------------------------------------------------------
// Menu
// ---------------------------------------------------------------------------

static void printHelp()
{
    Serial.println();
    Serial.println("=== BLDCHandset menu ===");
    Serial.println("  s          scan the ELRS parameter table");
    Serial.println("  l          list the discovered parameters");
    Serial.println("  t          set Telemetry Ratio to the fastest option");
    Serial.println("  c <id> <v> set parameter <id> to <v>");
    Serial.println("  r <id>     read parameter <id>");
    Serial.println("  d          show link / timing status");
    Serial.println("  p          show the motor target angle");
    Serial.println("  p <deg>    set the motor trim angle (from the boot position)");
    Serial.println("  z          clear the latency statistics");
    Serial.println("  x          toggle the CRSF shot phase correction");
    Serial.println("  f <us>     force the channel frame interval (f 0 = follow the radio)");
    Serial.println("  g          show the servo follow gain");
    Serial.println("  g <gain>   set the servo follow gain (rad of wheel per unit)");
    Serial.println("  a          toggle the automatic Telemetry Ratio setup");
    Serial.println("  h          this help");
    Serial.println();
}

static void listParams()
{
    if (paramCount == 0)
    {
        Serial.println("  (no parameters discovered yet - run 's' first)");
        return;
    }
    Serial.println();
    Serial.println("id  type  value  name");
    for (uint8_t i = 0; i < paramCount; i++)
    {
        ParamSlot *p = &params[i];
        Serial.print(p->id);
        Serial.print("  ");
        if (p->type == CRSF_PARAM_TEXT_SELECTION)
        {
            char cur[CRSF_PARAM_OPTIONS_MAX];
            optionAt(p->options, p->value, cur, sizeof(cur));
            Serial.print("SEL ");
            Serial.print(p->value);
            Serial.print("  ");
            Serial.print(p->name);
            Serial.print("  [");
            Serial.print(cur);
            Serial.print("]");
            const char *all = p->options;
            if (*all)
            {
                Serial.print("  options:");
                while (*all)
                {
                    const char *end = strchr(all, ';');
                    if (!end) end = all + strlen(all);
                    Serial.print(' ');
                    Serial.write((const uint8_t *)all, end - all);
                    all = (*end == ';') ? end + 1 : end;
                }
            }
        }
        else
        {
            Serial.print(p->type);
            Serial.print("   ");
            Serial.print(p->name);
        }
        Serial.println();
    }
    Serial.println();
}

// Pick the option with the highest telemetry rate, i.e. the smallest
// denominator of a "1:N" ratio. Falls back to the last option.
static int8_t fastestOption(const char *options, uint8_t count)
{
    int8_t best = -1;
    uint16_t bestDen = 0xFFFF;
    char opt[CRSF_PARAM_OPTIONS_MAX];
    for (uint8_t i = 0; i < count; i++)
    {
        if (!optionAt(options, i, opt, sizeof(opt)))
            continue;
        uint16_t den = ratioDenominator(opt);
        if (den == 0)
            continue;
        if (best < 0 || den < bestDen)
        {
            best = (int8_t)i;
            bestDen = den;
        }
    }
    if (best < 0)
        return (int8_t)(count ? count - 1 : 0);
    return best;
}

static void setTelemRatio()
{
    ParamSlot *slot = paramFind("Telem Ratio");
    if (!slot)
    {
        // Fall back to a looser match in case the radio renamed it
        for (uint8_t i = 0; i < paramCount; i++)
        {
            if (params[i].used && params[i].type == CRSF_PARAM_TEXT_SELECTION &&
                strstr(params[i].name, "Telem") != NULL)
            {
                slot = &params[i];
                break;
            }
        }
    }
    if (!slot)
    {
        Serial.println("  'Telem Ratio' parameter not found - run 's' first");
        return;
    }
    if (slot->type != CRSF_PARAM_TEXT_SELECTION)
    {
        Serial.println("  'Telem Ratio' is not a text selection parameter");
        return;
    }

    uint8_t count = optionCount(slot->options);
    int8_t best = fastestOption(slot->options, count);
    if (best < 0)
        return;

    char cur[CRSF_PARAM_OPTIONS_MAX];
    char opt[CRSF_PARAM_OPTIONS_MAX];
    optionAt(slot->options, slot->value, cur, sizeof(cur));
    optionAt(slot->options, (uint8_t)best, opt, sizeof(opt));

    Serial.print("  current Telem Ratio : ");
    Serial.println(cur);
    Serial.print("  fastest option      : ");
    Serial.println(opt);
    Serial.print("  index               : ");
    Serial.println(best);

    if ((uint8_t)best == slot->value)
    {
        Serial.println("  already at the fastest ratio");
        return;
    }

    Serial.print("  writing value ");
    Serial.print(best);
    Serial.println(" ...");
    crsf.sendParamWriteU8(slot->id, (uint8_t)best);
}

static void printStatus()
{
    Serial.println();
    // The ELRS module does not always relay 0x14 link statistics, so treat
    // fresh servo telemetry as the authoritative sign of a live link.
    Serial.print("  link up           : "); Serial.println(linkIsUp() ? "yes" : "no");
    Serial.print("  packet rate       : "); Serial.print(packet_rate); Serial.println(" us");
    Serial.print("  phase correction  : "); Serial.print(phase_correction); Serial.println(" us (+ = later)");
    Serial.print("  phase integrator   : "); Serial.print(phase_integral); Serial.println(" us");
    Serial.print("  last phase offset   : "); Serial.print(last_offset_us); Serial.println(" us");
    Serial.print("  phase correction    : "); Serial.println(phase_enabled ? "on" : "off");
    Serial.print("  frame interval      : ");
    if (forced_interval_us) { Serial.print(forced_interval_us); Serial.println(" us (forced)"); }
    else Serial.println("follows the radio");
    Serial.print("  motor trim          : ");
    Serial.print(motor_target_rad * 180.0f / PI);
    Serial.println(" deg from the boot position");
    Serial.print("  servo follow gain   : ");
    Serial.print(follow_gain_rad);
    Serial.print(" rad per unit (");
    Serial.print(follow_gain_rad * 2.0f * 180.0f / PI);
    Serial.println(" deg of wheel over the servo's full travel)");
    Serial.print("  crsf frames/bytes   : ");
    Serial.print(crsf.framesReceived());
    Serial.print(" / ");
    Serial.println(crsf.bytesReceived());
    Serial.print("  round trips         : ");
    Serial.println(lat_samples);
    Serial.print("  rtt min/avg/max     : ");
    if (lat_samples)
    {
        Serial.print(lat_min / 1000.0f, 2); Serial.print(" / ");
        Serial.print(lat_sum / lat_samples / 1000.0f, 2); Serial.print(" / ");
        Serial.print(lat_max / 1000.0f, 2);
    }
    else
    {
        Serial.print("-");
    }
    Serial.println(" ms");
    Serial.println("  rtt histogram (ms : count)");
    for (uint8_t i = 0; i < LAT_HIST; i++)
    {
        if (lat_hist[i] == 0) continue;
        Serial.print("    ");
        if (i < 10) Serial.print(' ');
        Serial.print(i);
        Serial.print(" : ");
        Serial.println(lat_hist[i]);
    }
    Serial.print("  servo position      : ");
    Serial.print(servo_position);
    Serial.print("  (age ");
    Serial.print(millis() - servo_position_ts);
    Serial.println(" ms)");
    Serial.print("  parameters found  : "); Serial.println(paramCount);
    Serial.print("  auto tlm ratio    : "); Serial.println(autoTlmEnabled ? "on" : "off");
    Serial.println();
}

static char cmdBuf[64];
static uint8_t cmdLen = 0;

static void handleCommand(char *line)
{
    while (*line == ' ') line++;
    if (*line == 0)
        return;

    if (strcmp(line, "x") == 0)
    {
        phase_enabled = !phase_enabled;
        if (!phase_enabled)
        {
            phase_correction = 0;
            phase_integral = 0;
        }
        Serial.print("  phase correction ");
        Serial.println(phase_enabled ? "on" : "off");
    }
    else if (strncmp(line, "f ", 2) == 0)
    {
        int32_t us = atoi(&line[2]);
        if (us < 0)
            us = 0;
        forced_interval_us = us;
        Serial.print("  frame interval ");
        if (us == 0)
            Serial.println("follows the radio");
        else
        {
            Serial.print(us);
            Serial.println(" us");
        }
    }
    else if (strcmp(line, "z") == 0)
    {
        lat_samples = 0;
        lat_sum = 0;
        lat_min = 0xFFFFFFFF;
        lat_max = 0;
        memset(lat_hist, 0, sizeof(lat_hist));
        Serial.println("  latency statistics cleared");
    }
    else if (strcmp(line, "h") == 0 || strcmp(line, "help") == 0)
    {
        printHelp();
    }
    else if (strcmp(line, "s") == 0 || strcmp(line, "scan") == 0)
    {
        Serial.println("  pinging ELRS device ...");
        paramsClear();
        deviceInfoSeen = false;
        crsf.sendDevicePing(CRSF_ADDRESS_BROADCAST);
    }
    else if (strcmp(line, "l") == 0 || strcmp(line, "list") == 0)
    {
        listParams();
    }
    else if (strcmp(line, "t") == 0 || strcmp(line, "tlm") == 0)
    {
        setTelemRatio();
    }
    else if (strcmp(line, "a") == 0 || strcmp(line, "auto") == 0)
    {
        autoTlmEnabled = !autoTlmEnabled;
        autoTlmState = AUTO_WAIT_LINK;
        autoTlmTimer = millis();
        Serial.print("  automatic Telemetry Ratio setup : ");
        Serial.println(autoTlmEnabled ? "on" : "off");
    }
    else if (strcmp(line, "d") == 0 || strcmp(line, "status") == 0)
    {
        printStatus();
    }
    else if (line[0] == 'g' && (line[1] == ' ' || line[1] == 0))
    {
        if (line[1] == ' ')
        {
            // strtod, not sscanf("%f"): this newlib build has no float
            // scanf support, so sscanf always failed and "g 0" was rejected.
            char *end = NULL;
            double gain = strtod(line + 2, &end);
            if (end != NULL && end != line + 2)
            {
                if (gain < 0.0f) gain = 0.0f;
                if (gain > FOLLOW_GAIN_LIMIT) gain = FOLLOW_GAIN_LIMIT;
                follow_gain_rad = gain;
                Serial.print("  servo follow gain set to ");
                Serial.print(gain);
                Serial.println(" rad per unit");
            }
            else
            {
                Serial.println("  usage: g <gain>");
            }
        }
        else
        {
            Serial.print("  servo follow gain: ");
            Serial.print(follow_gain_rad);
            Serial.println(" rad per unit");
        }
    }
    else if (line[0] == 'p' && (line[1] == ' ' || line[1] == 0))
    {
        if (line[1] == ' ')
        {
            char *end = NULL;
            double deg = strtod(line + 2, &end);
            if (end != NULL && end != line + 2)
            {
                if (deg < -MOTOR_TARGET_LIMIT_DEG) deg = -MOTOR_TARGET_LIMIT_DEG;
                if (deg >  MOTOR_TARGET_LIMIT_DEG) deg =  MOTOR_TARGET_LIMIT_DEG;
                motor_target_rad = deg * (PI / 180.0f);
                Serial.print("  motor target set to ");
                Serial.print(deg);
                Serial.println(" deg from the boot position");
            }
            else
            {
                Serial.println("  usage: p <degrees>");
            }
        }
        else
        {
            Serial.print("  motor target: ");
            Serial.print(motor_target_rad * 180.0f / PI);
            Serial.println(" deg from the boot position");
        }
    }
    else if (line[0] == 'c' && line[1] == ' ')
    {
        int id = 0, val = 0;
        if (sscanf(line + 2, "%d %d", &id, &val) == 2 && id >= 0 && val >= 0 && val < 256)
        {
            Serial.print("  writing parameter "); Serial.print(id);
            Serial.print(" = "); Serial.println(val);
            crsf.sendParamWriteU8((uint8_t)id, (uint8_t)val);
        }
        else
        {
            Serial.println("  usage: c <id> <value>");
        }
    }
    else if (line[0] == 'r' && line[1] == ' ')
    {
        int id = 0;
        if (sscanf(line + 2, "%d", &id) == 1 && id >= 0 && id < 256)
        {
            Serial.print("  reading parameter "); Serial.println(id);
            crsf.sendParamRead((uint8_t)id, 0);
        }
        else
        {
            Serial.println("  usage: r <id>");
        }
    }
    else
    {
        Serial.println("  unknown command - type 'h' for help");
    }
}

static void pollConsole()
{
    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\r')
            continue;
        if (c == '\n')
        {
            cmdBuf[cmdLen] = 0;
            handleCommand(cmdBuf);
            cmdLen = 0;
            Serial.print("> ");
            continue;
        }
        if (cmdLen < sizeof(cmdBuf) - 1)
            cmdBuf[cmdLen++] = c;
    }
}

// Walk the parameter table one id at a time
static void pollScan()
{
    if (!scanInProgress)
        return;
    if (millis() - scanTimer < 30)
        return;

    scanTimer = millis();
    if (scanNextId > scanLastId)
    {
        scanInProgress = false;
        Serial.println("  scan complete");
        Serial.println();
        return;
    }
    crsf.sendParamRead(scanNextId++, 0);
}

// ---------------------------------------------------------------------------
// Automatic Telemetry Ratio setup
// ---------------------------------------------------------------------------

// Not every ELRS build relays 0x14 link statistics to the handset, so fresh
// servo telemetry is the reliable sign that the link is up.
static bool linkIsUp()
{
    return crsf.isLinkUp() ||
           (servo_position_ts != 0 && (uint32_t)(millis() - servo_position_ts) < 500);
}

static void pollAutoTlm()
{
    if (!autoTlmEnabled || autoTlmState == AUTO_DONE)
        return;

    switch (autoTlmState)
    {
    case AUTO_WAIT_LINK:
        if (linkIsUp())
        {
            Serial.println("  link up, querying ELRS parameters ...");
            crsf.sendDevicePing(CRSF_ADDRESS_BROADCAST);
            autoTlmState = AUTO_WAIT_INFO;
            autoTlmTimer = millis();
        }
        else if (millis() - autoTlmTimer > 5000)
        {
            // Keep trying - the radio may still be booting
            autoTlmTimer = millis();
        }
        break;

    case AUTO_WAIT_INFO:
        if (deviceInfoSeen)
        {
            autoTlmState = AUTO_SCANNING;
        }
        else if (millis() - autoTlmTimer > 2000)
        {
            Serial.println("  no answer, retrying ...");
            crsf.sendDevicePing(CRSF_ADDRESS_BROADCAST);
            autoTlmTimer = millis();
        }
        break;

    case AUTO_SCANNING:
        if (!scanInProgress)
        {
            Serial.println("  parameter table read, applying Telemetry Ratio ...");
            setTelemRatio();
            autoTlmState = AUTO_DONE;
        }
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Timing correction (TBS spec "0x3A.0x10 Timing Correction (CRSF Shot)")
// ---------------------------------------------------------------------------

void opentxSyncMessage(crsf_opentx_sync_t *TXsync)
{
    // Both fields are big endian and in units of 100 ns.
    int32_t interval_us = (int32_t)htonl(TXsync->packet_inteval) / 10;
    int32_t offset_us   = (int32_t)htonl(TXsync->phase_shift) / 10;
    last_offset_us = offset_us;

    // The interval field echoes the rate the radio measured us sending at,
    // so it is a measurement, not a command. Match the over-the-air rate
    // instead of half of it, or our newest frame is always half an interval
    // old by the time the radio transmits it.
    // Cap the interval at 2 ms: the radio echoes the rate it measured us
    // sending at, and at 250 Hz the newest frame is already half an interval
    // old by the time it goes out. Sending at 500 Hz halves that, which
    // measured as a tighter round trip distribution.
    #define HANDSET_MAX_INTERVAL_US 2000
    if (forced_interval_us > 0)
        packet_rate = (uint32_t)forced_interval_us;
    else if (interval_us < HANDSET_MAX_INTERVAL_US)
        packet_rate = (uint32_t)interval_us;
    else
        packet_rate = HANDSET_MAX_INTERVAL_US;

    if (!phase_enabled)
    {
        phase_correction = 0;
        phase_integral = 0;
        return;
    }

    // offset > 0 means our data was already this old when the radio sent it,
    // so transmit later. The correction shifts the phase; the period stays at
    // the requested interval. Parameter traffic perturbs the radio's phase
    // measurement, so the controller is frozen while a scan is running.
    if (!scanInProgress)
    {
        phase_integral += (int32_t)(offset_us * PHASE_KI);
        // Clamp the integrator itself: otherwise it winds up without bound
        // once the correction saturates, and the saturated phase shift is
        // pure added latency.
        if (phase_integral > PHASE_LIMIT)
            phase_integral = PHASE_LIMIT;
        if (phase_integral < -PHASE_LIMIT)
            phase_integral = -PHASE_LIMIT;
        int32_t corr = (int32_t)(offset_us * PHASE_KP) + phase_integral;
        if (corr > PHASE_LIMIT)
            corr = PHASE_LIMIT;
        if (corr < -PHASE_LIMIT)
            corr = -PHASE_LIMIT;
        phase_correction = corr;
    }

#ifdef CRSFSHOT_DEBUG
    Serial.print(packet_rate);
    Serial.print('\t');
    Serial.print(offset_us);
    Serial.print('\t');
    Serial.print(phase_correction);
    Serial.print('\t');
    Serial.println(phase_integral);
#endif //CRSFSHOT_DEBUG
}

void setup() {
  spi2.begin();
  sensor1.init(&spi2);


  crsf.onPacketVBat = &vbatMessage;
  crsf.onPacketSync = &opentxSyncMessage;
  crsf.onPacketLinkStatistics = &linkStats;
  crsf.onPacketAny = &anyMessage;
  crsf.onPacketDeviceInfo = &deviceInfoMessage;
  crsf.onPacketParamEntry = &paramEntryMessage;
  crsf.onPacketParamWriteAck = &paramWriteAck;

  crsf.begin();
  crsf.onPacketVBat = &vbatMessage;
  Serial.begin(115200);
  motor.linkSensor(&sensor1);

  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 24000;
  driver.init();

  motor.linkDriver(&driver);
  motor.controller = MotionControlType::angle;
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I =10;
  motor.PID_velocity.D = 0.0;
  motor.voltage_limit = 12;
  motor.LPF_velocity.Tf = 0.01;

  motor.init();
  if (!motor.initFOC())
  {
    // Alignment failed: the electrical zero is unknown, so the motor has to
    // stay disabled - driving it now would use a bogus commutation angle.
    // initFOC() has already disabled the driver.
    Serial.println("FOC alignment FAILED - motor left disabled.");
    Serial.println("check the supply, the driver wiring and the pole pair count");
  }
  else
  {
    // Hold the position the motor booted at: make the current angle read as
    // zero and leave the target at zero, so the position controller keeps it
    // there against disturbance.
    motor.loopFOC();
    motor.move();
    motor.sensor_offset = motor.shaft_angle;
    encoderReset();
  }

  // Start the transmit schedule from now
  channelstime = (uint32_t)micros();

   _delay(100);
  printHelp();
  Serial.print("> ");
}

void loop() {
  float angle;
  static crsf_channels_t channels;
  static uint32_t target = 172;

  // FOC: commutate the motor and hold its position. This has to run every
  // iteration - without it the motor is never commutated and never held.
  // The wheel holds the trim. follow_gain_rad (menu 'g') optionally adds the
  // servo's reported position on top of that.
  motor.target = motor_target_rad + follow_gain_rad * servo_position;
  motor.loopFOC();
  encoderCheckWrap();
  motor.move();

  crsf.loop();
  pollConsole();
  pollScan();
  pollAutoTlm();

  // Once a second, report the state of the CRSF link on this side. If the
  // byte counter stays at 0 then nothing at all is arriving on the CRSF RX
  // pin, which is a wiring / power problem rather than a protocol problem.
  static uint32_t health_timer = 0;
  if ((uint32_t)(millis() - health_timer) >= 1000)
  {
    health_timer = millis();
    Serial.print("HEALTH frames=");
    Serial.print(crsf.framesReceived());
    Serial.print(" bytes=");
    Serial.print(crsf.bytesReceived());
    Serial.print(" link=");
    Serial.print(crsf.isLinkUp() ? "up" : "down");
    Serial.print(" servo_pos=");
    Serial.print(servo_position, 3);
    Serial.print(" age=");
    Serial.print((uint32_t)(millis() - servo_position_ts));
    Serial.print(" rtt=");
    Serial.println(lat_samples);
  }

  // The transmit instant is the nominal schedule plus the phase correction.
  // The nominal schedule advances by exactly the radio's requested interval,
  // so the correction shifts the phase without changing the rate.
  // micros() is 32 bit and wraps every ~71 minutes, so all the time
  // arithmetic below is done on wrapped 32 bit values.
  uint32_t now = (uint32_t)micros();
  uint32_t tx_at = channelstime + (uint32_t)phase_correction;
  if ((int32_t)(now - tx_at) >= 0) {                    // now >= tx_at
    //send channels
    sensor1.update();
    // The wheel's angle relative to where it booted, not the raw encoder
    // angle: the encoder's zero is arbitrary and differs from board to board,
    // so using it directly would put the boot position at an arbitrary point
    // of the servo's travel.
    angle = motor.shaft_angle;
    // Map one full turn of the wheel onto the servo's calibrated travel, with
    // the boot position in the middle of that travel: -180 deg is the servo's
    // min stop, 0 deg its centre and +180 deg its max stop.
    float turn = fmodf(angle, 2.0f * PI);
    if (turn < 0.0f)
      turn += 2.0f * PI;
    float frac = turn / (2.0f * PI) + 0.5f;      // 0.5 at boot, wraps at 1.5
    target = 1000.0f + frac * 1000.0f;
    if (target >= 2000.0f)
      target -= 1000.0f;                          // wrap into the channel range
    if(target < 1000)
      target = 1000;
    if(target > 2000)
      target = 2000;




    // Rate limited trace: the scheduler runs at the radio's packet rate, so
    // printing every frame floods the console.
    static uint32_t trace_timer = 0;
    if ((uint32_t)(millis() - trace_timer) >= 100)
    {
      trace_timer = millis();
      Serial.print(angle);
      Serial.print('\t');
      Serial.println(target);
    }

    // The CRSF channel field is a raw 11 bit value, not microseconds.
    // 191 = 1000 us and 1792 = 2000 us, which is exactly what ELRS's own
    // decoder expects (see CrsfSerial::packetChannelsPacked).
    channels.ch1 = map(target, 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
    // The token rides along with the position so the servo can echo it back
    tx_token = (uint16_t)((tx_token + 1) & 0x7FF);
    uint8_t tidx = (uint8_t)(tx_token % LAT_RING);
    lat_token[tidx] = tx_token;
    lat_sent_us[tidx] = micros();
    channels.ch2 = tx_token;
    channels.ch3 = 1;
    channels.ch4 = 1;
    channels.ch5 = 1;
    channels.ch6 = 1;
    channels.ch7 = 1;
    channels.ch8 = 1;
    channels.ch9 = 1;
    channels.ch10 = 1;
    channels.ch11 = 1;
    channels.ch12 = 1;
    channels.ch13 = 1;
    channels.ch14 = 1;
    channels.ch15 = 1;
    // channels.ch16 = target;

    channelstime += (uint32_t)packet_rate;
    // If we fell far behind, resynchronise the schedule. The signed
    // comparison keeps this from firing when the schedule is ahead of us.
    if ((int32_t)(now - channelstime) > (int32_t)(packet_rate * 4))
      channelstime = now;

    crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &channels, sizeof(channels));
  }

}
