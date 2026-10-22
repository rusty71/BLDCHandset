#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"
#include <CrsfSerial.h>
#include <PID_v1.h>

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

int packet_rate = 20000;
int packet_rate_adj = 0;
int phase_shift = 0;

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

void vbatMessage(crsf_sensor_battery_t *vbatSensor){
  Serial.print('.');
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
    Serial.print("  link up           : "); Serial.println(crsf.isLinkUp() ? "yes" : "no");
    Serial.print("  packet rate       : "); Serial.print(packet_rate); Serial.println(" us");
    Serial.print("  rate adjustment   : "); Serial.print(packet_rate_adj); Serial.println(" us");
    Serial.print("  phase offset      : "); Serial.print(phase_shift); Serial.println(" (raw)");
    Serial.print("  parameters found  : "); Serial.println(paramCount);
    Serial.println();
}

static char cmdBuf[64];
static uint8_t cmdLen = 0;

static void handleCommand(char *line)
{
    while (*line == ' ') line++;
    if (*line == 0)
        return;

    if (strcmp(line, "h") == 0 || strcmp(line, "help") == 0)
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
    else if (strcmp(line, "d") == 0 || strcmp(line, "status") == 0)
    {
        printStatus();
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

double Setpoint, Input, Output;

double Kp=0.02, Ki=0.1, Kd=0.0;
PID syncPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);

//needs to be normalized for different packet rates!
void opentxSyncMessage(crsf_opentx_sync_t *TXsync){
  packet_rate = (int32_t)htonl(TXsync->packet_inteval)/10;  //micros
  phase_shift = (int32_t)htonl(TXsync->phase_shift)/10;
  Input = ((packet_rate/2)-phase_shift);
  syncPID.Compute();
  packet_rate_adj = Output;
#ifdef CRSFSHOT_DEBUG
  Serial.print(packet_rate+packet_rate_adj);
  Serial.print('\t');
  Serial.print((packet_rate/2)-phase_shift);
  Serial.print('\t');
  Serial.println((int32_t)htonl(TXsync->phase_shift));
#endif //CRSFSHOT_DEBUG
}



void setup() {
  spi2.begin();
  sensor1.init(&spi2);


  crsf.onPacketVBat = &vbatMessage;
  crsf.onPacketSync = &opentxSyncMessage;
  crsf.onPacketDeviceInfo = &deviceInfoMessage;
  crsf.onPacketParamEntry = &paramEntryMessage;
  crsf.onPacketParamWriteAck = &paramWriteAck;
  Setpoint = 0;
  syncPID.SetMode(AUTOMATIC);
  syncPID.SetOutputLimits(-100,100);


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

  motor.sensor_offset = motor.shaft_angle;
  motor.init();
  motor.initFOC();

   _delay(100);
  printHelp();
  Serial.print("> ");
}

void loop() {
  float angle;
  // static uint32_t looptime = micros();
  static uint32_t channelstime = micros();
  static crsf_channels_t channels;
  static uint32_t target = 172;
  // uint8_t ping_msg[] = {0xee, 4, 0x28, 0x0, 0xe};

  crsf.loop();
  pollConsole();
  pollScan();
  if( (micros() - channelstime) > (uint32_t)(packet_rate+packet_rate_adj) ) {
    //send channels
    sensor1.update();
    angle = sensor1.getAngle();
    target =   (angle - -3.0f) * (2000 - 1000) / (3.0f - -3.0f) + 1000;
    if(target < 1000)
      target = 1000;
    if(target > 2000)
      target = 2000;




    Serial.print(angle);
    Serial.print('\t');
    Serial.println(target);

    channels.ch1 = target;
    channels.ch2 = 1;
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

    channelstime = channelstime + ((packet_rate+packet_rate_adj));
    crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &channels, sizeof(channels));
    // Serial.println(sizeof(channels));
  }

}
