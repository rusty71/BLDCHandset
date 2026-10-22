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

void vbatMessage(crsf_sensor_battery_t *vbatSensor){
  Serial.print('.');
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
}

void loop() {
  float angle;
  // static uint32_t looptime = micros();
  static uint32_t channelstime = micros();
  static crsf_channels_t channels;
  static uint32_t target = 172;
  // uint8_t ping_msg[] = {0xee, 4, 0x28, 0x0, 0xe};

  crsf.loop();
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

