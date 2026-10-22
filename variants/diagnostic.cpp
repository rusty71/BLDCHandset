/**
 * BLDCHandset diagnostic
 *
 * Reads the AS5048A registers directly so we can tell apart the usual
 * suspects:
 *
 *   0x0A  magnetic field status   (is the magnet detected at all?)
 *   0x0E  angle (14 bit)
 *
 * Phase A prints the registers continuously - TURN THE SHAFT BY HAND and
 * watch whether the angle changes.
 * Phase B drives the motor with a fixed voltage and prints the angle
 * before and after, so we can tell whether the motor and driver work.
 *
 * To run it, copy it over src/main.cpp, flash it, then restore:
 *     cp variants/diagnostic.cpp src/main.cpp
 *     pio run -t upload --upload-port /dev/ttyACM0
 *     git checkout src/main.cpp
 */

#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"

#define SPI_2_MISO  12
#define SPI_2_MOSI  11
#define SPI_2_SCK   10
#define SENSOR1_CS  13

MbedSPI spi2(SPI_2_MISO, SPI_2_MOSI, SPI_2_SCK);
SPISettings settings(10000000, MSBFIRST, SPI_MODE1);

MagneticSensorSPIConfig_s myAS5048_SPI = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 10000000,
  .bit_resolution = 11,
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15
};
MagneticSensorSPI sensor1 = MagneticSensorSPI(myAS5048_SPI, SENSOR1_CS);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 2, 6, 7);

// Raw AS5048A register read, same framing as SimpleFOC but without the
// bit shifting, so we see the raw 16 bits.
static uint16_t as5048aRead(uint16_t reg)
{
    uint16_t command = reg | (1 << 14);          // read bit
    uint8_t cnt = 0;
    uint16_t v = command;
    for (int i = 0; i < 16; i++) { if (v & 1) cnt++; v >>= 1; }
    command |= (uint16_t)((cnt & 1) << 15);        // even parity

    spi2.beginTransaction(settings);
    digitalWrite(SENSOR1_CS, LOW);
    spi2.transfer16(command);
    digitalWrite(SENSOR1_CS, HIGH);
    delayMicroseconds(1);
    digitalWrite(SENSOR1_CS, LOW);
    uint16_t val = spi2.transfer16(0x0000);
    digitalWrite(SENSOR1_CS, HIGH);
    spi2.endTransaction();
    return val;
}

void setup()
{
    Serial.begin(115200);
    _delay(1500);
    Serial.println();
    Serial.println("=== BLDCHandset diagnostic ===");
    Serial.println("send any character on the serial port to start");
    while (!Serial.available())
        _delay(10);
    Serial.read();

    spi2.begin();
    pinMode(SENSOR1_CS, OUTPUT);
    digitalWrite(SENSOR1_CS, HIGH);
    sensor1.init(&spi2);

    // -------------------------------------------------------------------
    // Phase A - sensor
    // -------------------------------------------------------------------
    Serial.println();
    Serial.println("[A] AS5048A registers - TURN THE SHAFT BY HAND");
    Serial.println("    field=0x0A  angle=0x0E");
    Serial.println("    t_ms   field   angle14   ang_deg   simplefoc_deg");
    uint32_t t0 = millis();
    while (millis() - t0 < 12000)
    {
        uint16_t field = as5048aRead(0x0A);
        uint16_t ang   = as5048aRead(0x0E);
        sensor1.update();
        Serial.print("    ");
        Serial.print(millis() - t0);
        Serial.print("   0x");
        Serial.print(field, HEX);
        Serial.print("    0x");
        Serial.print(ang, HEX);
        Serial.print("    ");
        Serial.print((ang & 0x3FFF) * 360.0f / 16384.0f, 2);
        Serial.print("      ");
        Serial.println(ang & 0x3FFF);
        _delay(250);
    }

    // -------------------------------------------------------------------
    // Phase B - motor and driver
    // -------------------------------------------------------------------
    Serial.println();
    Serial.println("[B] motor / driver test");
    motor.linkSensor(&sensor1);
    driver.voltage_power_supply = 12;
    driver.pwm_frequency = 24000;
    driver.init();
    motor.linkDriver(&driver);
    // torque mode with no phase resistance => voltage.q = target (volts)
    motor.controller = MotionControlType::torque;
    motor.voltage_limit = 12;

    float a0 = sensor1.getAngle();
    Serial.print("    angle before: ");
    Serial.println(a0, 4);

    Serial.println("    applying 3.0 V open loop for 2 s ...");
    motor.target = 3.0f;
    t0 = millis();
    while (millis() - t0 < 2000)
    {
        motor.loopFOC();
        motor.move();
        _delay(1);
    }
    motor.target = 0.0f;
    _delay(500);

    float a1 = sensor1.getAngle();
    Serial.print("    angle after : ");
    Serial.println(a1, 4);
    Serial.print("    change      : ");
    Serial.println(a1 - a0, 4);
    Serial.println(a1 - a0 == 0.0f ? "    => the motor did NOT move" : "    => the motor moved");

    // -------------------------------------------------------------------
    // Phase C - FOC alignment, with the movement check made explicit
    // -------------------------------------------------------------------
    Serial.println();
    Serial.println("[C] FOC alignment");
    motor.controller = MotionControlType::angle;
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 10.0f;
    motor.voltage_limit = 12;
    motor.LPF_velocity.Tf = 0.01f;
    motor.init();

    float before = sensor1.getAngle();
    bool ok = motor.initFOC();
    float after = sensor1.getAngle();
    Serial.print("    initFOC       : ");
    Serial.println(ok ? "OK" : "FAILED");
    Serial.print("    angle before  : ");
    Serial.println(before, 4);
    Serial.print("    angle after   : ");
    Serial.println(after, 4);
    Serial.print("    moved         : ");
    Serial.println(after - before, 4);
    Serial.print("    sensor_dir    : ");
    Serial.println(motor.sensor_direction);
    Serial.print("    zero_elec_ang : ");
    Serial.println(motor.zero_electric_angle, 4);
    Serial.println("=== diagnostic done ===");
    Serial.flush();

    while (1)
        _delay(1000);
}

void loop()
{
}
