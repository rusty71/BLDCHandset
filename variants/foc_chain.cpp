/**
 * BLDCHandset diagnostic - FOC control chain internals
 *
 * The position controller computes a sensible error but the motor does not
 * move, while the open loop alignment does spin the rotor. This prints the
 * whole control chain so we can see where it breaks:
 *
 *   shaft_angle -> P_angle -> shaft_velocity_sp -> PID_velocity -> voltage.q
 *
 * plus the commutation angle, so we can check it against the rotor.
 *
 * To run it, copy it over src/main.cpp, flash it, then restore:
 *     cp variants/foc_chain.cpp src/main.cpp
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
MbedSPI spi2(SPI_2_MISO, SPI_2_MOSI, SPI_2_SCK);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 2, 6, 7);

void setup()
{
    Serial.begin(115200);
    _delay(1500);
    Serial.println();
    Serial.println("=== FOC control chain diagnostic ===");
    Serial.println("send any character on the serial port to start");
    while (!Serial.available())
        _delay(10);
    Serial.read();

    spi2.begin();
    sensor1.init(&spi2);

    motor.linkSensor(&sensor1);
    driver.voltage_power_supply = 12;
    driver.pwm_frequency = 24000;
    driver.init();
    motor.linkDriver(&driver);
    motor.controller = MotionControlType::angle;
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 10.0f;
    motor.PID_velocity.D = 0.0f;
    motor.voltage_limit = 12;
    motor.LPF_velocity.Tf = 0.01f;
    motor.init();

    Serial.println("[1] alignment");
    bool ok = motor.initFOC();
    Serial.print("    initFOC=");
    Serial.println(ok ? "OK" : "FAILED");
    Serial.print("    sensor_direction=");
    Serial.println(motor.sensor_direction);
    Serial.print("    zero_elect_angle=");
    Serial.println(motor.zero_electric_angle, 4);

    // settle
    motor.loopFOC();
    motor.move();
    float settle = motor.shaft_angle;
    uint32_t t0 = millis();
    while (millis() - t0 < 2000)
    {
        motor.target = settle;
        motor.loopFOC();
        motor.move();
        _delay(1);
    }

    Serial.println();
    Serial.println("[2] closed loop, target = current + 0.2 rad");
    float target = motor.shaft_angle + 0.2f;
    motor.target = target;
    Serial.println("    target  shaft_ang  shaft_vel  vel_sp   v_q       elec_ang  raw_ang");
    t0 = millis();
    uint32_t last = 0;
    while (millis() - t0 < 3000)
    {
        motor.loopFOC();
        motor.move();
        if (millis() - last > 200)
        {
            last = millis();
            Serial.print("    ");
            Serial.print(target, 4);
            Serial.print("  ");
            Serial.print(motor.shaft_angle, 4);
            Serial.print("  ");
            Serial.print(motor.shaft_velocity, 4);
            Serial.print("  ");
            Serial.print(motor.shaft_velocity_sp, 4);
            Serial.print("  ");
            Serial.print(motor.voltage.q, 4);
            Serial.print("  ");
            Serial.print(motor.electrical_angle, 4);
            Serial.print("  ");
            Serial.println(sensor1.getSensorAngle(), 4);
        }
    }

    Serial.println();
    Serial.println("[3] open loop: drive 3 V for 1 s and watch the raw angle");
    motor.controller = MotionControlType::torque;
    motor.target = 3.0f;
    t0 = millis();
    last = 0;
    while (millis() - t0 < 2000)
    {
        motor.loopFOC();
        motor.move();
        if (millis() - last > 200)
        {
            last = millis();
            Serial.print("    raw_ang=");
            Serial.print(sensor1.getSensorAngle(), 4);
            Serial.print("  v_q=");
            Serial.print(motor.voltage.q, 4);
            Serial.println();
        }
    }
    motor.target = 0.0f;
    _delay(1000);

    Serial.println();
    Serial.println("[4] electrical angle vs rotor");
    motor.controller = MotionControlType::angle;
    motor.loopFOC();
    motor.move();
    Serial.print("    elec_angle      = ");
    Serial.println(motor.electrical_angle, 4);
    Serial.print("    pole_pairs*mech = ");
    Serial.println((float)motor.pole_pairs * sensor1.getSensorAngle(), 4);
    Serial.print("    zero_elec_angle = ");
    Serial.println(motor.zero_electric_angle, 4);
    Serial.println("=== done ===");
    Serial.flush();

    while (1)
        _delay(1000);
}

void loop()
{
}
