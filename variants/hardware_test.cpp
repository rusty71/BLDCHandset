/**
 * BLDCHandset hardware test
 *
 * 1. reads the AS5048A over SPI and prints the raw count and angle
 * 2. runs the SimpleFOC alignment and reports whether it succeeded
 * 3. drives the motor through a series of position targets and prints the
 *    commanded and the measured position for each
 *
 * WARNING: this moves the motor. The FOC alignment alone sweeps one
 * electrical revolution (2*pi / pole_pairs of mechanical travel) before the
 * test even starts.
 *
 * To run it, copy it over src/main.cpp, flash it, then restore:
 *     cp variants/hardware_test.cpp src/main.cpp
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

// How far around the boot position the test sweeps, in radians.
#define SWEEP_RADIUS   0.15f
// How long to settle at each target, in milliseconds.
#define SETTLE_MS      2000
// How often to report while settling.
#define REPORT_MS      250

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

// Encoder wrap handling - the AS5048A is single turn and SimpleFOC only
// tracks the wrap when more than 0.8 of a revolution passes between two
// updates. Keep motor.sensor_offset in step so the position frame stays
// continuous across the encoder zero.
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

static uint32_t loopCount = 0;

// One FOC iteration plus the timing bookkeeping
static inline void focStep()
{
    motor.loopFOC();
    encoderCheckWrap();
    motor.move();
    loopCount++;
}

static void reportFocRate()
{
    static uint32_t lastCount = 0;
    static uint32_t lastMs = 0;
    uint32_t now = millis();
    if (now - lastMs < 1000)
        return;
    if (lastMs)
    {
        Serial.print("  FOC loop rate: ");
        Serial.print((loopCount - lastCount) * 1000.0f / (now - lastMs));
        Serial.println(" Hz");
    }
    lastCount = loopCount;
    lastMs = now;
}

// Hold the target for SETTLE_MS, reporting as it goes
static float holdTarget(float target, const char *label)
{
    Serial.print("  target ");
    Serial.print(label);
    Serial.print(" = ");
    Serial.print(target, 4);
    Serial.println(" rad");

    motor.target = target;          // <- the actual command

    uint32_t start = millis();
    uint32_t lastReport = 0;
    while (millis() - start < (uint32_t)SETTLE_MS)
    {
        focStep();
        if (millis() - lastReport > (uint32_t)REPORT_MS)
        {
            lastReport = millis();
            Serial.print("    t=");
            Serial.print(millis() - start);
            Serial.print("ms  shaft_angle=");
            Serial.print(motor.shaft_angle, 4);
            Serial.print("  raw_ang=");
            Serial.print(sensor1.getSensorAngle(), 4);
            Serial.print("  turns=");
            Serial.print(sensor1.getFullRotations());
            Serial.print("  err=");
            Serial.print(target - motor.shaft_angle, 4);
            Serial.println();
        }
    }
    reportFocRate();

    float err = target - motor.shaft_angle;
    Serial.print("  RESULT ");
    Serial.print(label);
    Serial.print(": reached=");
    Serial.print(motor.shaft_angle, 4);
    Serial.print("  err=");
    Serial.print(err, 4);
    Serial.print(" rad (");
    Serial.print(err * 180.0f / PI, 2);
    Serial.println(" deg)");
    return err;
}

void setup()
{
    Serial.begin(115200);
    _delay(1500);                       // let the USB CDC settle
    Serial.println();
    Serial.println("=== BLDCHandset hardware test ===");
    Serial.println("send any character on the serial port to start");
    while (!Serial.available())
        _delay(10);
    Serial.read();
    Serial.println("starting...");

    // -------------------------------------------------------------------
    // 1. encoder
    // -------------------------------------------------------------------
    Serial.println();
    Serial.println("[1] AS5048A over SPI");
    spi2.begin();
    sensor1.init(&spi2);
    for (int i = 0; i < 5; i++)
    {
        sensor1.update();
        // getRawCount() is private, so derive it from the angle
        Serial.print("    raw=");
        Serial.print((int)(sensor1.getSensorAngle() / (2.0f * PI) * (1 << 11)));
        Serial.print("  angle=");
        Serial.print(sensor1.getAngle(), 4);
        Serial.print(" rad (");
        Serial.print(sensor1.getAngle() * 180.0f / PI, 2);
        Serial.println(" deg)");
        _delay(200);
    }

    // -------------------------------------------------------------------
    // 2. FOC alignment
    // -------------------------------------------------------------------
    Serial.println();
    Serial.println("[2] FOC alignment (the motor WILL move)");
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
    uint32_t alignStart = millis();
    bool ok = motor.initFOC();
    Serial.print("    initFOC: ");
    Serial.println(ok ? "OK" : "FAILED");
    Serial.print("    took ");
    Serial.print(millis() - alignStart);
    Serial.println(" ms");
    Serial.print("    sensor_direction  = ");
    Serial.println(motor.sensor_direction);
    Serial.print("    zero_elect_angle  = ");
    Serial.println(motor.zero_electric_angle, 4);
    if (!ok)
    {
        Serial.println("    alignment failed - the motor stays disabled.");
        Serial.println("    check the 12V supply, the driver wiring and the pole pair count.");
        while (1)
            _delay(1000);
    }

    // -------------------------------------------------------------------
    // 3. position lock
    // -------------------------------------------------------------------
    Serial.println();
    Serial.println("[3] position lock sweep");

    // The alignment leaves the rotor spinning. Let it settle first, otherwise
    // the boot reading is taken while the motor is still coasting and every
    // target ends up on the far side of a mechanical stop.
    Serial.println("    settling for 2 s ...");
    motor.loopFOC();
    motor.move();
    float settleTarget = motor.shaft_angle;
    uint32_t t0 = millis();
    while (millis() - t0 < 2000)
    {
        motor.target = settleTarget;
        focStep();
    }
    Serial.print("    settled at ");
    Serial.print(motor.shaft_angle, 4);
    Serial.print(" rad, raw encoder ");
    Serial.print(sensor1.getSensorAngle(), 4);
    Serial.print(", turns=");
    Serial.println(sensor1.getFullRotations());

    focStep();
    encoderReset();
    float boot = motor.shaft_angle;
    Serial.print("    boot shaft_angle = ");
    Serial.print(boot, 4);
    Serial.println(" rad");

    float targets[7] = {
        boot,
        boot + SWEEP_RADIUS,
        boot + SWEEP_RADIUS / 2.0f,
        boot,
        boot - SWEEP_RADIUS / 2.0f,
        boot - SWEEP_RADIUS,
        boot
    };
    const char *labels[7] = { "0", "+full", "+half", "centre", "-half", "-full", "0 (return)" };

    float worst = 0.0f;
    for (int i = 0; i < 7; i++)
    {
        float err = holdTarget(targets[i], labels[i]);
        if (fabsf(err) > fabsf(worst))
            worst = err;
    }

    Serial.println();
    Serial.print("=== worst error over the sweep: ");
    Serial.print(worst, 4);
    Serial.print(" rad (");
    Serial.print(worst * 180.0f / PI, 2);
    Serial.println(" deg) ===");
    Serial.println("=== test done ===");
    Serial.flush();

    // keep holding the last target
    while (1)
    {
        focStep();
        reportFocRate();
    }
}

void loop()
{
}
