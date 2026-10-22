/**
 * BLDCHandset diagnostic - SPI mode test
 *
 * The bit banged probe read a live angle from the AS5048A at SPI mode 0,
 * while SimpleFOC's MagneticSensorSPI (configured for SPI_MODE1) read a
 * constant. This reads the same register through SimpleFOC with each mode
 * so we can see which one the sensor actually accepts.
 *
 * To run it, copy it over src/main.cpp, flash it, then restore:
 *     cp variants/spi_mode_test.cpp src/main.cpp
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

static MagneticSensorSPIConfig_s makeConfig(uint8_t mode)
{
    MagneticSensorSPIConfig_s c;
    c.spi_mode = mode;
    c.clock_speed = 10000000;
    c.bit_resolution = 11;
    c.angle_register = 0x3FFF;
    c.data_start_bit = 13;
    c.command_rw_bit = 14;
    c.command_parity_bit = 15;
    return c;
}

static void testMode(uint8_t mode, const char *name)
{
    MagneticSensorSPIConfig_s cfg = makeConfig(mode);
    MagneticSensorSPI sensor(cfg, SENSOR1_CS);
    sensor.init(&spi2);

    Serial.print("--- ");
    Serial.print(name);
    Serial.println(" ---");
    int first = -1, last = -1, distinct = 0;
    static int seen[64];
    int nseen = 0;
    for (int i = 0; i < 20; i++)
    {
        sensor.update();
        // getRawCount() is private, derive the raw value from the angle
        int raw = (int)(sensor.getSensorAngle() / (2.0f * PI) * (1 << 11));
        if (i == 0) first = raw;
        last = raw;
        bool known = false;
        for (int k = 0; k < nseen; k++) if (seen[k] == raw) known = true;
        if (!known && nseen < 64) seen[nseen++] = raw;
        distinct = nseen;
        Serial.print("    raw=");
        Serial.print(raw);
        Serial.print("  angle=");
        Serial.print(sensor.getAngle(), 3);
        Serial.println(" rad");
        _delay(150);
    }
    Serial.print("    first=");
    Serial.print(first);
    Serial.print("  last=");
    Serial.print(last);
    Serial.print("  distinct values=");
    Serial.print(distinct);
    Serial.println(distinct > 1 ? "   <- LIVE, this mode works" : "   <- constant, this mode does not work");
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    _delay(1500);
    Serial.println();
    Serial.println("=== AS5048A SPI mode test ===");
    Serial.println("send any character on the serial port to start");
    while (!Serial.available())
        _delay(10);
    Serial.read();

    spi2.begin();

    testMode(SPI_MODE1, "SPI_MODE1 (what the sketch uses today)");
    testMode(SPI_MODE0, "SPI_MODE0 (what the bit banged probe used)");
    testMode(SPI_MODE2, "SPI_MODE2");
    testMode(SPI_MODE3, "SPI_MODE3");

    Serial.println("=== done ===");
    Serial.flush();
    while (1)
        _delay(1000);
}

void loop()
{
}
