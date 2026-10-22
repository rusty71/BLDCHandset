/**
 * BLDCHandset diagnostic - AS5048A bit-banged SPI probe
 *
 * The hardware SPI reads came back as 0x0000 for every register, which is
 * what a disconnected or unpowered sensor looks like - but it could also be
 * an SPI timing problem (long wires, 10 MHz). This reads the same registers
 * with a hand rolled, slow (500 kHz) bit banged SPI and dumps everything,
 * so we can tell the two apart.
 *
 * To run it, copy it over src/main.cpp, flash it, then restore:
 *     cp variants/spi_probe.cpp src/main.cpp
 *     pio run -t upload --upload-port /dev/ttyACM0
 *     git checkout src/main.cpp
 */

#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"

#define SCK_PIN  10
#define MOSI_PIN 11
#define MISO_PIN 12
#define CS_PIN   13

static uint16_t sclkToEdge = 4;      // half period in us

static void spiBegin()
{
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    pinMode(SCK_PIN, OUTPUT);
    digitalWrite(SCK_PIN, LOW);
    pinMode(MOSI_PIN, OUTPUT);
    digitalWrite(MOSI_PIN, LOW);
    pinMode(MISO_PIN, INPUT);
}

// MSB first, 16 bits
static uint16_t spiTransfer16(uint16_t data)
{
    uint16_t rx = 0;
    for (int i = 15; i >= 0; i--)
    {
        digitalWrite(MOSI_PIN, (data >> i) & 1);
        delayMicroseconds(sclkToEdge);
        digitalWrite(SCK_PIN, HIGH);
        delayMicroseconds(sclkToEdge);
        rx = (uint16_t)((rx << 1) | (digitalRead(MISO_PIN) & 1));
        digitalWrite(SCK_PIN, LOW);
        delayMicroseconds(sclkToEdge);
    }
    return rx;
}

static uint16_t readRegister(uint16_t reg)
{
    uint16_t command = reg | (1 << 14);          // read bit
    uint8_t cnt = 0;
    uint16_t v = command;
    for (int i = 0; i < 16; i++) { if (v & 1) cnt++; v >>= 1; }
    command |= (uint16_t)((cnt & 1) << 15);        // even parity

    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(2);
    spiTransfer16(command);
    digitalWrite(CS_PIN, HIGH);
    delayMicroseconds(2);                          // t_SU, 350 ns min
    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(2);
    uint16_t val = spiTransfer16(0x0000);
    digitalWrite(CS_PIN, HIGH);
    return val;
}

static void decodeFieldStatus(uint16_t f)
{
    const char *field[4] = { "TOO STRONG", "TOO WEAK", "GOOD", "OVERFLOW" };
    const char *push[4] = { "push-pull", "push-pull", "open-drain", "open-drain" };
    Serial.print("  field status : ");
    Serial.print(field[(f >> 6) & 3]);
    Serial.print(", ");
    Serial.print(push[(f >> 4) & 3]);
    Serial.print(", ");
    Serial.print(((f >> 2) & 1) ? "magnetic field overflowed" : "normal");
    Serial.print(", ");
    Serial.print((f & 1) ? "push-pull" : "push-pull");
}

void setup()
{
    Serial.begin(115200);
    _delay(1500);
    Serial.println();
    Serial.println("=== AS5048A bit-banged SPI probe ===");
    Serial.println("send any character on the serial port to start");
    while (!Serial.available())
        _delay(10);
    Serial.read();

    spiBegin();

    Serial.println();
    Serial.println("--- reading every register at 500 kHz ---");
    Serial.println("reg   raw      notes");
    for (uint16_t reg = 0x00; reg <= 0x0F; reg++)
    {
        uint16_t v = readRegister(reg);
        Serial.print("0x");
        if (reg < 0x10) Serial.print('0');
        Serial.print(reg, HEX);
        Serial.print("  0x");
        Serial.print(v, HEX);
        Serial.print("   ");
        if (reg == 0x0A) decodeFieldStatus(v);
        else if (reg == 0x0E) { Serial.print("angle = "); Serial.print((v & 0x3FFF) * 360.0f / 16384.0f, 2); Serial.print(" deg"); }
        Serial.println();
        _delay(20);
    }
    for (uint16_t reg = 0x3F00; reg <= 0x3FFF; reg += 0x100)
    {
        uint16_t v = readRegister(reg);
        Serial.print("0x");
        Serial.print(reg, HEX);
        Serial.print("  0x");
        Serial.print(v, HEX);
        Serial.println();
        _delay(20);
    }

    Serial.println();
    Serial.println("--- the register SimpleFOC actually uses: 0x3FFF (ENC_ANG) ---");
    for (int i = 0; i < 10; i++)
    {
        uint16_t v = readRegister(0x3FFF);
        Serial.print("  0x3FFF -> 0x");
        Serial.print(v, HEX);
        Serial.print("  angle=");
        Serial.print((v & 0x3FFF) * 360.0f / 16384.0f, 2);
        Serial.println(" deg");
        _delay(250);
    }

    Serial.println();
    Serial.println("--- continuous read mode: CS held low, 0xFFFF frames ---");
    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(2);
    spiTransfer16(0xFFFF);              // command: read ENC_ANG
    for (int i = 0; i < 10; i++)
    {
        uint16_t v = spiTransfer16(0x0000);
        Serial.print("  0x");
        Serial.print(v, HEX);
        Serial.print("  angle=");
        Serial.print((v & 0x3FFF) * 360.0f / 16384.0f, 2);
        Serial.println(" deg");
        _delay(250);
    }
    digitalWrite(CS_PIN, HIGH);

    Serial.println();
    Serial.println("--- repeat the angle register 20 times (turn the shaft by hand) ---");
    for (int i = 0; i < 20; i++)
    {
        uint16_t v = readRegister(0x0E);
        Serial.print("  0x");
        Serial.print(v, HEX);
        Serial.print("  ");
        Serial.print((v & 0x3FFF) * 360.0f / 16384.0f, 2);
        Serial.println(" deg");
        _delay(250);
    }

    Serial.println();
    Serial.println("--- electrical test: is MISO stuck low? ---");
    Serial.print("  MISO pin level (idle) = ");
    Serial.println(digitalRead(MISO_PIN));
    Serial.println("=== probe done ===");
    Serial.flush();

    while (1)
        _delay(1000);
}

void loop()
{
}
