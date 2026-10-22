#pragma once

#include <Arduino.h>
#include <crc8.h>
#include "crsf_protocol.h"

enum eFailsafeAction { fsaNoPulses, fsaHold };

// A decoded parameter entry as described by the TBS CRSF spec 0x2B
struct crsf_param_t
{
    uint8_t id;                              // parameter number
    uint8_t parent;                          // parent folder number
    uint8_t type;                            // crsf_param_type_e, hidden bit stripped
    bool    hidden;                          // entry is hidden from the host UI
    char    name[CRSF_PARAM_NAME_MAX];
    char    options[CRSF_PARAM_OPTIONS_MAX]; // semicolon separated, TEXT_SELECTION only
    uint8_t value;                           // TEXT_SELECTION: index into options
    uint8_t min;
    uint8_t max;
    uint8_t def;
};

class CrsfSerial
{
public:
    // Packet timeout where buffer is flushed if no data is received in this time
    static const unsigned int CRSF_PACKET_TIMEOUT_MS = 100;
    static const unsigned int CRSF_FAILSAFE_STAGE1_MS = 300;

    CrsfSerial(HardwareSerial &port, uint32_t baud = CRSF_BAUDRATE);
    void begin(uint32_t baud = 0);
    void loop();
    void write(uint8_t b);
    void write(const uint8_t *buf, size_t len);
    void queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len);
    // Extended header frame: [addr][len][type][dest][orig][payload][crc]
    void queuePacketExt(uint8_t addr, uint8_t dest, uint8_t orig, uint8_t type,
                        const void *payload, uint8_t len);

    // --- Parameter / menu protocol -----------------------------------------
    // Ask a device (0x00 = broadcast) to announce its parameter table (0x28)
    void sendDevicePing(uint8_t dest_addr);
    // Read parameter <id>, chunk <chunk> (0x2C)
    void sendParamRead(uint8_t id, uint8_t chunk = 0);
    // Write parameter <id> (0x2D). len depends on the parameter type.
    void sendParamWrite(uint8_t id, const void *value, uint8_t len);
    void sendParamWriteU8(uint8_t id, uint8_t value);

    uint32_t getBaud() const { return _baud; };
    // Return current channel value (1-based) in us
    int getChannel(unsigned int ch) const { return _channels[ch - 1]; }
    // Return the raw 11 bit channel value (1-based), without the us mapping.
    // Needed for anything that has to survive a round trip unchanged, e.g. a
    // latency token.
    int getChannelRaw(unsigned int ch) const { return _channels_raw[ch - 1]; }
    const crsfLinkStatistics_t *getLinkStatistics() const { return &_linkStatistics; }
    const crsf_sensor_gps_t *getGpsSensor() const { return &_gpsSensor; }
    bool isLinkUp() const { return _linkIsUp; }
    // Called for every valid frame, whatever its type. buf points at the
    // frame header (device address) and len is the CRSF length byte.
    void (*onPacketAny)(uint8_t addr, uint8_t type, uint8_t len, const uint8_t *buf) = nullptr;
    // Number of CRSF frames received with a valid CRC - for diagnostics
    uint32_t framesReceived() const { return _framesReceived; }
    uint32_t bytesReceived() const { return _bytesReceived; }
    bool getPassthroughMode() const { return _passthroughBaud != 0; }
    void setPassthroughMode(bool val, uint32_t passthroughBaud = 0);

    // Event Handlers
    void (*onLinkUp)();
    void (*onLinkDown)();
    // OobData is any byte which is not CRSF, including passthrough
    void (*onOobData)(uint8_t b);
    // CRSF Packet Callbacks
    void (*onPacketChannels)();
    void (*onPacketLinkStatistics)(crsfLinkStatistics_t *ls);
    void (*onPacketGps)(crsf_sensor_gps_t *gpsSensor);
    void (*onPacketVBat)(crsf_sensor_battery_t *vbatSensor);
    void (*onPacketSync)(crsf_opentx_sync_t *syncReq);
    // 0x29 Parameter Device Information
    void (*onPacketDeviceInfo)(crsf_device_info_t *info);
    // 0x2B Parameter Settings (Entry), fully reassembled and decoded
    void (*onPacketParamEntry)(crsf_param_t *param);
    // 0x2D Parameter write acknowledgement
    void (*onPacketParamWriteAck)(uint8_t id, const uint8_t *value, uint8_t len);

private:
    HardwareSerial &_port;
    uint8_t _rxBuf[CRSF_MAX_PACKET_SIZE];
    uint8_t _rxBufPos;
    Crc8 _crc;
    crsfLinkStatistics_t _linkStatistics;
    crsf_opentx_sync_t _opentxSync;
    crsf_sensor_gps_t _gpsSensor;
    crsf_sensor_battery_t  vbatSensor;
    crsf_device_info_t _deviceInfo;
    uint32_t _baud;
    uint32_t _lastReceive;
    uint32_t _pktStart;
    uint32_t _lastChannelsPacket;
    bool _linkIsUp;
    uint32_t _passthroughBaud;
    int _channels[CRSF_NUM_CHANNELS];
    int _channels_raw[CRSF_NUM_CHANNELS];
    uint32_t _framesReceived = 0;
    uint32_t _bytesReceived = 0;

    // Parameter entry reassembly
    uint8_t _paramBuf[256];
    uint8_t _paramChunk = 0;   // chunk index of the entry being reassembled
    uint8_t _paramBufLen;
    uint8_t _paramId;
    uint8_t _paramChunksLeft;

    void handleSerialIn();
    void handleByteReceived();
    void shiftRxBuffer(uint8_t cnt);
    void processPacketIn(uint8_t len);
    void checkPacketTimeout();
    void checkLinkDown();

    // Packet Handlers
    void packetChannelsPacked(const crsf_header_t *p);
    void packetLinkStatistics(const crsf_header_t *p);
    void packetBattery(const crsf_header_t *p);
    void packetOpenTXSync(const crsf_opentx_sync_t *p);
    void packetGps(const crsf_header_t *p);
    void packetDeviceInfo(const crsf_ext_header_t *p);
    void packetParamEntry(const crsf_ext_header_t *p);
    void packetParamWriteAck(const crsf_ext_header_t *p);
};
