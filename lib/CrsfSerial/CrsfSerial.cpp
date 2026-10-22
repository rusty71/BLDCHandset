#include "CrsfSerial.h"

#include <string.h>

#undef CRSF_PERFORMANCE_CSV
// static void hexdump(void *p, size_t len)
// {
//     char *data = (char *)p;
//     while (len > 0)
//     {
//         uint8_t linepos = 0;
//         char* linestart = data;
//         // Binary part
//         while (len > 0 && linepos < 16)
//         {
//             if (*data < 0x0f)
//             Serial.write('0');
//             Serial.print(*data, HEX);
//             Serial.write(' ');
//             ++data;
//             ++linepos;
//             --len;
//         }

//         // Spacer to align last line
//         for (uint8_t i = linepos; i < 16; ++i)
//             Serial.print("   ");

//         // ASCII part
//         for (uint8_t i = 0; i < linepos; ++i)
//             Serial.write((linestart[i] < ' ') ? '.' : linestart[i]);
//         Serial.println();
//     }
// }

CrsfSerial::CrsfSerial(HardwareSerial &port, uint32_t baud) :
    _port(port), _crc(0xd5), _baud(baud),
    _lastReceive(0), _lastChannelsPacket(0), _linkIsUp(false),
    _passthroughBaud(0), _paramBufLen(0), _paramId(0), _paramChunksLeft(0)
{
    memset(&_deviceInfo, 0, sizeof(_deviceInfo));
}

void CrsfSerial::begin(uint32_t baud)
{
    if (baud != 0)
        _port.begin(baud);
    else
        _port.begin(_baud);
}

// Call from main loop to update
void CrsfSerial::loop()
{
    handleSerialIn();
}

void CrsfSerial::handleSerialIn()
{
    while (_port.available())
    {
        uint8_t b = _port.read();
        _lastReceive = millis();

        if(_rxBufPos == 0)
            _pktStart = micros();
        if (getPassthroughMode())
        {
            if (onOobData)
                onOobData(b);
            continue;
        }

        _rxBuf[_rxBufPos++] = b;
        handleByteReceived();

        if (_rxBufPos == (sizeof(_rxBuf)/sizeof(_rxBuf[0])))
        {
            // Packet buffer filled and no valid packet found, dump the whole thing
            _rxBufPos = 0;
        }
    }

    checkPacketTimeout();
    checkLinkDown();
}

void CrsfSerial::handleByteReceived()
{
    bool reprocess;
    do
    {
        reprocess = false;
        if (_rxBufPos > 1)
        {
            uint8_t len = _rxBuf[1];
            // Sanity check the declared length isn't outside Type + X{1,CRSF_MAX_PAYLOAD_LEN} + CRC
            // assumes there never will be a CRSF message that just has a type and no data (X)
            if (len < 3 || len > (CRSF_MAX_PAYLOAD_LEN + 2))
            {
                shiftRxBuffer(1);
                reprocess = true;
            }

            else if (_rxBufPos >= (len + 2))
            {
                uint8_t inCrc = _rxBuf[2 + len - 1];
                uint8_t crc = _crc.calc(&_rxBuf[2], len - 1);
                if (crc == inCrc)
                {
                    processPacketIn(len);
                    shiftRxBuffer(len + 2);
                    reprocess = true;
                }
                else
                {
                    shiftRxBuffer(1);
                    reprocess = true;
                }
            }  // if complete packet
        } // if pos > 1
    } while (reprocess);
}

void CrsfSerial::checkPacketTimeout()
{
    // If we haven't received data in a long time, flush the buffer a byte at a time (to trigger shiftyByte)
    if (_rxBufPos > 0 && millis() - _lastReceive > CRSF_PACKET_TIMEOUT_MS)
        while (_rxBufPos)
            shiftRxBuffer(1);
}

void CrsfSerial::checkLinkDown()
{
    if (_linkIsUp && millis() - _lastChannelsPacket > CRSF_FAILSAFE_STAGE1_MS)
    {
        if (onLinkDown)
            onLinkDown();
        _linkIsUp = false;
    }
}


#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

void CrsfSerial::processPacketIn(uint8_t len)
{
    const crsf_header_t *hdr = (crsf_header_t *)_rxBuf;
    const crsf_opentx_sync_t *opentx_sync = (crsf_opentx_sync_t *)(_rxBuf+3);

#ifdef CRSF_PERFORMANCE_CSV
    const crsf_sensor_battery_t *battery = (crsf_sensor_battery_t *)(_rxBuf+3);
    Serial.print(_pktStart);
    Serial.print(',');
    Serial.print('<');
    Serial.print(',');
    Serial.print(hdr->device_addr, HEX);
    Serial.print(',');
    Serial.print(hdr->type, HEX);
    Serial.print(',');
    Serial.print(hdr->frame_size);
    if(hdr->type==0x8) {
        Serial.print(',');
        Serial.print(battery->voltage);
    }
    if(hdr->type==0x3a) {
        Serial.print(',');
        Serial.print((int32_t)htonl(opentx_sync->packet_inteval));
        Serial.print(',');
        Serial.print((int32_t)htonl(opentx_sync->phase_shift));
    }

    Serial.println();
#endif

    if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        switch (hdr->type)
        {
        case CRSF_FRAMETYPE_GPS:
            packetGps(hdr);
            break;
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            packetChannelsPacked(hdr);
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            packetLinkStatistics(hdr);
            break;
        case CRSF_FRAMETYPE_OPENTX_SYNC:
            packetOpenTXSync(opentx_sync);
            break;
        }
    } // CRSF_ADDRESS_FLIGHT_CONTROLLER

    // The radio forwards the ELRS receiver's telemetry (the servo reports its
    // position in the battery sensor frame) on the radio's own address.
    if (hdr->device_addr == CRSF_ADDRESS_RADIO_TRANSMITTER)
    {
        switch (hdr->type)
        {
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            packetBattery(hdr);
            break;
        }
    }

    // The HANDSET (timing correction) frame is matched on type only: the
    // radio may put its device address or the 0xC8 serial sync byte in the
    // first slot depending on which connector it is delivered through.
    if (hdr->type == CRSF_FRAMETYPE_HANDSET)
    {
        packetOpenTXSync(opentx_sync);
    }

// Parameter protocol frames are extended header frames. The device may
    // answer with either its device address or the 0xC8 serial sync byte in
    // the first slot, so match on the frame type only.
    if (hdr->type == CRSF_FRAMETYPE_DEVICE_INFO ||
        hdr->type == CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY ||
        hdr->type == CRSF_FRAMETYPE_PARAMETER_WRITE)
    {
        const crsf_ext_header_t *ext = (const crsf_ext_header_t *)_rxBuf;
        switch (hdr->type)
        {
        case CRSF_FRAMETYPE_DEVICE_INFO:
            packetDeviceInfo(ext);
            break;
        case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
            packetParamEntry(ext);
            break;
        case CRSF_FRAMETYPE_PARAMETER_WRITE:
            packetParamWriteAck(ext);
            break;
        }
    }
}

// Shift the bytes in the RxBuf down by cnt bytes
void CrsfSerial::shiftRxBuffer(uint8_t cnt)
{
    // If removing the whole thing, just set pos to 0
    if (cnt >= _rxBufPos)
    {
        _rxBufPos = 0;
        return;
    }

    if (cnt == 1 && onOobData)
        onOobData(_rxBuf[0]);

    // Otherwise do the slow shift down
    uint8_t *src = &_rxBuf[cnt];
    uint8_t *dst = &_rxBuf[0];
    _rxBufPos -= cnt;
    uint8_t left = _rxBufPos;
    while (left--)
        *dst++ = *src++;
}

void CrsfSerial::packetChannelsPacked(const crsf_header_t *p)
{
    crsf_channels_t *ch = (crsf_channels_t *)&p->data;
    _channels[0] = ch->ch0;
    _channels[1] = ch->ch1;
    _channels[2] = ch->ch2;
    _channels[3] = ch->ch3;
    _channels[4] = ch->ch4;
    _channels[5] = ch->ch5;
    _channels[6] = ch->ch6;
    _channels[7] = ch->ch7;
    _channels[8] = ch->ch8;
    _channels[9] = ch->ch9;
    _channels[10] = ch->ch10;
    _channels[11] = ch->ch11;
    _channels[12] = ch->ch12;
    _channels[13] = ch->ch13;
    _channels[14] = ch->ch14;
    _channels[15] = ch->ch15;

    for (unsigned int i=0; i<CRSF_NUM_CHANNELS; ++i)
        _channels[i] = map(_channels[i], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);

    if (!_linkIsUp && onLinkUp)
        onLinkUp();
    _linkIsUp = true;
    _lastChannelsPacket = millis();

    if (onPacketChannels)
        onPacketChannels();
}

void CrsfSerial::packetOpenTXSync(const crsf_opentx_sync_t *p)
{
    const crsf_opentx_sync_t *txsync = (crsf_opentx_sync_t *)p;
    memcpy(&_opentxSync, txsync, sizeof(_opentxSync));

    if (onPacketSync)
        onPacketSync(&_opentxSync);
}

// ---------------------------------------------------------------------------
// Parameter / menu protocol handlers
// ---------------------------------------------------------------------------

// Decode a fully reassembled 0x2B parameter entry and hand it to the user.
static void decodeParamEntry(uint8_t id, const uint8_t *buf, uint8_t len, crsf_param_t *out)
{
    memset(out, 0, sizeof(*out));
    out->id = id;

    // A well formed entry needs at least parent + type + a one char name
    if (len < 4)
        return;

    out->parent = buf[0];
    out->type = buf[1] & CRSF_PARAM_TYPE_MASK;
    out->hidden = (buf[1] & CRSF_PARAM_TYPE_HIDDEN) != 0;

    const uint8_t *p = &buf[2];
    uint8_t avail = len - 2;

    // Name, null terminated
    uint8_t nameLen = 0;
    while (nameLen < avail && p[nameLen] != 0)
        nameLen++;
    uint8_t nameCopy = nameLen < (CRSF_PARAM_NAME_MAX - 1) ? nameLen : (CRSF_PARAM_NAME_MAX - 1);
    memcpy(out->name, p, nameCopy);
    out->name[nameCopy] = 0;
    p += nameLen + 1;
    avail = (p > &buf[len]) ? 0 : (uint8_t)(&buf[len] - p);

    if (out->type == CRSF_PARAM_TEXT_SELECTION)
    {
        // Options, null terminated semicolon separated list
        uint8_t optLen = 0;
        while (optLen < avail && p[optLen] != 0)
            optLen++;
        uint8_t optCopy = optLen < (CRSF_PARAM_OPTIONS_MAX - 1) ? optLen : (CRSF_PARAM_OPTIONS_MAX - 1);
        memcpy(out->options, p, optCopy);
        out->options[optCopy] = 0;
        p += optLen + 1;
        avail = (p > &buf[len]) ? 0 : (uint8_t)(&buf[len] - p);

        // value, min, max, default
        if (avail >= 4)
        {
            out->value = p[0];
            out->min = p[1];
            out->max = p[2];
            out->def = p[3];
        }
    }
}

void CrsfSerial::packetDeviceInfo(const crsf_ext_header_t *p)
{
    // frame_size counts type + dest + orig + payload + crc
    if (p->frame_size < 4)
        return;
    uint8_t payloadLen = p->frame_size - 4;
    if (payloadLen < sizeof(crsf_device_info_t) + 1)
        return;

    // The payload starts with the null terminated device name
    const char *name = (const char *)p->data;
    uint8_t nameLen = 0;
    while (nameLen < payloadLen && name[nameLen] != 0)
        nameLen++;
    uint8_t infoLen = sizeof(crsf_device_info_t);
    if (nameLen + 1 + infoLen > payloadLen)
        return;

    memcpy(&_deviceInfo, &p->data[nameLen + 1], infoLen);
    _deviceInfo.serial_number = be32toh(_deviceInfo.serial_number);
    _deviceInfo.hardware_id = be32toh(_deviceInfo.hardware_id);
    _deviceInfo.firmware_id = be32toh(_deviceInfo.firmware_id);

    if (onPacketDeviceInfo)
        onPacketDeviceInfo(&_deviceInfo);
}

void CrsfSerial::packetParamEntry(const crsf_ext_header_t *p)
{
    if (p->frame_size < 6)
        return;
    uint8_t payloadLen = p->frame_size - 4;
    if (payloadLen < 2)
        return;

    uint8_t id = p->data[0];
    uint8_t remaining = p->data[1];
    uint8_t chunkLen = payloadLen - 2;
    const uint8_t *chunk = &p->data[2];

    // Reassemble chunks of a single parameter entry
    if (id != _paramId)
    {
        _paramId = id;
        _paramBufLen = 0;
    }
    uint8_t space = (uint8_t)(sizeof(_paramBuf) - _paramBufLen);
    if (chunkLen > space)
        chunkLen = space;
    memcpy(&_paramBuf[_paramBufLen], chunk, chunkLen);
    _paramBufLen = (uint8_t)(_paramBufLen + chunkLen);

    if (remaining == 0)
    {
        crsf_param_t param;
        decodeParamEntry(id, _paramBuf, _paramBufLen, &param);
        _paramBufLen = 0;
        if (onPacketParamEntry)
            onPacketParamEntry(&param);
    }
}

void CrsfSerial::packetParamWriteAck(const crsf_ext_header_t *p)
{
    if (p->frame_size < 5)
        return;
    uint8_t payloadLen = p->frame_size - 4;
    if (payloadLen < 1)
        return;

    if (onPacketParamWriteAck)
        onPacketParamWriteAck(p->data[0], &p->data[1], payloadLen - 1);
}

void CrsfSerial::packetBattery(const crsf_header_t *p)
{
    // frame_size counts type + payload + crc, so the payload is frame_size-2
    if (p->frame_size < 2 + (uint8_t)sizeof(crsf_sensor_battery_t))
        return;
    memcpy(&vbatSensor, (const uint8_t *)p + 3, sizeof(vbatSensor));
    if (onPacketVBat)
        onPacketVBat(&vbatSensor);
}

void CrsfSerial::packetLinkStatistics(const crsf_header_t *p)
{
    const crsfLinkStatistics_t *link = (crsfLinkStatistics_t *)p->data;
    memcpy(&_linkStatistics, link, sizeof(_linkStatistics));

    if (onPacketLinkStatistics)
        onPacketLinkStatistics(&_linkStatistics);
}

void CrsfSerial::packetGps(const crsf_header_t *p)
{
    const crsf_sensor_gps_t *gps = (crsf_sensor_gps_t *)p->data;
    _gpsSensor.latitude = be32toh(gps->latitude);
    _gpsSensor.longitude = be32toh(gps->longitude);
    _gpsSensor.groundspeed = be16toh(gps->groundspeed);
    _gpsSensor.heading = be16toh(gps->heading);
    _gpsSensor.altitude = be16toh(gps->altitude);
    _gpsSensor.satellites = gps->satellites;

    if (onPacketGps)
        onPacketGps(&_gpsSensor);
}

void CrsfSerial::write(uint8_t b)
{
    _port.write(b);
}

void CrsfSerial::write(const uint8_t *buf, size_t len)
{
#ifdef CRSF_PERFORMANCE_CSV
    const crsf_header_t *hdr = (crsf_header_t *)buf;
    const crsf_channels_t *channels = (crsf_channels_t *)(buf+3);
    const crsf_sensor_battery_t *battery = (crsf_sensor_battery_t *)(buf+3);

    Serial.print(micros());
    Serial.print(',');
    Serial.print('>');
    Serial.print(',');
    Serial.print(hdr->device_addr, HEX);
    Serial.print(',');
    Serial.print(hdr->type, HEX);
    Serial.print(',');
    Serial.print(hdr->frame_size);
    Serial.print(',');
    if(hdr->type==0x16) {
        Serial.print(channels->ch1);
        // Serial.print(',');
        // Serial.print(buf[4], HEX);
        // Serial.print(',');
        // Serial.print(buf[5], HEX);
        // Serial.print(',');
        // Serial.print(buf[6], HEX);
        // Serial.print(',');
        // Serial.print(buf[7], HEX);
        // Serial.print(',');
        // Serial.print(buf[8], HEX);
        // Serial.print(',');
        // Serial.print(buf[9], HEX);
        // Serial.print(',');
        // Serial.print(buf[10], HEX);

    }
    if(hdr->type==0x8) {
        Serial.print(battery->voltage);
        // Serial.print(',');
        // Serial.print(buf[4], HEX);
        // Serial.print(',');
        // Serial.print(buf[5], HEX);
        // Serial.print(',');
        // Serial.print(buf[6], HEX);
        // Serial.print(',');
        // Serial.print(buf[7], HEX);
        // Serial.print(',');
        // Serial.print(buf[8], HEX);
        // Serial.print(',');
        // Serial.print(buf[9], HEX);
        // Serial.print(',');
        // Serial.print(buf[10], HEX);

    }
    Serial.println(); 
#endif //CRSF_PERFORMANCE_CSV

    _port.write(buf, len);
}

void CrsfSerial::queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len)
{
    if (getPassthroughMode())
        return;
    if (len > CRSF_MAX_PAYLOAD_LEN)
        return;

    uint8_t buf[CRSF_MAX_PACKET_SIZE];
    buf[0] = addr;
    buf[1] = len + 2; // type + payload + crc
    buf[2] = type;
    memcpy(&buf[3], payload, len);
    buf[len+3] = _crc.calc(&buf[2], len + 1);

    write(buf, len + 4);
}

// Extended header frame: [addr][len][type][dest][orig][payload][crc]
void CrsfSerial::queuePacketExt(uint8_t addr, uint8_t dest, uint8_t orig, uint8_t type,
                                const void *payload, uint8_t len)
{
    if (getPassthroughMode())
        return;
    if (len > CRSF_MAX_PAYLOAD_LEN)
        return;

    uint8_t buf[CRSF_MAX_PACKET_SIZE];
    buf[0] = addr;
    buf[1] = len + 4; // type + dest + orig + payload + crc
    buf[2] = type;
    buf[3] = dest;
    buf[4] = orig;
    memcpy(&buf[5], payload, len);
    buf[len+5] = _crc.calc(&buf[2], len + 3);

    write(buf, len + 6);
}

// ---------------------------------------------------------------------------
// Parameter / menu protocol
// ---------------------------------------------------------------------------

void CrsfSerial::sendDevicePing(uint8_t dest_addr)
{
    // 0x28 has no payload. The origin address identifies us as the remote
    // control (0xEA) which is what ELRS expects on its handset port.
    queuePacketExt(CRSF_SYNC_BYTE, dest_addr, CRSF_ADDRESS_RADIO_TRANSMITTER,
                   CRSF_FRAMETYPE_DEVICE_PING, NULL, 0);
}

void CrsfSerial::sendParamRead(uint8_t id, uint8_t chunk)
{
    uint8_t payload[2] = { id, chunk };
    queuePacketExt(CRSF_SYNC_BYTE, CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER,
                   CRSF_FRAMETYPE_PARAMETER_READ, payload, sizeof(payload));
}

void CrsfSerial::sendParamWrite(uint8_t id, const void *value, uint8_t len)
{
    if (len + 1 > CRSF_MAX_PAYLOAD_LEN)
        return;
    uint8_t payload[CRSF_MAX_PAYLOAD_LEN];
    payload[0] = id;
    if (len)
        memcpy(&payload[1], value, len);
    queuePacketExt(CRSF_SYNC_BYTE, CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER,
                   CRSF_FRAMETYPE_PARAMETER_WRITE, payload, len + 1);
}

void CrsfSerial::sendParamWriteU8(uint8_t id, uint8_t value)
{
    sendParamWrite(id, &value, 1);
}

/**
 * @brief   Enter passthrough mode (serial sent directly to shiftybyte),
 *          optionally changing the baud rate used during passthrough mode
 * @param val
 *          True to start passthrough mode, false to resume processing CRSF
 * @param passthroughBaud
 *          New baud rate for passthrough mode, or 0 to not change baud
 *          Not used if disabling passthough
*/
void CrsfSerial::setPassthroughMode(bool val, uint32_t passthroughBaud)
{
    if (val)
    {
        // If not requesting any baud change
        if (passthroughBaud == 0)
        {
            // Enter passthrough mode if not yet
            if (_passthroughBaud == 0)
                _passthroughBaud = _baud;
            return;
        }

        _passthroughBaud = passthroughBaud;
    }
    else
    {
        // Not in passthrough, can't leave it any harder than we already are
        if (_passthroughBaud == 0)
            return;

        // Leaving passthrough, but going back to same baud, just disable
        if (_passthroughBaud == _baud)
        {
            _passthroughBaud = 0;
            return;
        }

        _passthroughBaud = 0;
    }

    // Can only get here if baud is changing, close and reopen the port
    _port.end(); // assumes flush()
    begin(_passthroughBaud);
}
