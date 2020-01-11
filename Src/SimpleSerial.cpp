#include "SimpleSerial.h"

SimpleSerial::SimpleSerial(AvailableFunType availableFun, ReadFunType readFun, WriteFunType writeFun) {
    _availableFun = availableFun;
    _readFun = readFun;
    _writeFun = writeFun;
}

/*
 * Returns true if data is available.
 */
bool SimpleSerial::available(void) {
    if (_readQueueLen != 0)
        return true;
    else
        return false;
}

/*
 * Takes payload (in bytes) and its id, frames it into a packet, and places it
 * in send queue.
 */
void SimpleSerial::send(uint8_t id, uint8_t len, uint8_t payload[]) {
    // Do nothing if queue full
    if (_sendQueueLen >= QUEUE_LEN)
        return;
    // Frame packet
    uint8_t packet[MAX_PCKT_LEN];
    uint8_t packetLen;
    this->frame(id, len, payload, packetLen, packet);
    // Place packet in send queue
    uint8_t queueIndex = _sendQueueLen;
    for (uint8_t i = 0; i < packetLen; i++) {
        // Copy data to array
        _sendQueue[queueIndex][i] = packet[i];
    }
    _sendQueueLengths[queueIndex] = packetLen;
    _sendQueueLen++;
}

/*
 * Converts float to 4 bytes and sends using send().
 */
void SimpleSerial::sendFloat(uint8_t id, float f) {
    union u {
        float _f = 0.;
        uint8_t b[4];
    } u;
    u._f = f;
    send(id, 4, u.b);
}

/*
 * Converts int to 4 bytes and sends using send();
 */
void SimpleSerial::sendInt(uint8_t id, int32_t i) {
    union u {
        int32_t _i = 0;
        uint8_t b[4];
    } u;
    u._i = i;
    send(id, 4, u.b);
}

/*
 * Frames array of bytes into a packet. Inserts flag bytes, id and length.
 */
void SimpleSerial::frame(uint8_t id, uint8_t payloadLen, uint8_t payload[],
                         uint8_t &packetLen, uint8_t packet[]) {

    // Calculate CRC
    uint8_t crc = calcCRC(payload, payloadLen);

    // Extend array to make place for CRC
    uint8_t payloadNew[payloadLen + 1];
    for (uint8_t k = 0; k < payloadLen; ++k) {
        payloadNew[k] = payload[k];
    }
    payload = payloadNew;
    payloadLen++;

    // Insert CRC byte to the end of payload
    payload[payloadLen - 1] = crc;

    // Insert ESC flags
    uint8_t i = 0; // payload index
    uint8_t j = 0; // packet index
    packet[0] = START;
    packet[1] = 0; // packet length
    packet[2] = id;
    j = 3;
    while (i < payloadLen) {
        uint8_t b = payload[i];
        if (b == ESC || b == START || b == END) {
            // Must insert ESC flag
            packet[j] = ESC;
            j++;
        }
        packet[j] = b;
        j++;
        i++;
    }
    packet[j] = END;
    j++;
    packet[1] = j; //packet length
    packetLen = j;
}

/*
 * Continuously running loop. Sends the next packet in send queue.
 */
void SimpleSerial::sendLoop(uint32_t time) {
    if (_sendQueueLen < 1)
        // Return if nothing to send
        return;
    // Loop through waiting packets
    else {
        // Send packet
        uint8_t i = _sendQueueLen - 1;
        _writeFun(_sendQueue[i], _sendQueueLengths[i]);
        _sendQueueLen--;
        }
}

/*
 * Returns and removes the oldest packet in the read queue.
 */
void SimpleSerial::read(uint8_t &id, uint8_t &len, uint8_t payload[]) {
    id = _readQueueIDs[0];
    len = _readQueueLengths[0];
    // Copy from read queue
    for (uint8_t i = 0; i < len; i++) {
        payload[i] = _readQueue[0][i];
    }
    _readQueueLen--;
    // Shift read list forward.
    for (uint8_t i = 0; i < _readQueueLen; i++) {
        for (uint8_t j = 0; j < _readQueueLengths[i]; j++) {
            _readQueue[i][j] = _readQueue[i + 1][j];
        }
    }
}

float SimpleSerial::bytes2Float(uint8_t bytes[]) {
    union u {
        float _f;
        uint8_t b[4];
    } u;
    for (uint8_t i = 0; i < 4; i++) {
        u.b[i] = bytes[i];
    }
    return u._f;
}

void SimpleSerial::float2Bytes(float f, uint8_t bytes[]) {
    union u {
        float _f;
        uint8_t b[4];
    } u;
    u._f = f;
    for (uint8_t i = 0; i < 4; i++) {
        bytes[i] = u.b[i];
    }
}

int16_t SimpleSerial::bytes2Int(uint8_t bytes[]) {
    union u {
        int16_t _i;
        uint8_t b[4];
    } u;
    for (uint8_t i = 0; i < 4; i++) {
        u.b[i] = bytes[i];
    }

    return u._i;
}

void SimpleSerial::int2Bytes(int32_t i, uint8_t bytes[]) {
    union u {
        int32_t _i;
        uint8_t b[4];
    } u;
    u._i = i;
    for (uint8_t i = 0; i < 4; i++) {
        bytes[i] = u.b[i];
    }
}

/*
 * Continuously running loop. Waits for bytes as they arrive and
 * decodes the packet. Reads READ_BYTES_NR in one iteration.
 */
void SimpleSerial::readLoop(uint32_t time) {
    for (int k = 0; k < READ_BYTES_NR; ++k) {

        if (!_availableFun())
            return;
        static uint8_t i = 0; // Byte counter
        static uint8_t l = 0; // Packet length
        static uint8_t id = 0; //Packet id
        static bool esc = false;
        static uint8_t payload[MAX_LEN_PYLD];
        static uint8_t payload_i = 0;
        static uint32_t startTime = time;

        uint8_t b = _readFun();
        if (i == 0 && b == START) {
            // First byte - START flag. Start count.
            startTime = time;
            i = 1;
            payload_i = 0;
        } else if (i == 1) {
            // Second byte - packet length
            l = b;
            i = 2;
        } else if (i == 2) {
            // Third byte - packet identifier
            id = b;
            i = 3;
        } else if (i > 2) {
            // Data value byte
            if (i > (2 * MAX_LEN_PYLD + 4) || (time - startTime) > PACKET_TIMEOUT) {
                // No END flag. Reset.
                i = 0;
                return;
            }
            if (!esc) {
                // No preceding ESC. Accept flags.
                if (b == ESC)
                    // ESC flag. Activate ESC mode.
                    esc = true;
                else if (b == END) {
                    // End of packet. Check if specified and actual length are equal.
                    uint8_t crcReceived = payload[payload_i - 1];
                    uint8_t crcCalculated = calcCRC(payload, payload_i - 1);
                    payload_i--;

                    if ((i == l - 1) && (crcReceived == crcCalculated)) {
                        // Valid data. Add to read queue.
                        if (_readQueueLen >= QUEUE_LEN) {
                            // Queue full. Delete oldest value
                            // Shift read list forward.
                            for (uint8_t i = 0; i < _readQueueLen; i++) {
                                for (uint8_t j = 0; j < _readQueueLengths[i]; j++) {
                                    _readQueue[i][j] = _readQueue[i + 1][j];
                                }
                            }
                            _readQueueLen--;
                        }
                        uint8_t queueIndex = _readQueueLen;
                        for (uint8_t i = 0; i < payload_i; i++) {
                            _readQueue[queueIndex][i] = payload[i];
                        }
                        _readQueueLengths[queueIndex] = payload_i;
                        _readQueueIDs[queueIndex] = id;
                        _readQueueLen++;
                        i = 0;
                    } else {
                        // CORRUPTED data. Reset
                        i = 0;
                        esc = false;
                    }
                    return;
                } else {
                    // Normal data byte. Add to array.
                    payload[payload_i] = b;
                    payload_i++;
                }
            } else {
                // ESC preceding. Ignore flag following ESC byte.
                payload[payload_i] = b;
                payload_i++;
                esc = false;
            }
            i++;
        }
    }
}

void SimpleSerial::loop(uint32_t time) {
    sendLoop(time);
    readLoop(time);
}

void SimpleSerial::confirmReceived(uint8_t id) {
    uint8_t pld[] = {255};
    send(id, 1, pld);
}

// Automatically generated CRC function from python crcmod
// CRC-8; polynomial: 0x107
uint8_t SimpleSerial::calcCRC(uint8_t *data, int len)
{
    static const uint8_t table[256] = {
            0x00U,0x07U,0x0EU,0x09U,0x1CU,0x1BU,0x12U,0x15U,
            0x38U,0x3FU,0x36U,0x31U,0x24U,0x23U,0x2AU,0x2DU,
            0x70U,0x77U,0x7EU,0x79U,0x6CU,0x6BU,0x62U,0x65U,
            0x48U,0x4FU,0x46U,0x41U,0x54U,0x53U,0x5AU,0x5DU,
            0xE0U,0xE7U,0xEEU,0xE9U,0xFCU,0xFBU,0xF2U,0xF5U,
            0xD8U,0xDFU,0xD6U,0xD1U,0xC4U,0xC3U,0xCAU,0xCDU,
            0x90U,0x97U,0x9EU,0x99U,0x8CU,0x8BU,0x82U,0x85U,
            0xA8U,0xAFU,0xA6U,0xA1U,0xB4U,0xB3U,0xBAU,0xBDU,
            0xC7U,0xC0U,0xC9U,0xCEU,0xDBU,0xDCU,0xD5U,0xD2U,
            0xFFU,0xF8U,0xF1U,0xF6U,0xE3U,0xE4U,0xEDU,0xEAU,
            0xB7U,0xB0U,0xB9U,0xBEU,0xABU,0xACU,0xA5U,0xA2U,
            0x8FU,0x88U,0x81U,0x86U,0x93U,0x94U,0x9DU,0x9AU,
            0x27U,0x20U,0x29U,0x2EU,0x3BU,0x3CU,0x35U,0x32U,
            0x1FU,0x18U,0x11U,0x16U,0x03U,0x04U,0x0DU,0x0AU,
            0x57U,0x50U,0x59U,0x5EU,0x4BU,0x4CU,0x45U,0x42U,
            0x6FU,0x68U,0x61U,0x66U,0x73U,0x74U,0x7DU,0x7AU,
            0x89U,0x8EU,0x87U,0x80U,0x95U,0x92U,0x9BU,0x9CU,
            0xB1U,0xB6U,0xBFU,0xB8U,0xADU,0xAAU,0xA3U,0xA4U,
            0xF9U,0xFEU,0xF7U,0xF0U,0xE5U,0xE2U,0xEBU,0xECU,
            0xC1U,0xC6U,0xCFU,0xC8U,0xDDU,0xDAU,0xD3U,0xD4U,
            0x69U,0x6EU,0x67U,0x60U,0x75U,0x72U,0x7BU,0x7CU,
            0x51U,0x56U,0x5FU,0x58U,0x4DU,0x4AU,0x43U,0x44U,
            0x19U,0x1EU,0x17U,0x10U,0x05U,0x02U,0x0BU,0x0CU,
            0x21U,0x26U,0x2FU,0x28U,0x3DU,0x3AU,0x33U,0x34U,
            0x4EU,0x49U,0x40U,0x47U,0x52U,0x55U,0x5CU,0x5BU,
            0x76U,0x71U,0x78U,0x7FU,0x6AU,0x6DU,0x64U,0x63U,
            0x3EU,0x39U,0x30U,0x37U,0x22U,0x25U,0x2CU,0x2BU,
            0x06U,0x01U,0x08U,0x0FU,0x1AU,0x1DU,0x14U,0x13U,
            0xAEU,0xA9U,0xA0U,0xA7U,0xB2U,0xB5U,0xBCU,0xBBU,
            0x96U,0x91U,0x98U,0x9FU,0x8AU,0x8DU,0x84U,0x83U,
            0xDEU,0xD9U,0xD0U,0xD7U,0xC2U,0xC5U,0xCCU,0xCBU,
            0xE6U,0xE1U,0xE8U,0xEFU,0xFAU,0xFDU,0xF4U,0xF3U,
    };

    uint8_t crc = 0x00;

    while (len > 0)
    {
        crc = table[*data ^ (uint8_t)crc];
        data++;
        len--;
    }
    return crc;
}