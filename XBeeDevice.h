//
// Created by William Scheirey on 3/12/24.
//

#ifndef XBEE_BACKEND_CPP_XBEEDEVICE_H
#define XBEE_BACKEND_CPP_XBEEDEVICE_H

#include <queue>
#include "../Utility.h"
#include "XBeeUtility.h"
#include "../CircularBuffer.hpp"

#define BUFFER_LENGTH 2048

class XBeeDevice
{
public:
    XBeeDevice();

    void sendFrame(uint8_t *frame, size_t size_bytes);

    void sendTransmitRequestCommand(uint64_t address, const uint8_t *data, size_t size_bytes);

    void sendNodeDiscoveryCommand();

    void queueAtCommandLocal(uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes);

    void
    sendAtCommandLocal(uint8_t frameType, uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes);

    void sendAtCommandLocal(uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes);

    void setParameter(uint16_t parameter, const uint8_t *value, size_t valueSize_bytes);

    void setParameter(uint16_t parameter, uint8_t value);

    void queryParameter(uint16_t parameter);

    void applyChanges();

    void write();

    void receive();

    void doCycle();

private:
    static uint16_t getAtCommand(const uint8_t *frame);

    virtual void serialWrite(const char *data, size_t length_bytes) = 0;

    virtual void serialRead(char *buffer, size_t length_bytes) = 0;

    virtual void packetRead() = 0;

    virtual void handleReceivePacket(XBee::ReceivePacket::Struct *frame) = 0;

    void parseReceivePacket(const uint8_t *frame, uint8_t length);

    bool handleFrame(const uint8_t *frame);

    void handleAtCommandResponse(const uint8_t *frame, uint8_t length_bytes);

    void handleNodeDiscoveryResponse(const uint8_t *frame, uint8_t length_bytes);

    uint8_t *transmitRequestFrame;
    uint8_t *atCommandFrame;

    uint8_t currentFrameID;

    std::queue<XBee::BasicFrame *> transmitFrameQueue;
    std::queue<uint16_t> atParamConfirmationsBeingWaitedOn;

    XBee::ReceivePacket::Struct *receivePacketStruct = new XBee::ReceivePacket::Struct;

    char *receiveFrame;
    char *nodeID;
    SerialCircularBuffer *buffer;
};


#endif //XBEE_BACKEND_CPP_XBEEDEVICE_H
