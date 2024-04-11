//
// Created by William Scheirey on 3/12/24.
//

#ifndef HPRC_XBEEDEVICE_H
#define HPRC_XBEEDEVICE_H

#include <queue>
#include "../Utility.h"
#include "XBeeUtility.h"
#include "CircularBuffer.hpp"
#include "circularQueue.hpp"

#define BUFFER_LENGTH 2048

class XBeeDevice
{
public:
    XBeeDevice();

    virtual void start() = 0;

    virtual void log(const char *format, ...) = 0;

    void sendFrame(uint8_t *frame, size_t size_bytes);

    void sendTransmitRequestCommand(uint64_t address, const uint8_t *data, size_t size_bytes);

    void sendNodeDiscoveryCommand();

    void queueAtCommandLocal(uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes);

    void
    sendAtCommandLocal(uint8_t frameType, uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes);

    void sendAtCommandLocal(uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes);

    void
    sendAtCommandRemote(uint64_t address, uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes);

    void sendAtCommandRemote(uint64_t address, uint8_t frameType, uint16_t command, const uint8_t *commandData,
                             size_t commandDataSize_bytes);

    void setParameterRemote(uint64_t address, uint16_t parameter, const uint8_t *value, size_t valueSize_bytes);

    void setParameterRemote(uint64_t address, uint16_t parameter, const uint8_t value);

    void queryParameterRemote(uint64_t address, uint16_t parameter);

    void setParameter(uint16_t parameter, const uint8_t *value, size_t valueSize_bytes);

    void setParameter(uint16_t parameter, uint8_t value);

    void queryParameter(uint16_t parameter);

    void applyChanges();

    void write();

    bool receive();

    void doCycle();

    bool sendTransmitRequestsImmediately = false;

    bool sendFramesImmediately = false;

    bool logWrongChecksums = true;

private:
    static uint16_t getAtCommand(const uint8_t *frame);

    static uint16_t getRemoteAtCommand(const uint8_t *frame);

    virtual void writeBytes(const char *data, size_t length_bytes) = 0;

    virtual void readBytes(uint8_t *buffer, size_t length_bytes) = 0;

    virtual void packetRead() = 0;

    virtual void handleReceivePacket(XBee::ReceivePacket::Struct *frame) = 0;

    virtual void handleReceivePacket64Bit(XBee::ReceivePacket64Bit::Struct *frame) = 0;

    virtual void incorrectChecksum(uint8_t calculated, uint8_t received) = 0;

    virtual void didCycle() = 0;

    void parseReceivePacket(const uint8_t *frame, uint8_t length);

    void parseReceivePacket64Bit(const uint8_t *frame, uint8_t length_bytes);

    bool handleFrame(const uint8_t *frame);

    virtual void _handleAtCommandResponse(const uint8_t *frame, uint8_t length_bytes, bool paramWasBeingWaitedOn);

    void handleAtCommandResponse(const uint8_t *frame, uint8_t length_bytes);

    virtual void _handleRemoteAtCommandResponse(const uint8_t *frame, uint8_t length_bytes, bool paramWasBeingWaitedOn);

    void handleRemoteAtCommandResponse(const uint8_t *frame, uint8_t length_bytes);

    void handleNodeDiscoveryResponse(const uint8_t *frame, uint8_t length_bytes);

    uint8_t *transmitRequestFrame;
    uint8_t *atCommandFrame;

    uint8_t currentFrameID;

    CircularQueue<XBee::BasicFrame> *transmitFrameQueue;

    XBee::BasicFrame tempFrame{};

    CircularQueue<uint16_t> *atParamConfirmationsBeingWaitedOn;

    XBee::ReceivePacket::Struct *receivePacketStruct = new XBee::ReceivePacket::Struct;
    XBee::ReceivePacket64Bit::Struct *receivePacket64BitStruct = new XBee::ReceivePacket64Bit::Struct;

    uint8_t *receiveFrame;
    char *nodeID;
    CircularBuffer *buffer;
};


#endif //HPRC_XBEEDEVICE_H
