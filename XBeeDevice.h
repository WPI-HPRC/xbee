//
// Created by William Scheirey on 3/12/24.
//

#ifndef HPRC_XBEEDEVICE_H
#define HPRC_XBEEDEVICE_H

#include "XBeeUtility.h"
#include "circularQueue.hpp"

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

    bool logTransmitStatus = false;

    XBee::ApiOptions::ApiOptions apiOptions;

private:
    virtual void writeBytes(const char *data, size_t length_bytes) = 0;

    virtual void readBytes(uint8_t *buffer, size_t length_bytes) = 0;

    virtual void packetRead() = 0;

    virtual void _handleRemoteAtCommandResponse(const uint8_t *frame, uint8_t length_bytes);

    virtual void _handleAtCommandResponse(const uint8_t *frame, uint8_t length_bytes);

    virtual void remoteDeviceDiscovered(XBee::RemoteDevice *device);

    virtual void handleReceivePacket(XBee::ReceivePacket::Struct *frame) = 0;

    virtual void handleReceivePacket64Bit(XBee::ReceivePacket64Bit::Struct *frame) = 0;

    virtual void _handleExtendedTransmitStatus(const uint8_t *frame, uint8_t length_bytes) = 0;

    virtual void handleTransmitStatus(const uint8_t *frame, uint8_t length_bytes);

    virtual void incorrectChecksum(uint8_t calculated, uint8_t received) = 0;

    virtual void didCycle();

    virtual void sentFrame(uint8_t frameID);

    void parseReceivePacket(const uint8_t *frame, uint8_t length);

    void parseReceivePacket64Bit(const uint8_t *frame, uint8_t length_bytes);

    void parseExplicitReceivePacket(const uint8_t *frame, uint8_t length_bytes);

    bool handleFrame(const uint8_t *frame);

    void handleAtCommandResponse(const uint8_t *frame, uint8_t length_bytes);

    void handleRemoteAtCommandResponse(const uint8_t *frame, uint8_t length_bytes);

    void handleNodeDiscoveryResponse(const uint8_t *frame, uint8_t length_bytes);

    void handleExtendedTransmitStatus(const uint8_t *frame, uint8_t length_bytes);

    uint8_t *transmitRequestFrame;
    uint8_t *atCommandFrame;

    uint8_t currentFrameID;

    CircularQueue<XBee::BasicFrame> *frameQueue;

    XBee::BasicFrame tempFrame{};

    bool waitingOnAtCommandResponse = false;
    bool waitingOnTransmitStatus = false;

    bool sendNextFrameImmediately = false;
    bool dontWaitOnNextFrame = false;

    XBee::ReceivePacket::Struct *receivePacketStruct = new XBee::ReceivePacket::Struct;
    XBee::ReceivePacket64Bit::Struct *receivePacket64BitStruct = new XBee::ReceivePacket64Bit::Struct;

    uint8_t *receiveFrame;
    char *nodeID;

protected:
    static uint8_t calcChecksum(const uint8_t *packet, uint8_t size_bytes);

    static uint8_t getFrameType(const uint8_t *packet);

    static uint8_t getFrameID(const uint8_t *packet);

    static uint64_t getAddressBigEndian(const uint8_t *packet, size_t *index_io);

    static uint64_t getAddressBigEndian(const uint8_t *packet);

    static uint64_t getAddressLittleEndian(const uint8_t *packet, size_t *index_io);

    static uint64_t getAddressLittleEndian(const uint8_t *packet);

    static void loadAddressBigEndian(uint8_t *packet, uint64_t address, size_t *index_io);

    static void loadAddressBigEndian(uint8_t *packet, uint64_t address);

    static uint16_t getAtCommand(const uint8_t *frame);

    static uint16_t getRemoteAtCommand(const uint8_t *frame);

};


#endif //HPRC_XBEEDEVICE_H
