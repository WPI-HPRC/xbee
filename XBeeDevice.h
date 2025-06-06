//
// Created by William Scheirey on 3/12/24.
//

#ifndef HPRC_XBEEDEVICE_H
#define HPRC_XBEEDEVICE_H

#define UART_BUFFER_SIZE 2048
#define SPI_BUFFER_SIZE 512 // should be more than enough - max bytes is ~260 but can change based on packet type

#include "XBeeUtility.h"
#include "circularQueue.hpp"

enum class SerialInterface
{
    UART,
    SPI
};

class XBeeDevice
{
public:
    explicit XBeeDevice(SerialInterface serialInterface);
    static void reverseBytes(void *start, int size);

    virtual void start();
    virtual void log(const char *format, ...) = 0;

    void sendFrame(uint8_t *frame, size_t size_bytes);
    void sendTransmitRequestCommand(uint64_t address, bool useFrameID, uint8_t transmitOptions, uint8_t broadcastRadius, const uint8_t *data, size_t size_bytes);
    void sendTransmitRequestCommand(uint64_t address, uint8_t transmitOptions, uint8_t broadcastRadius, const uint8_t *data, size_t size_bytes);
    void sendTransmitRequestCommand(uint64_t address, const uint8_t *data, size_t size_bytes);
    void sendExplicitAddressingCommand(XBee::ExplicitAddressingCommand::Struct frameInfo, uint8_t *data, size_t dataSize_bytes);

    void sendNodeDiscoveryCommand();
    virtual void sendLinkTestRequest(uint64_t destinationAddress, uint16_t payloadSize, uint16_t iterations);
    void sendEnergyDetectCommand(uint16_t msPerChannel);

    void queueAtCommandLocal(uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes);
    void
    sendAtCommandLocal(uint8_t frameType, uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes);
    void sendAtCommandLocal(uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes);
    void
    sendAtCommandRemote(uint64_t address, uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes);
    void sendAtCommandRemote(uint64_t address, uint8_t frameType, uint16_t command, const uint8_t *commandData,
                             size_t commandDataSize_bytes);

    void setParameterRemote(uint64_t address, uint16_t parameter, const uint8_t *value, size_t valueSize_bytes);
    void setParameterRemote(uint64_t address, uint16_t parameter, uint8_t value);
    void setParameter(uint16_t parameter, const uint8_t *value, size_t valueSize_bytes);
    void setParameter(uint16_t parameter, uint8_t value);

    void queryParameterRemote(uint64_t address, uint16_t parameter);
    void queryParameter(uint16_t parameter);

    void applyChanges();
    void writeChanges();

    void receiveKnownBytes(const uint8_t *bytes, size_t numBytes);
    void receive();

    void doCycle();
    
    bool sendTransmitRequestsImmediately = false;
    bool sendFramesImmediately = false;

    bool logWrongChecksums = true;
    bool logTransmitStatus = false;

    bool sendNextFrameImmediately = false;
    bool dontWaitOnNextFrame = false;

    bool recordThroughput = true;

private:
    void writeBytes(char *data, size_t length_bytes);
    virtual void writeBytes_uart(const char *data, size_t length_bytes){}
    virtual void writeBytes_spi(char *data_io, size_t length_bytes){}

    virtual size_t readBytes_uart(char *buffer, size_t max_bytes);
    virtual void readBytes_spi(uint8_t *buffer, size_t length_bytes);

    virtual void handlingFrame(const uint8_t *frame){}

    virtual void _handleRemoteAtCommandResponse(const uint8_t *frame, uint8_t length_bytes);
    virtual void _handleAtCommandResponse(const uint8_t *frame, uint8_t length_bytes);
    void _handleEnergyDetectResponse(const uint8_t *frame, size_t length_bytes);
    virtual void handleEnergyDetectResponse(uint8_t energyValues[XBee::MaxNumberOfChannels], uint8_t numChannels);

    virtual void remoteDeviceDiscovered(XBee::RemoteDevice *device);

    virtual void handleLinkTest(XBee::ExplicitRxIndicator::LinkTest data){}

    virtual void handleReceivePacket(XBee::ReceivePacket::Struct *frame) = 0;
    virtual void handleReceivePacket64Bit(XBee::ReceivePacket64Bit::Struct *frame) = 0;

    virtual void _handleExtendedTransmitStatus(const uint8_t *frame, uint8_t length_bytes);
    virtual void _handleTransmitStatus(uint8_t frameID, uint8_t statusCode){}
    void handleTransmitStatus(const uint8_t *frame, uint8_t length_bytes);

    virtual void incorrectChecksum(uint8_t calculated, uint8_t received) = 0;
    virtual void didCycle();
    virtual void sentFrame(uint8_t frameID);

    virtual bool canReadSPI(){return false;}

    void parseReceivePacket(const uint8_t *frame, uint8_t length);
    void parseReceivePacket64Bit(const uint8_t *frame, uint8_t length_bytes);
    void parseExplicitReceivePacket(const uint8_t *frame, uint8_t length_bytes);

    bool handleFrame(const uint8_t *frame);
    void handleAtCommandResponse(const uint8_t *frame, uint8_t length_bytes);
    void handleRemoteAtCommandResponse(const uint8_t *frame, uint8_t length_bytes);
    void handleNodeDiscoveryResponse(const uint8_t *frame, uint8_t length_bytes);
    void handleExtendedTransmitStatus(const uint8_t *frame, uint8_t length_bytes);

    uint8_t transmitFrame[XBee::MaxFrameBytes]{};

    uint8_t currentFrameID;

    CircularQueue<XBee::BasicFrame> *frameQueue;

    XBee::BasicFrame tempFrame{};

    bool waitingOnAtCommandResponse = false;
    bool waitingOnTransmitStatus = false;

    XBee::ReceivePacket::Struct *receivePacketStruct = new XBee::ReceivePacket::Struct;
    XBee::ReceivePacket64Bit::Struct *receivePacket64BitStruct = new XBee::ReceivePacket64Bit::Struct;

    char uartBuffer[UART_BUFFER_SIZE]{};
    uint8_t receiveFrame[XBee::MaxPacketBytes]{};
    uint8_t receiveFrameIndex = 0;
    int receiveFrameBytesLeftToRead = 0;


    char nodeID[20]{};

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

    SerialInterface serialInterface;
};


#endif //HPRC_XBEEDEVICE_H
