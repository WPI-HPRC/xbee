//
// Created by William Scheirey on 3/12/24.
//

#include "XBeeDevice.h"
#include <string>
#include <cstring>
#include <iostream>
#include <iomanip>

#include <algorithm>

void XBeeDevice::reverseBytes( void *start, int size )
{
    char *istart = static_cast<char *>(start), *iend = istart + size;
    std::reverse(istart, iend);
}

uint8_t XBeeDevice::calcChecksum(const uint8_t *packet, uint8_t size_bytes)
{
    uint8_t sum = 0;

    for (uint8_t i = 0; i < size_bytes; i++)
    {
        sum += packet[3 + i]; // Skip start delimiter and length bytes
    }

    return 0xFF - sum;
}

uint8_t XBeeDevice::getFrameType(const uint8_t *packet)
{
    return packet[3];
}

uint8_t XBeeDevice::getFrameID(const uint8_t *packet)
{
    return packet[4];
}

uint64_t XBeeDevice::getAddressBigEndian(const uint8_t *packet, size_t *index_io)
{
    uint64_t address = 0;

    for (int i = 0; i < 8; i++)
    {
        address |= (uint64_t) packet[*index_io] << (8 * (7 - i));
        (*index_io)++;
    }

    return address;
}

uint64_t XBeeDevice::getAddressBigEndian(const uint8_t *packet)
{
    size_t _ = 0;

    return getAddressBigEndian(packet, &_);
}

uint64_t XBeeDevice::getAddressLittleEndian(const uint8_t *packet, size_t *index_io)
{
    uint64_t address = 0;

    for (int i = 0; i < 8; i++)
    {
        address |= (uint64_t) packet[*index_io] << (8 * i);
        (*index_io)++;
    }

    return address;
}

uint64_t XBeeDevice::getAddressLittleEndian(const uint8_t *packet)
{
    size_t _ = 0;

    return getAddressLittleEndian(packet, &_);
}

void XBeeDevice::loadAddressBigEndian(uint8_t *packet, uint64_t address, size_t *index_io)
{
    for (int i = 0; i < 8; i++)
    {
        packet[*index_io] = (address >> ((7 - i) * 8)) & 0xFF;
        (*index_io)++;
    }
}

void XBeeDevice::loadAddressBigEndian(uint8_t *packet, uint64_t address)
{
    size_t _ = 0;
    loadAddressBigEndian(packet, address, &_);
}

uint16_t XBeeDevice::getAtCommand(const uint8_t *frame)
{
    return frame[XBee::AtCommandResponse::BytesBeforeCommand] << 8 |
           frame[XBee::AtCommandResponse::BytesBeforeCommand + 1];
}

uint16_t XBeeDevice::getRemoteAtCommand(const uint8_t *frame)
{
    return frame[XBee::RemoteAtCommandResponse::BytesBeforeCommand] << 8 |
           frame[XBee::RemoteAtCommandResponse::BytesBeforeCommand + 1];
}

XBeeDevice::XBeeDevice(SerialInterface serialInterface): serialInterface(serialInterface)
{
    frameQueue = circularQueueCreate<XBee::BasicFrame>(255);

    currentFrameID = 1;
}

void XBeeDevice::queryParameter(uint16_t parameter)
{
    sendAtCommandLocal(parameter, nullptr, 0);
}

void XBeeDevice::queryParameterRemote(uint64_t address, uint16_t parameter)
{
    sendAtCommandRemote(address, parameter, nullptr, 0);
}

void XBeeDevice::setParameter(uint16_t parameter, const uint8_t value)
{
    setParameter(parameter, &value, 1);
}

void XBeeDevice::setParameterRemote(uint64_t address, uint16_t parameter, const uint8_t value)
{
    setParameterRemote(address, parameter, &value, 1);
}

void XBeeDevice::setParameter(uint16_t parameter, const uint8_t *value, size_t valueSize_bytes)
{
    sendAtCommandLocal(XBee::FrameType::AtCommand, parameter, value, valueSize_bytes);
}

void XBeeDevice::setParameterRemote(uint64_t address, uint16_t parameter, const uint8_t *value, size_t valueSize_bytes)
{
    sendAtCommandRemote(address, parameter, value, valueSize_bytes);
}

void XBeeDevice::sendAtCommandRemote(uint64_t address, uint16_t command, const uint8_t *commandData,
                                     size_t commandDataSize_bytes)
{
    sendAtCommandRemote(address, XBee::FrameType::RemoteAtCommandRequest, command, commandData, commandDataSize_bytes);
}

void XBeeDevice::sendAtCommandLocal(uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes)
{
    sendAtCommandLocal(XBee::FrameType::AtCommand, command, commandData, commandDataSize_bytes);
}

void XBeeDevice::queueAtCommandLocal(uint16_t command, const uint8_t *commandData, size_t commandDataSize_bytes)
{
    sendAtCommandLocal(XBee::FrameType::AtCommandQueueParameterValue, command, commandData, commandDataSize_bytes);
}

void XBeeDevice::sendAtCommandLocal(uint8_t frameType, uint16_t command, const uint8_t *commandData,
                                    size_t commandDataSize_bytes)
{
    // NOTE: command data must be converted to Big Endian
    using namespace XBee::AtCommandTransmit;

    size_t index = 1;
    size_t contentLength_bytes = PacketBytes + commandDataSize_bytes;

    transmitFrame[index++] = (contentLength_bytes >> 8) & 0xFF;
    transmitFrame[index++] = contentLength_bytes & 0xFF;

    transmitFrame[index++] = frameType; // Local AT Command Request
    transmitFrame[index++] = currentFrameID++; // Frame ID

    transmitFrame[index++] = (command >> 8) & 0xFF;
    transmitFrame[index++] = command & 0xFF;

    if (commandData)
    {
        memcpy(&transmitFrame[index], commandData, commandDataSize_bytes);
    }

    sendFrame(transmitFrame, commandDataSize_bytes + FrameBytes);
}

void XBeeDevice::sendAtCommandRemote(uint64_t address, uint8_t frameType, uint16_t command, const uint8_t *commandData,
                                     size_t commandDataSize_bytes)
{
    using namespace XBee::RemoteAtCommandTransmit;

    size_t index = 1;
    size_t contentLength_bytes = PacketBytes + commandDataSize_bytes;

    transmitFrame[index++] = (contentLength_bytes >> 8) & 0xFF;
    transmitFrame[index++] = contentLength_bytes & 0xFF;

    transmitFrame[index++] = frameType; // Local AT Command Request
    transmitFrame[index++] = currentFrameID++; // Frame ID

    loadAddressBigEndian(transmitFrame, address, &index);

    transmitFrame[index++] = 0xFF; // Reserved
    transmitFrame[index++] = 0xFE; // Reserved

    transmitFrame[index++] = 0x01;

    transmitFrame[index++] = (command >> 8) & 0xFF;
    transmitFrame[index++] = command & 0xFF;

    if (commandData)
    {
        memcpy(&transmitFrame[index], commandData, commandDataSize_bytes);
    }

    sendFrame(transmitFrame, commandDataSize_bytes + FrameBytes);
}

void XBeeDevice::applyChanges()
{
    sendAtCommandLocal(XBee::AtCommand::ApplyChanges, nullptr, 0);
}

void XBeeDevice::writeChanges()
{
    sendAtCommandLocal(XBee::AtCommand::Write, nullptr, 0);
}

void XBeeDevice::sendNodeDiscoveryCommand()
{
    setParameter(XBee::AtCommand::NodeDiscoveryBackoff, 0x20);
    setParameter(XBee::AtCommand::NodeDiscoveryOptions, 0x02);
    sendAtCommandLocal(XBee::AtCommand::NodeDiscovery, nullptr, 0);
}

void XBeeDevice::sendTransmitRequestCommand(uint64_t address, const uint8_t *data, size_t size_bytes)
{
    sendTransmitRequestCommand(address, 0x00, 0x00, data, size_bytes);
}

void XBeeDevice::sendTransmitRequestCommand(uint64_t address, uint8_t transmitOptions, uint8_t broadcastRadius, const uint8_t *data, size_t size_bytes)
{
    using namespace XBee::TransmitRequest;

    size_t contentLength_bytes = size_bytes + PacketBytes;
    size_t index = 1; // skip first byte (start delimiter)

    transmitFrame[index++] = (contentLength_bytes >> 8) & 0xFF;
    transmitFrame[index++] = contentLength_bytes & 0xFF;

    transmitFrame[index++] = 0x10; // Transmit Request
    currentFrameID = currentFrameID == 0 ? 1 : currentFrameID;
    transmitFrame[index++] = currentFrameID++; // Frame ID

    loadAddressBigEndian(transmitFrame, address, &index);

    transmitFrame[index++] = 0xFF; // Reserved
    transmitFrame[index++] = 0xFE; // Reserved

    transmitFrame[index++] = broadcastRadius; // Broadcast radius
    transmitFrame[index++] = transmitOptions; // Transmit options

    memcpy(&transmitFrame[index], data, size_bytes);
    sendFrame(transmitFrame, size_bytes + FrameBytes);
}

void XBeeDevice::sendExplicitAddressingCommand(XBee::ExplicitAddressingCommand::Struct frameInfo, uint8_t *data,
                                               size_t dataSize_bytes)
{
    using namespace XBee::ExplicitAddressingCommand;

    size_t contentLength_bytes = dataSize_bytes + PacketBytes;
    size_t index = 1;

    transmitFrame[index++] = (contentLength_bytes >> 8) & 0xFF;
    transmitFrame[index++] = contentLength_bytes & 0xFF;
    transmitFrame[index++] = 0x11;
    currentFrameID = currentFrameID == 0 ? 1 : currentFrameID;
    transmitFrame[index++] = currentFrameID++;
    memcpy(&transmitFrame[index], (uint8_t *)&frameInfo, sizeof(frameInfo));
    index += sizeof(frameInfo);
    memcpy(&transmitFrame[index], data, dataSize_bytes);

    sendFrame(transmitFrame, dataSize_bytes + FrameBytes);
}

void XBeeDevice::sendLinkTestRequest(uint64_t destinationAddress, uint16_t payloadSize, uint16_t iterations)
{
    using namespace XBee::ExplicitAddressingCommand;

    Struct frameInfo;
    frameInfo.destinationAddress = 0x0013A200422CDAC2;
    reverseBytes(&frameInfo.destinationAddress, sizeof(frameInfo.destinationAddress));
    frameInfo.reserved = 0xFFFE;
    reverseBytes(&frameInfo.reserved, sizeof(frameInfo.reserved));
    frameInfo.sourceEndpoint = 0xE6;
    reverseBytes(&frameInfo.sourceEndpoint, sizeof(frameInfo.sourceEndpoint));
    frameInfo.destinationEndpoint = 0xE6;
    reverseBytes(&frameInfo.destinationEndpoint, sizeof(frameInfo.destinationEndpoint));
    frameInfo.clusterID = 0x0014;
    reverseBytes(&frameInfo.clusterID, sizeof(frameInfo.clusterID));
    frameInfo.profileID = 0xC105;
    reverseBytes(&frameInfo.profileID, sizeof(frameInfo.profileID));
    frameInfo.broadcastRadius = 0;
    reverseBytes(&frameInfo.broadcastRadius, sizeof(frameInfo.broadcastRadius));
    frameInfo.transmitOptions = 0;
    reverseBytes(&frameInfo.transmitOptions, sizeof(frameInfo.transmitOptions));

    LinkTest linkTest = {
            .destinationAddress = 0x0013A200423F474C,
            .payloadSize = payloadSize,
            .iterations = iterations
    };
    reverseBytes(&linkTest.destinationAddress, sizeof(linkTest.destinationAddress));
    reverseBytes(&linkTest.payloadSize, sizeof(linkTest.payloadSize));
    reverseBytes(&linkTest.iterations, sizeof(linkTest.iterations));

    sendExplicitAddressingCommand(frameInfo, (uint8_t *)&linkTest, sizeof(linkTest));
}

void XBeeDevice::sendEnergyDetectCommand(uint16_t msPerChannel)
{
    sendAtCommandLocal(XBee::AtCommand::EnergyDetect, nullptr, 0);
}

void XBeeDevice::sendFrame(uint8_t *frame, size_t size_bytes)
{
    frame[0] = 0x7E; // Start delimiter;

    frame[size_bytes - 1] = calcChecksum(frame, size_bytes - XBee::FrameBytes);

    if (sendFramesImmediately || sendNextFrameImmediately ||
        (getFrameType(frame) == XBee::FrameType::TransmitRequest && sendTransmitRequestsImmediately))
    {
        sendNextFrameImmediately = false;
        writeBytes((const char *) frame, size_bytes);
    }
    else
    {
        tempFrame.length_bytes = size_bytes;
        memcpy(tempFrame.frame, frame, size_bytes);

        circularQueuePush(frameQueue, tempFrame);
    }
    sentFrame(currentFrameID - 1);
}

void XBeeDevice::parseReceivePacket(const uint8_t *frame, uint8_t length_bytes)
{
    using namespace XBee::ReceivePacket;

    uint8_t payloadLength =
            length_bytes - PacketBytes; // Subtract the number of base frame bytes from the total number of frame bytes

    uint64_t addr = getAddressLittleEndian(&frame[BytesBeforeAddress]);

    receivePacketStruct->dataLength_bytes = payloadLength;
    receivePacketStruct->senderAddress = addr;
    receivePacketStruct->data = &frame[BytesBeforePayload];

    handleReceivePacket(receivePacketStruct);
}

void XBeeDevice::parseExplicitReceivePacket(const uint8_t *frame, uint8_t length_bytes)
{
    using namespace XBee::ExplicitRxIndicator;

    auto *frameStruct = (_Struct *)&frame[3];
    reverseBytes(&frameStruct->clusterID, sizeof(frameStruct->clusterID));

    if(frameStruct->clusterID == XBee::LinkTestClusterID)
    {
        auto *data = (LinkTest *)(&frame[21]);

        reverseBytes(&data->destinationAddress, sizeof(data->destinationAddress));
        reverseBytes(&data->success, sizeof(data->success));
        reverseBytes(&data->iterations, sizeof(data->iterations));
        reverseBytes(&data->payloadSize, sizeof(data->payloadSize));
        reverseBytes(&data->retries, sizeof(data->retries));
        handleLinkTest(*data);
        return;
    }

    uint8_t payloadLength =
            length_bytes - PacketBytes; // Subtract the number of base frame bytes from the total number of frame bytes

    uint64_t addr = getAddressLittleEndian(&frame[BytesBeforeAddress]);
    receivePacketStruct->dataLength_bytes = payloadLength;
    receivePacketStruct->senderAddress = addr;
    receivePacketStruct->data = &frame[XBee::ExplicitRxIndicator::BytesBeforePayload];

    handleReceivePacket(receivePacketStruct);
}

void XBeeDevice::parseReceivePacket64Bit(const uint8_t *frame, uint8_t length_bytes)
{
    using namespace XBee::ReceivePacket64Bit;

    uint8_t payloadLength =
            length_bytes - PacketBytes; // Subtract the number of base frame bytes from the total number of frame bytes

    uint64_t addr = getAddressLittleEndian(&frame[BytesBeforeAddress]);

    receivePacket64BitStruct->dataLength_bytes = payloadLength;
    receivePacket64BitStruct->senderAddress = addr;
    receivePacket64BitStruct->data = &frame[XBee::ReceivePacket64Bit::BytesBeforePayload];
    receivePacket64BitStruct->negativeRssi = frame[XBee::ReceivePacket64Bit::BytesBeforeRssi];

    handleReceivePacket64Bit(receivePacket64BitStruct);
}

void XBeeDevice::remoteDeviceDiscovered(XBee::RemoteDevice *device)
{
    log("Found device: ");
    for (int i = 0; i < device->idLength; i++)
    {
        log("%c", device->id[i]);
    }
    log(" - %016llx\n", static_cast<unsigned long long>(device->serialNumber));
}

void XBeeDevice::handleNodeDiscoveryResponse(const uint8_t *frame, uint8_t length_bytes)
{
    auto *device = new XBee::RemoteDevice;

    size_t index = XBee::AtCommandResponse::BytesBeforeCommandData + 2; // Add two to skip the MY parameter

    device->serialNumber = getAddressBigEndian(frame, &index);

    int iDLength = 0;

    for (int i = 0; i < 20; i++)
    {
        uint8_t byte = frame[index++];

        if (byte == 0x00)
        {
            if (i > 0)
            {
                iDLength = i;
            } // Remember that there is an extra byte in the frame; this null character
            break;
        }

        nodeID[i] = (char) byte;
    }

    memcpy(device->id, nodeID, iDLength);
    device->idLength = iDLength;

    device->deviceType = frame[index++];
    device->status = frame[index++];

    device->profileID = frame[index++] << 8;
    device->profileID |= frame[index++];

    device->manufacturerID = frame[index++] << 8;
    device->manufacturerID |= frame[index++];

    remoteDeviceDiscovered(device);
}

void XBeeDevice::_handleAtCommandResponse(const uint8_t *frame, uint8_t length_bytes)
{
    // This function is marked virtual but is optional to override
    using namespace XBee::AtCommandResponse;

    uint16_t command = getAtCommand(frame);

    log("AT command response for %c%c: ", (command & 0xFF00) >> 8, command & 0x00FF);

    for (uint8_t i = 0; i < length_bytes - PacketBytes; i++)
    {
        log("%d ", (int) (frame[BytesBeforeCommandData + i] & 0xFF));
    }

    log("\n");
}

void XBeeDevice::handleEnergyDetectResponse(uint8_t *energyValues, uint8_t numChannels)
{
    log("Energy detect response:\n");
    for(int i = 0; i < numChannels; i++)
    {
        log("\tChannel %d: %d\n", i, energyValues[i]);
    }
}

void XBeeDevice::_handleEnergyDetectResponse(const uint8_t *frame, size_t length_bytes)
{
    uint8_t energyValues[XBee::MaxNumberOfChannels];
    uint8_t channelNum = 0;

    for(int i = XBee::AtCommandResponse::BytesBeforeCommandData; i < length_bytes; i++)
    {
        if(frame[i] == 0x0D)
        {
            // Carriage return, data is done
            break;
        }
        else if (frame[i] == 0x2C)
        {
            // Comma separated
            continue;
        }
        else
        {
            energyValues[channelNum++] = frame[i];
        }
    }
    handleEnergyDetectResponse(energyValues, channelNum);
}

void XBeeDevice::handleAtCommandResponse(const uint8_t *frame, uint8_t length_bytes)
{
    using namespace XBee::AtCommandResponse;

    uint8_t commandStatus = frame[BytesBeforeCommandStatus];

    uint16_t command = getAtCommand(frame);

    if (commandStatus != XBee::AtCommand::Ok)
    {
        log("AT command response for %c%c: ", (command & 0xFF00) >> 8, command & 0x00FF);
        switch (commandStatus)
        {
            case XBee::AtCommand::Error:
                log("Error in command\n");
                break;
            case XBee::AtCommand::InvalidCommand:
                log("Invalid command\n");
                break;
            case XBee::AtCommand::InvalidParameter:
                log("Invalid parameter\n");
                break;
            default:
                log("You have broken physics. Received %02x\n", (int) (commandStatus & 0xFF));
                break;
        }
        return;
    }
    else if (length_bytes == PacketBytes)
    {
        log("AT command response for %c%c: OK\n", (command & 0xFF00) >> 8, command & 0x00FF);
        return;
    }

    switch (command)
    {
        case XBee::AtCommand::NodeDiscovery:
            handleNodeDiscoveryResponse(frame, length_bytes);
            return;
        case XBee::AtCommand::EnergyDetect:
            _handleEnergyDetectResponse(frame, length_bytes);
            return;
        default:
            break;
    }

    _handleAtCommandResponse(frame, length_bytes);
}

void XBeeDevice::_handleRemoteAtCommandResponse(const uint8_t *frame, uint8_t length_bytes)
{
    // This function is marked virtual but is optional to override
    using namespace XBee::RemoteAtCommandResponse;

    uint16_t command = getRemoteAtCommand(frame);

    uint64_t address = getAddressBigEndian(&frame[BytesBeforeAddress]);

    log("Remote AT response from %016llx: ", (unsigned long long) address);
    log("%c%c: ", (command & 0xFF00) >> 8, command & 0x00FF);
    for (uint8_t i = 0; i < length_bytes - PacketBytes; i++)
    {
        log("%02x ", (int) (frame[BytesBeforeCommandData + i] & 0xFF));
    }

    log("\n");
}

void XBeeDevice::handleRemoteAtCommandResponse(const uint8_t *frame, uint8_t length_bytes)
{
    using namespace XBee::RemoteAtCommandResponse;

    uint8_t commandStatus = frame[BytesBeforeCommandStatus];

    uint16_t command = getRemoteAtCommand(frame);

    if (commandStatus != 0x00)
    {
        log("Remote AT command response for %c%c: ", (command & 0xFF00) >> 8, command & 0x00FF);
        switch (commandStatus)
        {
            case XBee::AtCommand::Error:
                log("Error in command\n");
                break;
            case XBee::AtCommand::InvalidCommand:
                log("Invalid command\n");
                break;
            case XBee::AtCommand::InvalidParameter:
                log("Invalid parameter\n");
                break;
            case XBee::RemoteAtCommand::TransmissionFailure:
                log("Transmission failure\n");
                break;
            default:
                log("You have broken physics. Received %02x\n", (int) (commandStatus & 0xFF));
                break;
        }
        return;
    }
    else if (length_bytes == PacketBytes)
    {
        log("AT command response for %c%c: OK\n", (command & 0xFF00) >> 8, command & 0x00FF);
        return;
    }

    _handleRemoteAtCommandResponse(frame, length_bytes);
}

void XBeeDevice::handleTransmitStatus(const uint8_t *frame, uint8_t length_bytes)
{
    using namespace XBee::TransmitStatus;

    uint8_t frameID = frame[BytesBeforeFrameID];
    uint8_t statusCode = frame[BytesBeforeStatus];

    if (logTransmitStatus)
    {
        log("Transmit Status for frame ID %03x -- [%02x]: ", (int) frameID, (int) statusCode);

        switch (statusCode)
        {
            case Success:
                log("Success");
                break;
            case NoAckReceived:
                log("No Ack Received");
                break;
            case CcaFailure:
                log("CCA Failure");
                break;
            case IndirectMessageUnrequested:
                log("Indirect Message Unrequested");
                break;
            case TransceiverUnableToCompleteTransmission:
                log("Transceiver Unable to Complete Transmission");
                break;
            case NetworkAckFailure:
                log("Network ACK Failure");
                break;
            case NotJoinedToNetwork:
                log("Not Joined to Network");
                break;
            case InvalidFrameValues:
                log("Invalid Frame Values (check the phone number)");
                break;
            case InternalError:
                log("Internal Error");
                break;
            case ResourceError:
                log("Resource Error - lack of free buffers, timers, etc.");
                break;
            case NoSecureSessionConnection:
                log("No Secure Session Connection");
                break;
            case EncryptionFailure:
                log("Encryption Failure");
                break;
            case MessageTooLong:
                log("Message Too Long");
                break;
            case SocketClosedUnexpectedly:
                log("Socket Closed Unexpectedly");
                break;
            case InvalidUdpPort:
                log("Invalid UDP Port");
                break;
            case InvalidTcpPort:
                log("Invalid TCP Port");
                break;
            case InvalidHostAddress:
                log("Invalid Host Address");
                break;
            case InvalidDataMode:
                log("Invalid Data Mode");
                break;
            case InvalidInterface:
                log("Invalid Interface");
                break;
            case InterfaceNotAcceptingFrames:
                log("Interface Not Accepting Frames");
                break;
            case ModemUpdateInProgress:
                log("A Modem Update is in Progress. Try again after the update is complete.");
                break;
            case ConnectionRefused:
                log("Connection Refused");
                break;
            case SocketConnectionLost:
                log("Socket Connection Lost");
                break;
            case NoServer:
                log("No Server");
                break;
            case SocketClosed:
                log("Socket Closed");
                break;
            case UnknownServer:
                log("Unknown Server");
                break;
            case UnknownError:
                log("Unknown Error");
                break;
            case InvalidTlsConfiguration:
                log("Invalid TLS Configuration (missing file, and so forth)");
                break;
            case SocketNotConnected:
                log("Socket Not Connected");
                break;
            case SocketNotBound:
                log("Socket Not Bound");
                break;
            default:
                log("Unknown Status Code");
                break;
        }
        log("\n");
    }
    _handleTransmitStatus(frameID, statusCode);
}

void XBeeDevice::_handleExtendedTransmitStatus(const uint8_t *frame, uint8_t length_bytes)
{
    // Optional to implement
}

void XBeeDevice::handleExtendedTransmitStatus(const uint8_t *frame, uint8_t length_bytes)
{
    using namespace XBee::ExtendedTransmitStatus;

    uint8_t frameID = frame[BytesBeforeFrameID];
    uint8_t statusCode = frame[BytesBeforeStatus];
    uint8_t retryCount = frame[BytesBeforeRetryCount];
    uint8_t discovery = frame[BytesBeforeDiscovery];

    if (logTransmitStatus)
    {
        log("Extended Transmit Status for frame ID %03x -- [%02x]: ", (int) frameID, (int) statusCode);

        switch (statusCode)
        {
            case Success:
                log("Success");
                break;
            case MacAckFailure:
                log("MAC ACK Failure");
                break;
            case CcaLbtFailure:
                log("CCA/LBT Failure");
                break;
            case IndirectMessageUnrequestedNoSpectrum:
                log("Indirect Message Unrequested / No Spectrum Available");
                break;
            case NetworkAckFailure:
                log("Network ACK Failure");
                break;
            case RouteNotFound:
                log("Route Not Found");
                break;
            case InternalResourceError:
                log("Internal Resource Error");
                break;
            case ResourceError:
                log("Resource Error - Lack of Free Buffers, Timers, etc.");
                break;
            case DataPayloadTooLarge:
                log("Data Payload Too Large");
                break;
            case IndirectMessageUnrequested:
                log("Indirect Message Unrequested");
                break;
            default:
                log("Unknown Status Code");
                break;
        }

        log(". Discovery: ");

        switch (discovery)
        {
            case 0x00:
                log("No discovery overhead");
                break;
            case 0x02:
                log("Route Discovery");
                break;
            default:
                log("Unknown: %02x", discovery);
        }

        log(". Retries: %d\n", (int) retryCount);
    }

    _handleExtendedTransmitStatus(frame, length_bytes);
}

bool XBeeDevice::handleFrame(const uint8_t *frame)
{
    size_t index = 1; // Skip start delimiter

    uint8_t lengthLow = frame[index++];
    uint8_t lengthHigh = frame[index++];

    handlingFrame(frame);

    uint8_t calculatedChecksum = calcChecksum(frame, lengthHigh);
    uint8_t receivedChecksum = frame[lengthHigh + XBee::FrameBytes - 1];

    if (calculatedChecksum != receivedChecksum)
    {
        if (logWrongChecksums)
        {
            log("Checksum mismatch. Calculated: %02x, received: %02x\n", (int) (calculatedChecksum & 0xFF),
                (int) (receivedChecksum & 0xFF));
        }
        incorrectChecksum(calculatedChecksum, receivedChecksum);
        return false;
    }

    uint8_t frameType = frame[index++];

    switch (frameType)
    {
        using namespace XBee::FrameType;
        case ReceivePacket:
            parseReceivePacket(frame, lengthHigh);
            break;

        case ReceivePacket64Bit:
            parseReceivePacket64Bit(frame, lengthHigh);
            break;

        case ExplicitRxIndicator:
            parseExplicitReceivePacket(frame, lengthHigh);
            break;

        case AtCommandResponse:
            waitingOnAtCommandResponse = false;
            handleAtCommandResponse(frame, lengthHigh);
            break;

        case RemoteAtCommandResponse:
            handleRemoteAtCommandResponse(frame, lengthHigh);
            break;

        case TransmitStatus:
            handleTransmitStatus(frame, lengthHigh);
            waitingOnTransmitStatus = false;
            break;

        case ExtendedTransmitStatus:
            waitingOnTransmitStatus = false;
            handleExtendedTransmitStatus(frame, lengthHigh);
            break;

        default:
            log("Unrecognized frame type: %02x\n", (int) (frameType & 0xFF));
            break;
    }
    return true;
}

void XBeeDevice::receive()
{
    if(serialInterface == SerialInterface::UART)
    {
        size_t numBytes = readBytes_uart(uartBuffer, UART_BUFFER_SIZE);
        for(int i = 0; i < numBytes; i++)
        {
            uint8_t next_byte = uartBuffer[i];
            if(receiveFrameBytesLeftToRead > 0)
            {
                receiveFrame[receiveFrameIndex++] = next_byte;
                receiveFrameBytesLeftToRead -= 1;

                if(receiveFrameIndex == 3)
                {
                    // We need to read the number of bytes denoted by the length byte, plus 1 for the checksum
                    receiveFrameBytesLeftToRead = receiveFrame[2] + 1;
                }

                if(receiveFrameBytesLeftToRead == 0)
                {
                    handleFrame(receiveFrame);

                    // Set the first byte to zero so we know the packet has been read
                    receiveFrame[0] = 0;
                }
            }
            else if(next_byte == XBee::StartDelimiter)
            {
                receiveFrameIndex = 0;
                receiveFrame[receiveFrameIndex++] = next_byte;
                // We need to read 3 more bytes: +1 for start delimiter, +2 for length bytes
                receiveFrameBytesLeftToRead = 3;
            }
        }
    }
    else if(serialInterface == SerialInterface::SPI)
    {
        /*
        readBytes(receiveFrame, 3);
        uint8_t length = receiveFrame[2];

        // Read the rest of the frame. The length represents the number of bytes between the length and the checksum.
        // The second of the two length bytes holds the real length of the frame.
        readBytes(&receiveFrame[3], length + 1);

        // Set the first byte to zero so we know the packet has been read
        handleFrame(receiveFrame);
        receiveFrame[0] = 0;
         */
    }
}

void XBeeDevice::doCycle()
{
    // First, read frames from serial
    receive();

    // Next, send out any frames that need to be sent

    while (true)
    {
        if (waitingOnAtCommandResponse ||
            waitingOnTransmitStatus || isCircularQueueEmpty(frameQueue))
        {
            break;
        }

        circularQueuePop(frameQueue, &tempFrame, 1);
        uint8_t frameType = getFrameType(tempFrame.frame);
        if (dontWaitOnNextFrame)
        {
            dontWaitOnNextFrame = false;
        }
        else
        {
            if (frameType == XBee::FrameType::AtCommandQueueParameterValue || frameType == XBee::FrameType::AtCommand)
            {
                waitingOnAtCommandResponse = true;
            }
        }
        writeBytes((const char *) tempFrame.frame, tempFrame.length_bytes);
    }
    didCycle();
}

void XBeeDevice::sentFrame(uint8_t frameID)
{
    // Optional to implement
}

void XBeeDevice::didCycle()
{
    // Optional to implement.
}

size_t XBeeDevice::readBytes_uart(char *buffer, size_t max_bytes)
{
    // Optional to implement
    return 0;
}

void XBeeDevice::readBytes_spi(uint8_t *buffer, size_t length_bytes)
{

}
void XBeeDevice::start()
{

}
