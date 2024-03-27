//
// Created by William Scheirey on 3/15/24.
//

#include <cstdint>

#define AsciiToUint16(x, y)((uint16_t)x << 8 | y)

#ifndef XBEE_BACKEND_CPP_XBEEUTILITY_H
#define XBEE_BACKEND_CPP_XBEEUTILITY_H

/*
 * XBee namespace with constants for utility.
 *
 * NOTE: PacketBytes and FrameBytes fields represent the *minimum* number of bytes for their respective fields
 */

namespace XBee
{
    const uint8_t StartDelimiter = 0x7E;

    namespace FrameType
    {
        enum FrameType
        {
            // --- Transmit ---

            AtCommand = 0x08,
            AtCommandQueueParameterValue = 0x09,
            TransmitRequest = 0x10,
            ExplicitAddressingCommandFrame = 0x11,
            RemoteAtCommandRequest = 0x17,
            CreateSourceRoute = 0x21,
            RegisterJoiningDevice = 0x24,

            // --- Receive ---

            AtCommandResponse = 0x88,
            ModemStatus = 0x8A,
            TransmitStatus = 0x8B,
            ReceivePacket = 0x90,
            ExplicitRxIndicator = 0x91,
            IoDataSampleRateRxIndicator = 0x92,
            XBeeSensorReadIndicator = 0x94,
            NodeIdentificationIndicator = 0x95,
            RemoteAtCommandResponse = 0x97,
            ExtendedModemStatus = 0x98,
            OTAFirmwareUpdateStatus = 0xA0,
            RouterRecordIndicator = 0xA1,
            MTORouteRequestIndicator = 0xA3,
            JoinNotificationStatus = 0xA5
        };
    }

    namespace AtCommand
    {
        enum AtCommand
        {
            NodeDiscovery = AsciiToUint16('N', 'D'),
            NodeDiscoveryOptions = AsciiToUint16('N', 'O'),
            NodeDiscoveryBackoff = AsciiToUint16('N', 'T'),
            Write = AsciiToUint16('W', 'R'),
            ApplyChanges = AsciiToUint16('A', 'C')
        };
        enum CommandStatus
        {
            Ok = 0,
            Error = 1,
            InvalidCommand = 2,
            InvalidParameter = 3
        };
    }


    struct BasicFrame
    {
        size_t length_bytes;
        uint8_t *frame;
    };

    const uint8_t MaxPacketBytes = 255;
    const uint8_t FrameBytes = 4; // +1 for start delimiter, +2 for length, +1 for checksum
    const uint16_t MaxFrameBytes = MaxPacketBytes + FrameBytes;

    // ------------- TRANSMIT -------------
    namespace AtCommandTransmit
    {
        const uint8_t PacketBytes = 4; // +1 for frame type, +1 for frame ID, +2 for AT command
        const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;

        namespace NodeDiscovery
        {
            const uint16_t CommandID = AsciiToUint16('N', 'D');
            const uint8_t PacketBytes = AtCommandTransmit::PacketBytes + 0;
            const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
        }
    }

    namespace AtCommandQueue
    {
        const uint8_t PacketBytes = 4;
        const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
    }

    namespace TransmitRequest
    {
        const uint8_t PacketBytes = 14; // +1 for frame type, +1 for frame ID, +8 for destination address, +2 for reserved, +1 for broadcast radius, +1 for transmit options
        const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
    }

    // ------------- RECEIVE -------------
    namespace ReceivePacket
    {
        const uint8_t PacketBytes = 12; // +1 for frame type, +8 for sender address, +2 for reserved, +1 for receive options
        const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
        const uint8_t BytesBeforePacket = 4;
        const uint8_t BytesBeforeAddress = 4;
        const uint8_t BytesBeforePayload = 15;

        struct Struct
        {
            uint8_t dataLength_bytes;
            uint64_t senderAddress;
            const uint8_t *data;
        };
    }


    namespace AtCommandResponse
    {
        const uint8_t PacketBytes = 5; // +1 for frame type, +1 for frame ID, +2 for AT command, +1 for command status
        const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
        const uint8_t BytesBeforeCommand = 5;
        const uint8_t BytesBeforeCommandStatus = 7;
        const uint8_t BytesBeforeCommandData = 8;

        namespace NodeDiscovery
        {
            const uint8_t PacketBytes = AtCommandResponse::PacketBytes +
                                        14; // +4 for SH, +4 for SL, +1 for Device Type, +1 for status, +2 for profile ID, +2 for manufacturer ID
            const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
            const uint8_t MaxPacketBytes = PacketBytes +
                                           26; // +20 for NI, +1 for NI null byte, +4 for Digi Device Type, +1 for RSSI of last hop
        }
    }

    // --- For Convenience ---
    /*
    namespace AtCommandResponseLocal = XBee::AtCommandResponse;
    namespace NodeDiscoveryTransmit = XBee::AtCommandTransmit::NodeDiscovery;
    namespace NodeDiscoveryResponseLocal = AtCommandResponseLocal::NodeDiscovery;
     */
}
#endif //XBEE_BACKEND_CPP_XBEEUTILITY_H
