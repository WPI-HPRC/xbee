//
// Created by William Scheirey on 3/15/24.
//

#ifndef HPRC_XBEEUTILITY_H
#define HPRC_XBEEUTILITY_H

#include <cstdint>
#include <cstddef>

#define AsciiToUint16(x, y)((uint16_t)x << 8 | y)

/*
 * XBee namespace with constants for utility.
 *
 * NOTE: PacketBytes and FrameBytes fields represent the minimum number of bytes for their respective fields
 *
 * Info from:
 * https://www.digi.com/resources/documentation/digidocs/pdfs/90001477.pdf
 * https://www.digi.com/resources/documentation/Digidocs/90002002/Reference/r_cmd_ND.htm
 *
 */

namespace XBee
{
    const uint8_t StartDelimiter = 0x7E;
    const uint8_t EscapeCharacter = 0x7D;
    const uint8_t EscapeXorByte = 0x20;

    struct RemoteDevice
    {
        uint64_t serialNumber;
        uint8_t id[20]; //  (variable, 0-20 bytes)
        uint8_t idLength;
        uint8_t deviceType;
        uint8_t status;
        uint16_t profileID;
        uint16_t manufacturerID;

        // These last two fields depend on 'NO' settings; they may not be included in the packet
        uint32_t digiDeviceType;
        uint8_t lastHopRssi;
    };

    namespace ApiOptions
    {
        enum ApiOptions
        {
            TransparentMode = 0x00,
            ApiWithoutEscapes = 0x01,
            ApiWithEscapes = 0x02
        };
    }

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
            TransmitStatus = 0x89,
            ExtendedTransmitStatus = 0x8B,
            ReceivePacket = 0x90,
            ReceivePacket64Bit = 0x80,
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
            ApplyChanges = AsciiToUint16('A', 'C'),
            SerialNumberHigh = AsciiToUint16('S', 'H'),
            SerialNumberLow = AsciiToUint16('S', 'L'),
            NodeIdentifier = AsciiToUint16('N', 'I'),
            PowerLevel = AsciiToUint16('P', 'L'),
            PowerMode = AsciiToUint16('P', 'M'),
            PeakPower = AsciiToUint16('P', 'P'),
            SupplyVoltage = AsciiToUint16('%', 'V'), // Read user manual for scaling
            VoltageSupplyMonitoring = AsciiToUint16('V', '+'),
            Temperature = AsciiToUint16('T', 'P'),
            FirmwareVersion = AsciiToUint16('V', 'R'),
            HardwareVersion = AsciiToUint16('H', 'V'),
            AvailableFrequencies = AsciiToUint16('A', 'F'),
            ChannelMask = AsciiToUint16('C', 'M'),
            MinimumFrequencies = AsciiToUint16('M', 'F'),
            RFDataRate = AsciiToUint16('B', 'R'),
            EnergyDetect = AsciiToUint16('E', 'D'),
            UnicastAttemptedCount = AsciiToUint16('U', 'A'),
            MacAckFailureCount = AsciiToUint16('E', 'A'),
            TransmissionFailureCount = AsciiToUint16('T', 'R'),
            ApiOptions = AsciiToUint16('A', 'O')
        };
        enum CommandStatus
        {
            Ok = 0,
            Error = 1,
            InvalidCommand = 2,
            InvalidParameter = 3
        };
    }

    namespace RemoteAtCommand
    {
        enum CommandStatus
        {
            Ok = 0,
            Error = 1,
            InvalidCommand = 2,
            InvalidParameter = 3,
            TransmissionFailure = 4
        };
    }

    const uint8_t MaxPacketBytes = 255;
    const uint8_t FrameBytes = 4; // +1 for start delimiter, +2 for length, +1 for checksum
    const uint16_t MaxFrameBytes = MaxPacketBytes + FrameBytes;

#pragma pack(push, 1)
    struct BasicFrame
    {
        size_t length_bytes;
        uint8_t frame[MaxPacketBytes];
    };
#pragma pack(pop)

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

    namespace RemoteAtCommandTransmit
    {
        const uint8_t PacketBytes = 15; // +1 for frame type, +1 for frame ID, +8 for dest address, +2 for reserved, +1 for remote options, +2 for AT command
        const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
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

    namespace ReceivePacket64Bit
    {
        const uint8_t PacketBytes = 11; // +1 for frame type, +8 for sender address, +1 for rssi, +1 for receive options
        const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
        const uint8_t BytesBeforePacket = 4;
        const uint8_t BytesBeforeAddress = 4;
        const uint8_t BytesBeforeRssi = 12;
        const uint8_t BytesBeforePayload = 14;

        struct Struct
        {
            uint8_t dataLength_bytes;
            uint8_t negativeRssi;
            uint64_t senderAddress;
            const uint8_t *data;
        };
    }

    namespace ExplicitRxIndicator
    {/*
     * +1 for frame type, +8 for sender address, +2 for reserved, +1 for source endpoint, +1 for destination endpoint,
     * +2 for cluster ID, +2 for profile ID, +1 for receive options,
     *
     */
        const uint8_t PacketBytes = 18;
        const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
        const uint8_t BytesBeforePacket = 4;
        const uint8_t BytesBeforeAddress = 4;
        const uint8_t BytesBeforePayload = 21;

        struct Struct
        {
            uint8_t dataLength_bytes;
            uint8_t negativeRssi;
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

    namespace RemoteAtCommandResponse
    {
        const uint8_t PacketBytes = 15; // +1 for frame type, +1 for frame ID, +8 for dest address, +2 for reserved, +2 for AT command, + 1 for command status
        const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
        const uint8_t BytesBeforeAddress = 5;
        const uint8_t BytesBeforeCommand = 15;
        const uint8_t BytesBeforeCommandStatus = 17;
        const uint8_t BytesBeforeCommandData = 18;

        namespace NodeDiscovery
        {
            const uint8_t PacketBytes = AtCommandResponse::PacketBytes +
                                        14; // +4 for SH, +4 for SL, +1 for Device Type, +1 for status, +2 for profile ID, +2 for manufacturer ID
            const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
            const uint8_t MaxPacketBytes = PacketBytes +
                                           26; // +20 for NI, +1 for NI null byte, +4 for Digi Device Type, +1 for RSSI of last hop
        }
    }

    namespace TransmitStatus
    {
        const uint8_t PacketBytes = 3;// +1 for frame type, +1 for frameID, +1 for delivery status
        const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;
        const uint8_t BytesBeforeFrameID = 4;
        const uint8_t BytesBeforeStatus = 5;

        struct Struct
        {
            uint8_t frameID;
            uint8_t deliveryStatus;
        };

        enum StatusCode
        {
            Success = 0x00,
            NoAckReceived = 0x01,
            CcaFailure = 0x02,
            IndirectMessageUnrequested = 0x03,
            TransceiverUnableToCompleteTransmission = 0x04,
            NetworkAckFailure = 0x21,
            NotJoinedToNetwork = 0x22,
            InvalidFrameValues = 0x2C,
            InternalError = 0x31,
            ResourceError = 0x32,
            NoSecureSessionConnection = 0x34,
            EncryptionFailure = 0x35,
            MessageTooLong = 0x74,
            SocketClosedUnexpectedly = 0x76,
            InvalidUdpPort = 0x78,
            InvalidTcpPort = 0x79,
            InvalidHostAddress = 0x7A,
            InvalidDataMode = 0x7B,
            InvalidInterface = 0x7C,
            InterfaceNotAcceptingFrames = 0x7D,
            ModemUpdateInProgress = 0x7E,
            ConnectionRefused = 0x80,
            SocketConnectionLost = 0x81,
            NoServer = 0x82,
            SocketClosed = 0x83,
            UnknownServer = 0x84,
            UnknownError = 0x85,
            InvalidTlsConfiguration = 0x86,
            SocketNotConnected = 0x87,
            SocketNotBound = 0x88
        };
    }

    namespace ExtendedTransmitStatus
    {
        const uint8_t PacketBytes = 6; // +1 for frame type, +1 for frame ID, +1 for reserved, +1 for transmit retry count, +1 for delivery status, +1 for discovery
        const uint8_t FrameBytes = XBee::FrameBytes + PacketBytes;

        const uint8_t BytesBeforeFrameID = 4;
        const uint8_t BytesBeforeRetryCount = 7;
        const uint8_t BytesBeforeStatus = 8;
        const uint8_t BytesBeforeDiscovery = 9;

#pragma pack(push, 1)
        struct Struct
        {
            uint8_t frameID;
            uint16_t reserved;
            uint8_t retryCount;
            uint8_t deliveryStatus;
            uint8_t discovery;
        };
#pragma pack(pop)

        enum StatusCode
        {
            Success = 0x00,
            MacAckFailure = 0x01,
            CcaLbtFailure = 0x02,
            IndirectMessageUnrequestedNoSpectrum = 0x03,
            NetworkAckFailure = 0x21,
            RouteNotFound = 0x25,
            InternalResourceError = 0x31,
            ResourceError = 0x32,
            DataPayloadTooLarge = 0x74,
            IndirectMessageUnrequested = 0x75
        };

    }

    // --- For Convenience ---
    /*
    namespace AtCommandResponseLocal = XBee::AtCommandResponse;
    namespace NodeDiscoveryTransmit = XBee::AtCommandTransmit::NodeDiscovery;
    namespace NodeDiscoveryResponseLocal = AtCommandResponseLocal::NodeDiscovery;
     */
}
#endif //HPRC_XBEEUTILITY_H
