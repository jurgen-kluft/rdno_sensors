#include "rdno_core/c_log.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_sensors/c_hsp24.h"

#ifdef TARGET_ARDUINO
#    include "Arduino.h"
#    include "Wire.h"
#endif

namespace ncore
{
    namespace nsensors
    {
        namespace nseeed
        {
#define RECEIVE_TIMEOUT              100
#define FRAME_START_SIZE             4
#define FRAME_END_SIZE               4
#define FRAME_INTRAFRAME_DATA_OFFSET 6

            struct FrameSequence_t
            {
                u8 const *mData;
                const s32 mLength;
            };

            static const byte FRAME_START[FRAME_START_SIZE] = {0xF4, 0xF3, 0xF2, 0xF1};
            static const byte FRAME_END[FRAME_END_SIZE]     = {0xF8, 0xF7, 0xF6, 0xF5};

            enum ECommand
            {
                ENABLE_CONFIGURATION          = 0x00FF,  // Enable configuration commands
                END_CONFIGURATION             = 0x00FE,  // End configuration command
                SET_MAX_DISTANCE_UNOCCUPIED   = 0x0060,  // Configure max distance gate & unoccupied duration
                READ_PARAMETERS               = 0x0061,  // Read radar configuration parameters
                ENABLE_ENGINEERING_MODE       = 0x0062,  // Enable engineering mode
                CLOSE_ENGINEERING_MODE        = 0x0063,  // Close engineering mode
                SET_DISTANCE_GATE_SENSITIVITY = 0x0064,  // Configure distance gate sensitivity
                READ_FIRMWARE_VERSION         = 0x00A0,  // Read firmware version
                SET_SERIAL_BAUD_RATE          = 0x00A1,  // Set serial port baud rate
                RESTORE_FACTORY_SETTINGS      = 0x00A2,  // Restore factory settings
                RESTART_MODULE                = 0x00A3,  // Restart module
                BLUETOOTH_SETTINGS            = 0x00A4,  // Bluetooth on/off
                GET_MAC_ADDRESS               = 0x00A5,  // Query MAC address
                OBTAIN_BLUETOOTH_PERMISSIONS  = 0x00A8,  // Obtain Bluetooth permissions
                SET_BLUETOOTH_PASSWORD        = 0x00A9,  // Set Bluetooth password
                SET_DISTANCE_RESOLUTION       = 0x00AA,  // Configure distance resolution
                QUERY_DISTANCE_RESOLUTION     = 0x00AB   // Query distance resolution
            };

            // ---------------------------------------------------------------------------------------------------------------------------------
            // Command encoding functions
            // ---------------------------------------------------------------------------------------------------------------------------------

            inline void WriteFrameStart(u8 *&outBuffer)
            {
                for (s32 i = 0; i < FRAME_START_SIZE; i++)
                    *outBuffer++ = FRAME_START[i];
            }

            inline void SkipIntraFrameLength(u8 *&outBuffer)
            {
                outBuffer += 2;  // Reserve 2 bytes for length
            }

            inline void WriteFrameEnd(u8 *&outBuffer)
            {
                for (s32 i = 0; i < FRAME_END_SIZE; i++)
                    *outBuffer++ = FRAME_END[i];
            }

            inline void WritePayloadLength(u8 *lengthPos, u16 length)
            {
                lengthPos[0] = (length >> 8) & 0xFF;
                lengthPos[1] = length & 0xFF;
            }

            inline void WriteCmd(u8 *&outBuffer, u16 cmd)
            {
                *outBuffer++ = (cmd >> 8) & 0xFF;
                *outBuffer++ = cmd & 0xFF;
            }

            inline void WriteUInt16LE(u8 *&outBuffer, u16 value)
            {
                *outBuffer++ = value & 0xFF;
                *outBuffer++ = (value >> 8) & 0xFF;
            }

            inline void WriteUInt32LE(u8 *&outBuffer, u32 value)
            {
                *outBuffer++ = value & 0xFF;
                *outBuffer++ = (value >> 8) & 0xFF;
                *outBuffer++ = (value >> 16) & 0xFF;
                *outBuffer++ = (value >> 24) & 0xFF;
            }

            s32 EncodeEnableConfiguration(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, ENABLE_CONFIGURATION);

                // Payload
                WriteUInt16LE(outBuffer, 0x0001);

                // Compute and write length
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));

                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            s32 EncodeEndConfiguration(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, END_CONFIGURATION);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            s32 EncodeSetMaxDistanceUnoccupied(u8 *outBuffer, u16 motionGateWord, u32 motionGateVal, u16 restGateWord, u32 restGateVal, u16 unoccupiedWord, u32 unoccupiedVal)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, SET_MAX_DISTANCE_UNOCCUPIED);

                // Payload, 3 * (word + dword) = 18 bytes
                WriteUInt16LE(outBuffer, motionGateWord);
                WriteUInt32LE(outBuffer, motionGateVal);
                WriteUInt16LE(outBuffer, restGateWord);
                WriteUInt32LE(outBuffer, restGateVal);
                WriteUInt16LE(outBuffer, unoccupiedWord);
                WriteUInt32LE(outBuffer, unoccupiedVal);

                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));

                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Read Parameters
            s32 EncodeReadParameters(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, READ_PARAMETERS);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Enable Engineering Mode
            s32 EncodeEnableEngineeringMode(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, ENABLE_ENGINEERING_MODE);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Close Engineering Mode
            s32 EncodeCloseEngineeringMode(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, CLOSE_ENGINEERING_MODE);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Set Distance Gate Sensitivity
            s32 EncodeSetDistanceGateSensitivity(u8 *outBuffer, u16 gateWord, u32 gateVal, u16 motionWord, u32 motionVal, u16 staticWord, u32 staticVal)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, SET_DISTANCE_GATE_SENSITIVITY);
                WriteUInt16LE(outBuffer, gateWord);
                WriteUInt32LE(outBuffer, gateVal);
                WriteUInt16LE(outBuffer, motionWord);
                WriteUInt32LE(outBuffer, motionVal);
                WriteUInt16LE(outBuffer, staticWord);
                WriteUInt32LE(outBuffer, staticVal);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Read Firmware Version
            s32 EncodeReadFirmwareVersion(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, READ_FIRMWARE_VERSION);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Set Serial Baud Rate
            s32 EncodeSetSerialBaudRate(u8 *outBuffer, u16 baudIndex)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, SET_SERIAL_BAUD_RATE);
                WriteUInt16LE(outBuffer, baudIndex);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Restore Factory Settings
            s32 EncodeRestoreFactorySettings(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, RESTORE_FACTORY_SETTINGS);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Restart Module
            s32 EncodeRestartModule(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, RESTART_MODULE);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Bluetooth Settings
            s32 EncodeBluetoothSettings(u8 *outBuffer, bool enable)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, BLUETOOTH_SETTINGS);
                WriteUInt16LE(outBuffer, enable ? 0x0100 : 0x0000);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Get MAC Address
            s32 EncodeGetMacAddress(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, GET_MAC_ADDRESS);
                WriteUInt16LE(outBuffer, 0x0001);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Obtain Bluetooth Permissions
            s32 EncodeObtainBluetoothPermissions(u8 *outBuffer, const char *password)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, OBTAIN_BLUETOOTH_PERMISSIONS);
                for (s32 i = 0; i < 6; i++)
                    WriteUInt16LE(outBuffer, (u16)password[i]);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Set Bluetooth Password
            s32 EncodeSetBluetoothPassword(u8 *outBuffer, const char *password)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, SET_BLUETOOTH_PASSWORD);
                for (s32 i = 0; i < 6; i++)
                    WriteUInt16LE(outBuffer, (u16)password[i]);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Set Distance Resolution
            s32 EncodeSetDistanceResolution(u8 *outBuffer, u16 resolutionIndex)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, SET_DISTANCE_RESOLUTION);
                WriteUInt16LE(outBuffer, resolutionIndex);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Query Distance Resolution
            s32 EncodeQueryDistanceResolution(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, QUERY_DISTANCE_RESOLUTION);
                WritePayloadLength((u8 *)(outStart + FRAME_START_SIZE), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // ---------------------------------------------------------------------------------------------------------------------------------
            // ACK functions
            // ---------------------------------------------------------------------------------------------------------------------------------
            // Universal ACK validator
            inline bool ValidateAck(u16 expectedCmd, const u8 *buffer, size_t length)
            {
                if (length < 12)
                    return false;  // Minimum ACK size
                if (!(buffer[0] == 0xFD && buffer[1] == 0xFC && buffer[2] == 0xFB && buffer[3] == 0xFA))
                    return false;
                u16 cmd = (buffer[6] << 8) | buffer[7];
                if (cmd != expectedCmd)
                    return false;
                u16 ackStatus = (buffer[8] << 8) | buffer[9];
                return ackStatus == 0x0001;  // Success
            }

            // Utility readers
            inline u16 ReadUInt16LE(const u8 *buffer) { return (u16)buffer[0] | ((u16)buffer[1] << 8); }
            inline u32 ReadUInt32LE(const u8 *buffer) { return (u32)buffer[0] | ((u32)buffer[1] << 8) | ((u32)buffer[2] << 16) | ((u32)buffer[3] << 24); }

            // Struct for ReadParameters
            struct RadarParameters
            {
                u8  maxDistanceGate;
                u8  motionGate;
                u8  restGate;
                u8  motionSensitivity[9];
                u8  stationarySensitivity[9];
                u16 unoccupiedDuration;
            };

            // Parse functions
            inline void ParseEnableConfigurationAck(const u8 *buffer, u16 &protocolVersion, u16 &bufferSize)
            {
                protocolVersion = ReadUInt16LE(&buffer[10]);
                bufferSize      = ReadUInt16LE(&buffer[12]);
            }

            inline void ParseReadParametersAck(const u8 *buffer, RadarParameters &params)
            {
                size_t idx = 10;  // After ACK status
                idx++;            // Skip header 0xAA
                params.maxDistanceGate = buffer[idx++];
                params.motionGate      = buffer[idx++];
                params.restGate        = buffer[idx++];
                for (int i = 0; i < 9; i++)
                    params.motionSensitivity[i] = buffer[idx++];
                for (int i = 0; i < 9; i++)
                    params.stationarySensitivity[i] = buffer[idx++];
                params.unoccupiedDuration = ReadUInt16LE(&buffer[idx]);
            }

            inline void ParseReadFirmwareVersionAck(const u8 *buffer, u16 &firmwareType, u16 &majorVersion, u32 &minorVersion)
            {
                firmwareType = ReadUInt16LE(&buffer[10]);
                majorVersion = ReadUInt16LE(&buffer[12]);
                minorVersion = ReadUInt32LE(&buffer[14]);
            }

            inline void ParseGetMacAddressAck(const u8 *buffer, u8 mac[6])
            {
                for (int i = 0; i < 6; i++)
                    mac[i] = buffer[10 + i];
            }

            inline void ParseQueryDistanceResolutionAck(const u8 *buffer, u16 &resolutionIndex) { resolutionIndex = ReadUInt16LE(&buffer[10]); }

            bool ParseRadarStatusAck(const u8 *msg, s32 msgLen, RadarStatus &status)
            {
                if (msg[7] != 0xAA)
                {
                    status.targetStatus = TargetStatusFrameError;
                    return false;
                }

                status.radarMode    = (ERadarMode)msg[6];
                status.targetStatus = (ETargetStatus)msg[8];

                if (status.targetStatus != TargetStatusFrameError)
                {
                    status.detectionDistance = (s32)ReadUInt16LE(&msg[15]);
                }
                else
                {
                    status.detectionDistance = -1;
                }

                if (status.radarMode == RadarMode_Engineering)
                {
                    // Parse engineering mode data
                    s32 offset                         = 17;
                    status.maximumMovementDistanceDoor = (s32)msg[offset++];
                    status.maximumRestingDistanceDoor  = (s32)msg[offset++];
                    for (s32 i = 0; i < 9; i++)
                    {
                        status.movementDistanceGateEnergy.gate[i] = (s32)msg[offset++];
                    }
                    for (s32 i = 0; i < 9; i++)
                    {
                        status.stationaryDistanceGateEnergy.gate[i] = (s32)msg[offset++];
                    }
                    status.photosensitive = (s32)msg[offset];
                }
                else
                {
                    // Basic mode, set defaults
                    status.maximumMovementDistanceDoor = 0xFF;
                    status.maximumRestingDistanceDoor  = 0xFF;
                    status.movementDistanceGateEnergy.reset();
                    status.stationaryDistanceGateEnergy.reset();
                    status.photosensitive = 0xFF;
                }
            }

            // ---------------------------------------------------------------------------------------------------------------------------------
            // ---------------------------------------------------------------------------------------------------------------------------------

            void MovementEnergy::reset()
            {
                for (s32 i = 0; i < 9; i++)
                    gate[i] = 0xFF;
            }
            void StationaryEnergy::reset()
            {
                for (s32 i = 0; i < 9; i++)
                    gate[i] = 0xFF;
            }

            void InitRadarStatus(RadarStatus &status)
            {
                status.targetStatus = TargetStatusFrameError;
                status.radarMode    = RadarMode_Normal;

                status.movementTargetDistance   = 0xFFFF;
                status.movementTargetEnergy     = 0xFF;
                status.stationaryTargetDistance = 0xFFFF;
                status.stationaryTargetEnergy   = 0xFF;
                status.detectionDistance        = 0xFFFF;

                // u8               maximumMovementDistanceDoor;
                // u8               maximumRestingDistanceDoor;
                // MovementEnergy   movementDistanceGateEnergy;    // Movement energy value for each distance gate
                // StationaryEnergy stationaryDistanceGateEnergy;  // Stationary energy value for each distance gate
                // u8               photosensitive;                // Photosensitive value 0-255
                status.maximumMovementDistanceDoor = 0xFF;
                status.maximumRestingDistanceDoor  = 0xFF;
                status.movementDistanceGateEnergy.reset();
                status.stationaryDistanceGateEnergy.reset();
                status.photosensitive = 0xFF;
            }

            void begin(hsp24_t &sensor, Stream *serial, Stream *debugSerial)
            {
                sensor._serial           = serial;
                sensor._debugSerial      = debugSerial;
                sensor.receiveStartTime  = 0;
                sensor.bufferIndex       = 0;
                sensor.isInATMode        = 0;
                sensor.bufferIndex_hsp24 = 0;
            }

            bool getStatus(hsp24_t &sensor, RadarStatus &status)
            {
                InitRadarStatus(status);

                // Implementation to read and parse radar status goes here

                return false;
            }

        }  // namespace nseeed
    }  // namespace nsensors
}  // namespace ncore