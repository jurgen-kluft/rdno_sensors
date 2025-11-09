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
                uint8_t const *mData;
                const s32      mLength;
            };

            static const byte FRAME_START[FRAME_START_SIZE]    = {0xF4, 0xF3, 0xF2, 0xF1};
            static const byte FRAME_END[FRAME_END_SIZE]        = {0xF8, 0xF7, 0xF6, 0xF5};
            static const byte FRAME_ASKSTART[FRAME_START_SIZE] = {0xFD, 0xFC, 0xFB, 0xFA};
            static const byte FRAME_ASKEND[FRAME_END_SIZE]     = {0x04, 0x03, 0x02, 0x01};

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

            inline void WriteFrameStart(uint8_t *&outBuffer)
            {
                for (s32 i = 0; i < FRAME_START_SIZE; i++)
                    *outBuffer++ = FRAME_START[i];
            }

            inline void SkipIntraFrameLength(uint8_t *&outBuffer)
            {
                outBuffer += 2;  // Reserve 2 bytes for length
            }

            inline void WriteFrameEnd(uint8_t *&outBuffer)
            {
                for (s32 i = 0; i < FRAME_END_SIZE; i++)
                    *outBuffer++ = FRAME_END[i];
            }

            inline void WritePayloadLength(uint8_t *lengthPos, uint16_t length)
            {
                lengthPos[0] = (length >> 8) & 0xFF;
                lengthPos[1] = length & 0xFF;
            }

            inline void WriteCmd(uint8_t *&outBuffer, uint16_t cmd)
            {
                *outBuffer++ = (cmd >> 8) & 0xFF;
                *outBuffer++ = cmd & 0xFF;
            }

            inline void WriteUInt16LE(uint8_t *&outBuffer, uint16_t value)
            {
                *outBuffer++ = value & 0xFF;
                *outBuffer++ = (value >> 8) & 0xFF;
            }

            inline void WriteUInt32LE(uint8_t *&outBuffer, uint32_t value)
            {
                *outBuffer++ = value & 0xFF;
                *outBuffer++ = (value >> 8) & 0xFF;
                *outBuffer++ = (value >> 16) & 0xFF;
                *outBuffer++ = (value >> 24) & 0xFF;
            }

            s32 EncodeEnableConfiguration(uint8_t *outBuffer)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, ENABLE_CONFIGURATION);

                // Payload
                WriteUInt16LE(outBuffer, 0x0001);

                // Compute and write length
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));

                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            s32 EncodeEndConfiguration(uint8_t *outBuffer)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, END_CONFIGURATION);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            s32 EncodeSetMaxDistanceUnoccupied(uint8_t *outBuffer, uint16_t motionGateWord, uint32_t motionGateVal, uint16_t restGateWord, uint32_t restGateVal, uint16_t unoccupiedWord, uint32_t unoccupiedVal)
            {
                uint8_t const *const outStart = outBuffer;
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

                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));

                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Read Parameters
            s32 EncodeReadParameters(uint8_t *outBuffer)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, READ_PARAMETERS);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Enable Engineering Mode
            s32 EncodeEnableEngineeringMode(uint8_t *outBuffer)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, ENABLE_ENGINEERING_MODE);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Close Engineering Mode
            s32 EncodeCloseEngineeringMode(uint8_t *outBuffer)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, CLOSE_ENGINEERING_MODE);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Set Distance Gate Sensitivity
            s32 EncodeSetDistanceGateSensitivity(uint8_t *outBuffer, uint16_t gateWord, uint32_t gateVal, uint16_t motionWord, uint32_t motionVal, uint16_t staticWord, uint32_t staticVal)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, SET_DISTANCE_GATE_SENSITIVITY);
                WriteUInt16LE(outBuffer, gateWord);
                WriteUInt32LE(outBuffer, gateVal);
                WriteUInt16LE(outBuffer, motionWord);
                WriteUInt32LE(outBuffer, motionVal);
                WriteUInt16LE(outBuffer, staticWord);
                WriteUInt32LE(outBuffer, staticVal);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Read Firmware Version
            s32 EncodeReadFirmwareVersion(uint8_t *outBuffer)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, READ_FIRMWARE_VERSION);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Set Serial Baud Rate
            s32 EncodeSetSerialBaudRate(uint8_t *outBuffer, uint16_t baudIndex)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, SET_SERIAL_BAUD_RATE);
                WriteUInt16LE(outBuffer, baudIndex);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Restore Factory Settings
            s32 EncodeRestoreFactorySettings(uint8_t *outBuffer)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, RESTORE_FACTORY_SETTINGS);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Restart Module
            s32 EncodeRestartModule(uint8_t *outBuffer)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, RESTART_MODULE);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Bluetooth Settings
            s32 EncodeBluetoothSettings(uint8_t *outBuffer, bool enable)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, BLUETOOTH_SETTINGS);
                WriteUInt16LE(outBuffer, enable ? 0x0100 : 0x0000);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Get MAC Address
            s32 EncodeGetMacAddress(uint8_t *outBuffer)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, GET_MAC_ADDRESS);
                WriteUInt16LE(outBuffer, 0x0001);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Obtain Bluetooth Permissions
            s32 EncodeObtainBluetoothPermissions(uint8_t *outBuffer, const char *password)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, OBTAIN_BLUETOOTH_PERMISSIONS);
                for (s32 i = 0; i < 6; i++)
                    WriteUInt16LE(outBuffer, (uint16_t)password[i]);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Set Bluetooth Password
            s32 EncodeSetBluetoothPassword(uint8_t *outBuffer, const char *password)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, SET_BLUETOOTH_PASSWORD);
                for (s32 i = 0; i < 6; i++)
                    WriteUInt16LE(outBuffer, (uint16_t)password[i]);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Set Distance Resolution
            s32 EncodeSetDistanceResolution(uint8_t *outBuffer, uint16_t resolutionIndex)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, SET_DISTANCE_RESOLUTION);
                WriteUInt16LE(outBuffer, resolutionIndex);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // Query Distance Resolution
            s32 EncodeQueryDistanceResolution(uint8_t *outBuffer)
            {
                uint8_t const *const outStart = outBuffer;
                WriteFrameStart(outBuffer);
                SkipIntraFrameLength(outBuffer);
                WriteCmd(outBuffer, QUERY_DISTANCE_RESOLUTION);
                WritePayloadLength((uint8_t *)(outStart + FRAME_START_SIZE), (uint16_t)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                WriteFrameEnd(outBuffer);
                return (outBuffer - outStart);
            }

            // ---------------------------------------------------------------------------------------------------------------------------------
            // ACK functions
            // ---------------------------------------------------------------------------------------------------------------------------------
            // Universal ACK validator
            inline bool ValidateAck(uint16_t expectedCmd, const uint8_t *buffer, size_t length)
            {
                if (length < 12)
                    return false;  // Minimum ACK size
                if (!(buffer[0] == 0xFD && buffer[1] == 0xFC && buffer[2] == 0xFB && buffer[3] == 0xFA))
                    return false;
                uint16_t cmd = (buffer[6] << 8) | buffer[7];
                if (cmd != expectedCmd)
                    return false;
                uint16_t ackStatus = (buffer[8] << 8) | buffer[9];
                return ackStatus == 0x0001;  // Success
            }

            // Utility readers
            inline uint16_t ReadUInt16LE(const uint8_t *buffer) { return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8); }

            inline uint32_t ReadUInt32LE(const uint8_t *buffer) { return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24); }

            // Struct for ReadParameters
            struct RadarParameters
            {
                uint8_t  maxDistanceGate;
                uint8_t  motionGate;
                uint8_t  restGate;
                uint8_t  motionSensitivity[9];
                uint8_t  stationarySensitivity[9];
                uint16_t unoccupiedDuration;
            };

            // Parse functions
            inline void ParseEnableConfigurationAck(const uint8_t *buffer, uint16_t &protocolVersion, uint16_t &bufferSize)
            {
                protocolVersion = ReadUInt16LE(&buffer[10]);
                bufferSize      = ReadUInt16LE(&buffer[12]);
            }

            inline void ParseReadParametersAck(const uint8_t *buffer, RadarParameters &params)
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

            inline void ParseReadFirmwareVersionAck(const uint8_t *buffer, uint16_t &firmwareType, uint16_t &majorVersion, uint32_t &minorVersion)
            {
                firmwareType = ReadUInt16LE(&buffer[10]);
                majorVersion = ReadUInt16LE(&buffer[12]);
                minorVersion = ReadUInt32LE(&buffer[14]);
            }

            inline void ParseGetMacAddressAck(const uint8_t *buffer, uint8_t mac[6])
            {
                for (int i = 0; i < 6; i++)
                    mac[i] = buffer[10 + i];
            }

            inline void ParseQueryDistanceResolutionAck(const uint8_t *buffer, uint16_t &resolutionIndex) { resolutionIndex = ReadUInt16LE(&buffer[10]); }

            // ---------------------------------------------------------------------------------------------------------------------------------
            // ---------------------------------------------------------------------------------------------------------------------------------

            void RadarMovePower::reset()
            {
                for (s32 i = 0; i < 9; i++)
                    moveGate[i] = -1;
            }
            void RadarStaticPower::reset()
            {
                for (s32 i = 0; i < 9; i++)
                    staticGate[i] = -1;
            }

            void InitRadarStatus(RadarStatus &status)
            {
                status.targetStatus      = TargetStatusFrameError;
                status.distance          = -1;
                status.moveSetDistance   = -1;
                status.staticSetDistance = -1;
                status.detectionDistance = -1;
                status.resolution        = -1;
                status.noTargrtduration  = -1;
                status.radarMode         = -1;
                status.radarMovePower.reset();
                status.radarStaticPower.reset();
                status.photosensitive = -1;
            }

            s32 findSequence(byte *arr, s32 arrLen, const byte *seq, s32 seqLen)
            {
                for (s32 i = 0; i < arrLen - seqLen + 1; i++)
                {
                    s32 j = 0;
                    while (j < seqLen)
                    {
                        if (arr[i + j] != seq[j])
                            goto sequence_not_found;
                        j++;
                    }
                    return i;
                sequence_not_found:
                }
                return -1;
            }

            s32 checkBuffer(hsp24_t &sensor)
            {
                u64 startTime = millis();  // 记录开始时间
                while (true)
                {
                    // 检查是否有数据可用于接收
                    while (_serial->available())
                    {
                        char receivedChar   = _serial->read();  //   接收串口数据
                        buffer[bufferIndex] = receivedChar;     //   接收到的数据都存入缓冲区里
                        bufferIndex++;

                        // 更新接收开始时间
                        receiveStartTime = millis();
                    }

                    // 检查是否达到发送条件
                    if (bufferIndex > 0 && (bufferIndex >= BUFFER_SIZE || millis() - receiveStartTime >= RECEIVE_TIMEOUT))
                    {
                        // 检查缓冲区中是否包含 "OK"
                        char *found = strstr(this->buffer, "ok");
                        if (found != NULL)
                        {
                            if (_debugSerial != nullptr && _debugSerial->available() > 0)
                            {
                                _debugSerial->println("Setting Success!");
                            }

                            // 清空缓冲区和索引
                            memset(this->buffer, 0, BUFFER_SIZE);
                            this->bufferIndex = 0;

                            return 1;
                        }
                        else
                        {
                            if (_debugSerial != nullptr && _debugSerial->available() > 0)
                            {
                                // 发送缓冲区中的数据到另一个串口
                                for (s32 i = 0; i < this->bufferIndex; i++)
                                {
                                    _debugSerial->print(buffer[i]);
                                }
                            }

                            // 清空缓冲区和索引
                            memset(this->buffer, 0, BUFFER_SIZE);
                            this->bufferIndex = 0;
                            return 0;
                        }
                    }

                    if (millis() - startTime > 3000)  // 超过3秒等待时间
                    {
                        break;
                    }
                }

                return 0;  // 超时，未成功进入AT模式
            }

            s32 sendATCommand(hsp24_t &sensor, const char *command)
            {
                sensor._serial->println(command);
                s32 ret = checkBuffer(sensor);
                return ret;
            }

            s32 sendATCommandWithExit(hsp24_t &sensor, const char *command)
            {
                sensor._serial->println(command);
                s32 ret = checkBuffer(sensor);
                exitATMode(sensor);
                return ret;
            }

            void begin(hsp24_t &sensor, Stream *serial, Stream *debugSerial = nullptr)
            {
                sensor._serial           = serial;
                sensor._debugSerial      = debugSerial;
                sensor.receiveStartTime  = 0;
                sensor.bufferIndex       = 0;
                sensor.isInATMode        = 0;
                sensor.bufferIndex_hsp24 = 0;
            }

        }  // namespace nseeed
    }  // namespace nsensors
}  // namespace ncore