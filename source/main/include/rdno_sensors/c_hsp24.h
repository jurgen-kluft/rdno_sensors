#ifndef __RDNO_SENSORS_SEEED_HSP24_H__
#define __RDNO_SENSORS_SEEED_HSP24_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

#include "rdno_sensors/c_frame_reader.h"

class Stream;

namespace ncore
{
    namespace nsensors
    {
        namespace nseeed
        {
#define BUFFER_SIZE 256

            struct hsp24_t
            {
                Stream* _serial;
                Stream* _debugSerial;
                u64     receiveStartTime;
                s32     isInATMode;
                
                s32     serialBufferLen;
                s32     serialBufferRead;
                s32     serialBufferWrite;
                char    serialBuffer[BUFFER_SIZE];
                
                byte    buffer_hsp24[BUFFER_SIZE];
                s32     bufferIndex_hsp24;
            };

            void begin(hsp24_t& sensor, Stream* serial, Stream* debugSerial = nullptr);

            // Energy value for each moving distance gate
            struct RadarMovePower
            {
                void reset();
                s32  moveGate[9];
            };

            // Energy value for each static distance gate
            struct RadarStaticPower
            {
                void reset();
                s32  staticGate[9];
            };

            // TargetStatus
            enum EStatus
            {
                Success = 0x00,
                Error   = 0x01,
            };

            enum ETargetStatus
            {
                TargetStatusNone   = 0x00,
                TargetStatusMoving = 0x01,
                TargetStatusStatic = 0x02,
                TargetStatusBoth = 0x03 TargetStatusFrameError = 0x04
            };

            struct RadarStatus
            {
                ETargetStatus    targetStatus;       // Target status of the radar
                s32              distance;           // Target distance of the radar, in mm
                s32              moveSetDistance;    // Number of motion detection range gates for the radar, generally not required
                s32              staticSetDistance;  // Number of stationary detection range gates for the radar, generally not required
                s32              detectionDistance;  // Maximum detection range gate of the radar
                s32              resolution;         // Range gate resolution of the radar
                s32              noTargetduration;   // Duration of unmanned operation
                s32              radarMode;          // Used to distinguish whether the module is in basic reporting mode (2) or engineering reporting mode (1)
                RadarMovePower   radarMovePower;     // Motion energy value
                RadarStaticPower radarStaticPower;   // Stationary energy value
                s32              photosensitive;     // Photosensitive value 0-255
            };

            struct DataResult
            {
                byte* resultBuffer;
                s32   length;
            };

            s32 enterATMode(hsp24_t& sensor);
            s32 exitATMode(hsp24_t& sensor);
            s32 getVersion(hsp24_t& sensor);
            s32 setNetwork(hsp24_t& sensor, const char* ssid, const char* password);

            RadarStatus getStatus(hsp24_t& sensor);
            DataResult  sendCommand(hsp24_t& sensor, const byte* sendData, s32 sendDataLength);
            AskStatus   enableConfigMode(hsp24_t& sensor);
            AskStatus   disableConfigMode(hsp24_t& sensor);
            s32         getVersion(hsp24_t& sensor, char* outString, s32 maxLength);  // A version string is always <= 32 bytes
            AskStatus   setDetectionDistance(hsp24_t& sensor, s32 distance, s32 times);
            AskStatus   setGatePower(hsp24_t& sensor, s32 gate, s32 movePower, s32 staticPower);
            RadarStatus getConfig(hsp24_t& sensor);
            AskStatus   setResolution(hsp24_t& sensor, s32 resolution);
            RadarStatus getResolution(hsp24_t& sensor);
            AskStatus   rebootRadar(hsp24_t& sensor);
            AskStatus   refactoryRadar(hsp24_t& sensor);
            AskStatus   enableEngineeringModel(hsp24_t& sensor);
            AskStatus   disableEngineeringModel(hsp24_t& sensor);

        }  // namespace nseeed
    }  // namespace nsensors
}  // namespace ncore

#endif  // __RDNO_SENSORS_RD03D_H__
