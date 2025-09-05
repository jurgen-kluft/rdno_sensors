#    include "rdno_core/c_memory.h"
#    include "rdno_core/c_serial.h"
#    include "rdno_core/c_str.h"

#    include "rdno_sensors/c_hmmd.h"

namespace ncore
{
    namespace nsensors
    {
        bool initHMMD(u8 rxPin, u8 txPin)
        {
            nserial2::begin(nbaud::Rate115200, nconfig::MODE_8N1, rxPin, txPin);

            // // Put the sensor into 'normal' mode and keep it active
            // const byte command_normal_mode[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
            // nserial2::write(command_normal_mode, sizeof(command_normal_mode));

            // Put the sensor into 'report' mode and keep it active
            const byte command_report_mode[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
            nserial2::write(command_report_mode, sizeof(command_report_mode));

            return true;
        }

        // Report Mode
        //   Byte 1~4       Byte5,6   Byte 7,8    Byte9~10    Byte 11~14    Byte 15~18
        //   FD FC FB FA    08 00     12 00       00 00       04 00 00 00   04 03 02 01
        //
        // In report mode serial port data frame example:
        //
        // Frame Header    Length   Detection Result   Target Distance   The energy values for each distance gate    Frame Tail
        // 4 bytes,        2 bytes, 1 byte,            2 bytes,          32 bytes,                                   4 bytes
        //
        //   Frame Header (F4 F3 F2 F1)
        //   Detection Result (00 absent, 01 present)
        //   Length, total number of bytes for detection result, target distance, and energy values for each distance gate
        //   Target Distance, indicating the distance of the target phase from the radar in the scene
        //   Energy values, 16 (total number of distance gates) * 2 bytes, size of energy value for each distance gate from 0 to 15
        //   Frame Tail (F8 F7 F6 F5)
        // Total data frame length = 4 + 2 + 1 + 2 + 32 + 4 = 45 bytes

        static inline bool matchesFrameHeader(const byte* buffer, s32 pos, s32 bufferSize)
        {
            if (buffer[pos] == 0xF4)
            {
                pos = (pos + 1) & (bufferSize - 1);
                if (buffer[pos] == 0xF3)
                {
                    pos = (pos + 1) & (bufferSize - 1);
                    if (buffer[pos] == 0xF2)
                    {
                        pos = (pos + 1) & (bufferSize - 1);
                        return (buffer[pos] == 0xF1);
                    }
                }
            }
            return false;
        }

        static inline bool matchesFrameTail(const byte* buffer, s32 readPos, s32 frameSize, s32 bufferSize)
        {
            readPos = (readPos + frameSize - 4) & (bufferSize - 1);
            if (buffer[readPos] == 0xF8)
            {
                readPos = (readPos + 1) & (bufferSize - 1);
                if (buffer[readPos] == 0xF7)
                {
                    readPos = (readPos + 1) & (bufferSize - 1);
                    if (buffer[readPos] == 0xF6)
                    {
                        readPos = (readPos + 1) & (bufferSize - 1);
                        return (buffer[readPos] == 0xF5);
                    }
                }
            }
            return false;
        }

#    define hmmdRxBufferSize 256
#    define hmmdFrameLength  45

        // Circular 'moving window' buffer for receiving data from the HMMD sensor
        byte hmmdRxBuffer[hmmdRxBufferSize];
        s32  hmmdRxBufferReadPos  = 0;
        s32  hmmdRxBufferWritePos = 0;

        bool readHMMD2(u8* outDetection, u16* outDistanceInCm)
        {
            // Drain data from Serial2
            s32 available = nserial2::available();
            while (available > 0)
            {
                // Calculate the current length of read data in the buffer, taking into account wrap-around of the circular buffer
                s32 currentFrameLength = (hmmdRxBufferWritePos + hmmdRxBufferSize - hmmdRxBufferReadPos) & (hmmdRxBufferSize - 1);

                // Read available bytes into the buffer
                while (available > 0 && currentFrameLength < hmmdRxBufferSize)
                {
                    // Determine how many bytes we can read without overflowing the circular buffer
                    const s32 numBytesCanRead = hmmdRxBufferWritePos >= hmmdRxBufferReadPos ? hmmdRxBufferSize - hmmdRxBufferWritePos : hmmdRxBufferReadPos - hmmdRxBufferWritePos;
                    const s32 numBytesRead    = static_cast<s32>(nserial2::read_bytes(&hmmdRxBuffer[hmmdRxBufferWritePos], numBytesCanRead <= available ? numBytesCanRead : available));
                    if (numBytesRead == 0)
                    {
                        available = -1;  // Error condition, exit the loop
                        break;
                    }
                    hmmdRxBufferWritePos = (hmmdRxBufferWritePos + numBytesRead) & (hmmdRxBufferSize - 1);
                    currentFrameLength   = currentFrameLength + numBytesRead;
                    available            = available - numBytesRead;
                }

                // When the above read loop exits and available == -1, it indicates an error condition.
                // In that case, we skip the processing of the buffer and will loop back to the top to try reading again.
                if (available >= 0)
                {
                    // Advance the read position until we find a valid frame header
                    bool foundHeader = false;
                    while (currentFrameLength >= hmmdFrameLength)
                    {
                        if (matchesFrameHeader(hmmdRxBuffer, hmmdRxBufferReadPos, hmmdRxBufferSize))
                        {
                            foundHeader = true;  // Found a valid frame header
                            break;
                        }
                        hmmdRxBufferReadPos = (hmmdRxBufferReadPos + 1) & (hmmdRxBufferSize - 1);  // Move read position forward by one byte
                        currentFrameLength--;                                                      // Decrease the current frame length
                    }

                    if (foundHeader && currentFrameLength >= hmmdFrameLength)
                    {
                        // We know the length of a complete frame, so verify the frame tail
                        if (!matchesFrameTail(hmmdRxBuffer, hmmdRxBufferReadPos, hmmdFrameLength, hmmdRxBufferSize))
                        {
                            // We have a valid frame header but invalid frame tail?
                            // Move read position forward by one byte
                            hmmdRxBufferReadPos = (hmmdRxBufferReadPos + 1) & (hmmdRxBufferSize - 1);
                            continue;
                        }

                        // Extract the detection result (byte 6)
                        const s32 detectionPos = (hmmdRxBufferReadPos + 6) & (hmmdRxBufferSize - 1);
                        *outDetection          = static_cast<u8>(hmmdRxBuffer[detectionPos]);
                        // Extract the target distance (bytes 7 and 8)
                        const s32 distancePosL = (hmmdRxBufferReadPos + 7) & (hmmdRxBufferSize - 1);
                        const s32 distancePosH = (hmmdRxBufferReadPos + 8) & (hmmdRxBufferSize - 1);
                        *outDistanceInCm       = static_cast<u16>(hmmdRxBuffer[distancePosL]) | (static_cast<u16>(hmmdRxBuffer[distancePosH]) << 8);

                        // Move read position to the next frame
                        hmmdRxBufferReadPos += hmmdFrameLength;

                        return true;
                    }
                }

                available = nserial2::available();
            }
            return false;
        }

        bool readHMMD(u8* outDetection, u16* outDistanceInCm)
        {
            char line[128];
            g_memset(line, 0, sizeof(line));

            // Drain data from Serial2
            s32 available = nserial2::available();
            while (available > 0)
            {
                // Read a line of text until a newline character (\n) is received
                s32 lineLength = nserial2::read_until('\n', line, 128 - 1);
                if (lineLength == 0)
                {
                    break;  // No more data available
                }

                // Clean up the line: remove potential carriage return (\r) and leading/trailing whitespace
                char* begin = line;
                char* end   = line + lineLength;
                while (end > begin && (*(end - 1) == '\r' || *(end - 1) == '\n' || *(end - 1) == ' '))
                {
                    end = end - 1;
                }
                while (begin < end && (*begin == ' ' || *begin == '\r' || *begin == '\n'))
                {
                    begin = begin + 1;
                }

                // Check if the line contains the "Range" information
                lineLength = static_cast<s32>(end - begin);
                str_t str = str_const_n(begin, lineLength);
                if (lineLength > 6 && str_cmp_n(str, "Range ", 6) == 0)
                {
                    str.m_str += 6;
                    s32 distance = 0;
                    if (from_str(str, &distance, 10))
                    {
                        *outDistanceInCm = (u16)distance;
                        return true;
                    }
                }

                available = nserial2::available();
            }

            return false;
        }
    }  // namespace nsensors
}  // namespace ncore

