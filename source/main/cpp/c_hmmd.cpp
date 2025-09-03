#ifdef TARGET_ESP32

#    include "Arduino.h"
#    include "Wire.h"

#    include "rdno_core/c_serial.h"
#    include "rdno_sensors/c_hmmd.h"

namespace ncore
{
    namespace nsensors
    {
        bool initHMMD(u8 rxPin, u8 txPin)
        {
            nserial2::begin(nbaud::Rate115200, nconfig::MODE_8N1, rxPin, txPin);

            // Put the sensor into the correct output mode and keep it active
            const byte command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
            nserial2::write(command, sizeof(command));

            return true;
        }

        bool readHMMD(f32* outDistance)
        {
            char line[128];
            memset(line, 0, sizeof(line));

            // Check if data is available on Serial2
            while (nserial2::available() > 0)
            {
                // Read a line of text until a newline character (\n) is received
                s32  lineLength = nserial2::read_until('\n', line, 128 - 1);

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
                if (lineLength > 6 && strncmp(begin, "Range ", 6) == 0)
                {
                    const char* distanceStr = begin + 6;
                    *outDistance            = atof(distanceStr);  // Convert the distance string to a float
                    return true;
                }
            }

            return false;
        }
    }  // namespace nsensors
}  // namespace ncore

#else

#    include "rdno_sensors/c_hmmd.h"

namespace ncore
{
    namespace nsensors
    {
        bool initHMMD(u8 rxPin, u8 txPin)
        {
            (void)rxPin;       // Suppress unused parameter warning
            (void)txPin;       // Suppress unused parameter warning

            return false;
        }

        bool readHMMD(f32* outDistance)
        {
            *outDistance = 0.0f;  // Set default value
            return false;
        }
    }  // namespace nsensors
}  // namespace ncore

#endif