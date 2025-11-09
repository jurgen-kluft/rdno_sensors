#include "rdno_core/c_log.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"

#include "rdno_sensors/c_frame_reader.h"

#ifdef TARGET_ARDUINO
#    include "Arduino.h"
#    include "Wire.h"
#endif

namespace ncore
{
    namespace nserial
    {
        void frame_reader_t::initialize(Stream *serial, u8 *buffer, s32 bufferCapacity, s32 maxFrameSize)
        {
            mSerial               = serial;
            mSerialBuffer         = buffer;
            mSerialBufferCapacity = bufferCapacity;
            mMaxFrameSize         = maxFrameSize;
            mSerialBufferWrite    = mSerialBuffer;
            mCurrentFrameHeader   = nullptr;
            mCurrentFrameCursor   = mSerialBuffer;
        }

        bool frame_reader_t::read(frame_sequence_t const &header, frame_sequence_t const &tail, u8 *&outMessage, u32 &outMessageLen)
        {
            if (mCurrentFrameHeader == nullptr)
            {
                // Move bytes to the front of the buffer since we haven't found a frame header yet
                const s32 leftoverBytes = mSerialBufferWrite - mCurrentFrameCursor;
                if (leftoverBytes > 0 && mCurrentFrameCursor > mSerialBuffer)
                {
                    memmove(mSerialBuffer, mCurrentFrameCursor, leftoverBytes);
                    mCurrentFrameCursor = mSerialBuffer;
                    mSerialBufferWrite  = mSerialBuffer + leftoverBytes;
                }
            }

            // Read available bytes into buffer using readBytes
            s32 available = mSerial->available();
            if (available > 0 && mSerialBufferWrite < capacity())
            {
                const s32 spaceLeft = capacity() - mSerialBufferWrite;
                const s32 toRead    = (available < spaceLeft) ? available : spaceLeft;
                mSerialBufferWrite += mSerial->readBytes(&serialBuffer[mSerialBufferWrite], toRead);
            }

            if (mCurrentFrameHeader == nullptr)
            {
                // Search for frame header
                while (mCurrentFrameCursor + header.mLength <= mSerialBufferWrite)
                {
                    if (memcmp(mCurrentFrameCursor, header.mData, header.mLength) == 0)
                    {
                        mCurrentFrameHeader = mCurrentFrameCursor;
                        mCurrentFrameCursor += header.mLength;
                        break;
                    }
                    mCurrentFrameCursor++;
                }
                if (mCurrentFrameHeader == nullptr)
                {
                    // The last few bytes might be the start of a header, so we can't discard them
                    return false;
                }
            }

            // Guard for maximum frame size, e.g. we found a header but for some reason the end
            // of a frame never arrives and we keep accumulating data in the buffer.
            if ((mSerialBufferWrite - mCurrentFrameHeader) > mMaxFrameSize)
            {
                // Discard current frame search
                mCurrentFrameHeader = nullptr;
                mCurrentFrameCursor = mSerialBufferWrite;
                return false;
            }

            // Search for 'frame end'
            uint8_t const *currentFrameEnd = nullptr;
            while (mCurrentFrameCursor + tail.mLength <= mSerialBufferWrite)
            {
                if (memcmp(mCurrentFrameCursor, tail.mData, tail.mLength) == 0)
                {
                    currentFrameEnd = mCurrentFrameCursor + tail.mLength;
                    break;
                }
                mCurrentFrameCursor++;
            }

            if (currentFrameEnd == nullptr)
                return false;

            // Setup found message
            outMessageLen = (uint32_t)(currentFrameEnd - mCurrentFrameHeader);
            outMessage    = mCurrentFrameHeader;

            mCurrentFrameHeader = nullptr;
            mCurrentFrameCursor = currentFrameEnd;

            return true;
        }
    }  // namespace nserial
}  // namespace ncore