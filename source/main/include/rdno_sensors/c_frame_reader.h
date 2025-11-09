#ifndef __RDNO_SENSORS_SEEED_HSP24_H__
#define __RDNO_SENSORS_SEEED_HSP24_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

class Stream;  // Forward declaration (Arduino)

namespace ncore
{
    namespace nserial
    {
        struct frame_sequence_t
        {
            uint8_t const *mData;
            const s32      mLength;
        };

        struct frame_reader_t
        {
            Stream *mSerial;
            u8     *mSerialBuffer;
            s32     mSerialBufferCapacity;
            s32     mMaxFrameSize;
            u8     *mSerialBufferWrite;
            u8     *mCurrentFrameHeader;  // Pointer to frame header that was found
            u8     *mCurrentFrameCursor;  // Current position in the frame being read

            frame_reader_t()
                : mSerial(nullptr)
                , mSerialBuffer(nullptr)
                , mSerialBufferCapacity(0)
                , mMaxFrameSize(0)
                , mSerialBufferWrite(mSerialBuffer)
                , mCurrentFrameHeader(nullptr)
                , mCurrentFrameCursor(nullptr)
            {
            }

            void       initialize(Stream *serial, u8 *buffer, s32 bufferCapacity, s32 maxFrameSize);
            inline s32 capacity() const { return mSerialBufferCapacity; }
            bool       read(frame_sequence_t const &header, frame_sequence_t const &tail, u8 *&outMessage, u32 &outMessageLen);
        };
    }  // namespace nserial
}  // namespace ncore

#endif