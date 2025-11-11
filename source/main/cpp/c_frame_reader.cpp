#include "rdno_core/c_log.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "ccore/c_memory.h"

#include "rdno_sensors/c_frame_reader.h"

#ifdef TARGET_ARDUINO
#    include "Arduino.h"
#    include "Wire.h"
#endif

namespace ncore
{
    namespace nserial
    {
        void frame_reader_t::initialize(Stream *serial, u8 *buffer, u16 bufferCapacity)
        {
            mSerial               = serial;
            mSerialBuffer         = buffer;
            mSerialBufferCapacity = bufferCapacity;
            mMaxFrameSize         = 64;
            mSerialBufferWrite    = mSerialBuffer;
            mSequenceCount        = 0;
            mStartSequences       = nullptr;
            mFrameData            = nullptr;
            mEndSequences         = nullptr;
            mFoundSequence        = -1;
            mFoundFrame           = nullptr;
        }

        void frame_reader_t::set_frame_data(frame_sequence_t const *startSequences, frame_sequence_t const *endSequences, frame_data_t *frameData, s8 sequenceCount)
        {
            mSequenceCount  = sequenceCount;
            mStartSequences = startSequences;
            mEndSequences   = endSequences;
            mFrameData      = frameData;
            mFoundSequence  = -1;
            mFoundFrame     = nullptr;
            mFrameData      = frameData;

            for (u8 i = 0; i < mSequenceCount; ++i)
            {
                mFrameData[i].mStartPtr = mSerialBuffer;
                mFrameData[i].mEndPtr   = nullptr;
            }
        }

        bool find_sequence(frame_sequence_t const &sequence, u8 const *&readCursor, u8 const *cursorEnd)
        {
            while ((readCursor + sequence.mLength) <= cursorEnd)
            {
                u8 j = 0;
                while (j < sequence.mLength)
                {
                    if (readCursor[j] != sequence.mSequence[j])
                        break;
                    ++j;
                }
                if (j == sequence.mLength)
                {
                    return true;
                }
                readCursor += 1;
            }
            return false;
        }

        bool frame_reader_t::read(u8 const *&outFrameStart, u16 &outFrameLength, s8 &outSequenceIndex)
        {
            if (mFoundFrame != nullptr)
            {
                // Last read found a complete frame, so move any remaining data to the front of the buffer
                const u16 remainingDataLen = (u16)(mSerialBufferWrite - mFoundFrame->mEndPtr);
                g_memmove(mSerialBuffer, outFrameStart + outFrameLength, remainingDataLen);

                // Update read/write pointer
                mSerialBufferWrite = mSerialBuffer + remainingDataLen;

                // Reset pointers for each sequence
                for (u8 i = 0; i < mSequenceCount; ++i)
                {
                    mFrameData[i].mStartPtr = mSerialBuffer;
                    mFrameData[i].mEndPtr   = nullptr;
                }

                // Reset
                mFoundFrame = nullptr;
            }

            // Read available bytes into buffer using readBytes
            s32 available = mSerial->available();
            if (available > 0 && (u16)(mSerialBufferWrite - mSerialBuffer) < mSerialBufferCapacity)
            {
                const s32 spaceLeft = mSerialBufferCapacity - (u16)(mSerialBufferWrite - mSerialBuffer);
                const s32 toRead    = (available < spaceLeft) ? available : spaceLeft;
                mSerialBufferWrite += mSerial->readBytes(mSerialBufferWrite, toRead);
            }

            if (mFoundSequence == -1)
            {
                for (u8 i = 0; i < mSequenceCount; ++i)
                {
                    if (find_sequence(mStartSequences[i], mFrameData[i].mStartPtr, mSerialBufferWrite))
                    {
                        mFrameData[i].mEndPtr = mFrameData[i].mStartPtr + mStartSequences[i].mLength;
                        mFoundSequence        = i;
                        break;
                    }
                }
                if (mFoundSequence == -1)
                {
                    // No start sequence found yet
                    return false;
                }
            }

            // Guard for maximum frame size, e.g. we found a header but for some reason the end
            // of a frame never arrives and we keep accumulating data in the buffer.
            if ((mSerialBufferWrite - mFrameData[mFoundSequence].mEndPtr) > mMaxFrameSize)
            {
                // Discard current frame search
                mFoundSequence     = -1;
                mSerialBufferWrite = mSerialBuffer;
                for (u8 i = 0; i < mSequenceCount; ++i)
                {
                    mFrameData[i].mStartPtr = mSerialBuffer;
                    mFrameData[i].mEndPtr   = nullptr;
                }
                return false;
            }

            // Search for 'frame end'
            if (!find_sequence(mEndSequences[mFoundSequence], mFrameData[mFoundSequence].mEndPtr, mSerialBufferWrite))
            {
                // No frame end found yet
                return false;
            }

            // Frame end found, set found end pointer to after the end sequence
            mFrameData[mFoundSequence].mEndPtr += mEndSequences[mFoundSequence].mLength;

            // Setup found message
            mFoundFrame      = &mFrameData[mFoundSequence];
            outFrameLength   = (u16)(mFoundFrame->mEndPtr - mFoundFrame->mStartPtr);
            outFrameStart    = mFoundFrame->mStartPtr;
            outSequenceIndex = mFoundSequence;

            // Initialize to find the next sequence
            mFoundSequence = -1;

            return true;
        }
    }  // namespace nserial
}  // namespace ncore