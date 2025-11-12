#ifndef __RDNO_SENSORS_FRAME_READER_H__
#define __RDNO_SENSORS_FRAME_READER_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

#include "rdno_core/c_serial.h"

namespace ncore
{
    namespace nserial
    {
        struct frame_sequence_t
        {
            frame_sequence_t(u8 const *sequence, s16 length)
                : mSequence(sequence)
                , mLength(length)
            {
            }
            u8 const *const mSequence;  // Pointer to the byte sequence to match
            const s16       mLength;    // Length of the byte sequence
        };

        struct frame_data_t
        {
            frame_data_t(s16 minFrameLen, s16 maxFrameLen)
                : mStartPtr(nullptr)
                , mEndPtr(nullptr)
                , mMinFrameLen(minFrameLen)
                , mMaxFrameLen(maxFrameLen)
            {
            }

            u8 const *mStartPtr;     // Start frame position
            u8 const *mEndPtr;       // End frame position
            s16       mMinFrameLen;  // Minimum frame length
            s16       mMaxFrameLen;  // Maximum frame length
        };

        struct frame_reader_t
        {
            Stream                  *mSerial;                // Pointer to the serial stream
            u8                      *mSerialBuffer;          // Buffer where we are reading serial data into
            u16                      mSerialBufferCapacity;  // Capacity of the serial buffer
            u8                      *mSerialBufferWrite;     // Current write position in the serial buffer
            u8                       mSequenceCount;         // Number of sequences configured
            frame_sequence_t const **mStartSequences;        // Array of possible start sequences
            frame_sequence_t const **mEndSequences;          // Array of possible end sequences (matching the order of start sequences)
            frame_data_t            *mFrameData;             // Active frame data for each detected sequence
            s8                       mFoundSequence;         // Start Sequence that was found
            frame_data_t            *mFoundFrame;            // End Sequence was found, this is the end of that sequence in the serial buffer

            frame_reader_t()
                : mSerial(nullptr)
                , mSerialBuffer(nullptr)
                , mSerialBufferCapacity(0)
                , mSerialBufferWrite(nullptr)
                , mSequenceCount(0)
                , mStartSequences(nullptr)
                , mEndSequences(nullptr)
                , mFrameData(nullptr)
                , mFoundSequence(-1)
                , mFoundFrame(nullptr)
            {
            }

            void initialize(Stream *serial, u8 *buffer, u16 bufferCapacity);
            void set_frame_data(frame_sequence_t const **startSequences, frame_sequence_t const **endSequences, frame_data_t *framePointers, s8 sequenceCount);
            bool read(u8 const *&outFrameStart, u16 &outFrameLength, s8 &outSequenceIndex);
        };
    }  // namespace nserial
}  // namespace ncore

#endif  // __RDNO_SENSORS_FRAME_READER_H__