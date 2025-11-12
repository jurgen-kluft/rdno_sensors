#include "rdno_core/c_target.h"
#include "ccore/c_allocator.h"
#include "rdno_sensors/c_frame_reader.h"

#include "cunittest/cunittest.h"

using namespace ncore;
using namespace ncore::nserial;

UNITTEST_SUITE_BEGIN(frame_reader)
{
    class TestStream : public Stream
    {
    public:
        const byte* mTestData;
        s32         mReadSize;
        s32         mTestByteSize;
        s32         mTestByteCursor;

        virtual ~TestStream() {}

        virtual ncore::s32 available()
        {
            s32 count = mTestByteSize - mTestByteCursor;
            if (mReadSize > 0)
            {
                if (count == 0)
                    return 0;
                if (count > mReadSize)
                    count = count % mReadSize;
                if (count == 0)
                    count = 1;
            }
            return count;
        }
        virtual ncore::s32 read()
        {
            if (mTestByteCursor < mTestByteSize)
            {
                return mTestData[mTestByteCursor++];
            }
            return -1;
        }

        virtual ncore::s32 readBytes(ncore::u8* buffer, ncore::s32 length)
        {
            s32 bytesRead = 0;
            while (bytesRead < length && mTestByteCursor < mTestByteSize)
            {
                buffer[bytesRead] = mTestData[mTestByteCursor];
                ++mTestByteCursor;
                ++bytesRead;
            }
            return bytesRead;
        }
    };

    const byte frame_start_A[] = {0xAA, 0xBB, 0xCC, 0xDD};
    const byte frame_end_A[]   = {0xF1, 0xF2, 0xF3, 0xF4};
    const byte frame_start_B[] = {0xFA, 0xFB, 0xFC, 0xFD};
    const byte frame_end_B[]   = {0xF6, 0xF7, 0xF8, 0xF9};

    UNITTEST_FIXTURE(single_sequence)
    {
        // clang-format off
        const byte testData[] = {
            0x00, 0x11, 0x22, 
            0xAA, 0xBB, 0xCC, 0xDD,  // frame start
            0x10, 0x20, 0x30, 0x40,  // frame data
            0xF1, 0xF2, 0xF3, 0xF4,  // frame end
            0x88, 0x99, 0xAA, 
            0xAA, 0xBB, 0xCC, 0xDD,  // frame start
            0x50, 0x60, 0x70, 0x80,  // frame data
            0xF1, 0xF2, 0xF3, 0xF4,  // frame end
            0x88, 0x99, 0x00, 0x11, 
            0x22, 0x33, 0x44, 0x55, 
            0x66, 0x77, 
            0xAA, 0xBB, 0xCC, 0xDD,  // frame start
            0x51, 0x61, 0x71, 0x81,  // frame data
            0xF1, 0xF2, 0xF3, 0xF4,  // frame end
            0x88, 0x99};
        // clang-format on

        UNITTEST_TEST(setup)
        {
            TestStream testStream;
            testStream.mTestData       = testData;
            testStream.mReadSize       = 0;
            testStream.mTestByteSize   = DARRAYSIZE(testData);
            testStream.mTestByteCursor = 0;

            frame_sequence_t startSeq(frame_start_A, DARRAYSIZE(frame_start_A));
            frame_sequence_t endSeq(frame_end_A, DARRAYSIZE(frame_end_A));

            u8        serialBuffer[128];
            const u16 serialBufferSize = DARRAYSIZE(serialBuffer);

            frame_data_t   frameData[] = {frame_data_t(8, 16)};
            frame_reader_t frameReader;
            frameReader.initialize(&testStream, serialBuffer, serialBufferSize);
            frame_sequence_t const* startSeqs[] = {&startSeq};
            frame_sequence_t const* endSeqs[]   = {&endSeq};
            frameReader.set_frame_data(startSeqs, endSeqs, frameData, 1);

            u8 const* outMessage       = nullptr;
            u16       outMessageLen    = 0;
            s8        outSequenceIndex = -1;

            // Read first frame
            bool result = frameReader.read(outMessage, outMessageLen, outSequenceIndex);
            CHECK_TRUE(result);
            CHECK_NOT_NULL(outMessage);
            CHECK_EQUAL(12, outMessageLen);  // header + data + tail
            CHECK_EQUAL(0, outSequenceIndex);
            CHECK_EQUAL(0x10, outMessage[4 + 0]);
            CHECK_EQUAL(0x20, outMessage[4 + 1]);
            CHECK_EQUAL(0x30, outMessage[4 + 2]);
            CHECK_EQUAL(0x40, outMessage[4 + 3]);

            // Read second frame
            result = frameReader.read(outMessage, outMessageLen, outSequenceIndex);
            CHECK_TRUE(result);
            CHECK_NOT_NULL(outMessage);
            CHECK_EQUAL(12, outMessageLen);
            CHECK_EQUAL(0, outSequenceIndex);
            CHECK_EQUAL(0x50, outMessage[4 + 0]);
            CHECK_EQUAL(0x60, outMessage[4 + 1]);
            CHECK_EQUAL(0x70, outMessage[4 + 2]);
            CHECK_EQUAL(0x80, outMessage[4 + 3]);
        }
    }

    UNITTEST_FIXTURE(multiple_sequences)
    {
        // clang-format off
        const byte testData[] = {
            0x00, 0x11, 0x22, 
            0xAA, 0xBB, 0xCC, 0xDD,  // frame A start
            0x10, 0x20, 0x30, 0x40,  // frame data
            0xF1, 0xF2, 0xF3, 0xF4,  // frame A end
            0x88, 0x99, 0xAA, 
            0xFA, 0xFB, 0xFC, 0xFD,  // frame B start
            0x50, 0x60, 0x70, 0x80,  // frame data
            0xF6, 0xF7, 0xF8, 0xF9,  // frame B end
            0x88, 0x99, 0x00, 0x11, 
            0x22, 0x33, 0x44, 0x55, 
            0x66, 0x77, 
            0xAA, 0xBB, 0xCC, 0xDD,  // frame A start
            0x51, 0x61, 0x71, 0x81,  // frame data
            0xF1, 0xF2, 0xF3, 0xF4,  // frame A end
            0x88, 0x99,
            0x88, 0x99, 0xAA, 
            0xFA, 0xFB, 0xFC, 0xFD,  // frame B start
            0x52, 0x62, 0x72, 0x82,  // frame data
            0xF6, 0xF7, 0xF8, 0xF9,  // frame B end
            0x88, 0x99, 0x00, 0x11, 
            };
        // clang-format on

        UNITTEST_TEST(setup)
        {
            TestStream testStream;
            testStream.mTestData       = testData;
            testStream.mReadSize       = 0;
            testStream.mTestByteSize   = DARRAYSIZE(testData);
            testStream.mTestByteCursor = 0;

            const frame_sequence_t startSeqA(frame_start_A, DARRAYSIZE(frame_start_A));
            const frame_sequence_t endSeqA(frame_end_A, DARRAYSIZE(frame_end_A));
            const frame_sequence_t startSeqB(frame_start_B, DARRAYSIZE(frame_start_B));
            const frame_sequence_t endSeqB(frame_end_B, DARRAYSIZE(frame_end_B));

            u8        serialBuffer[512];
            const u16 serialBufferSize = DARRAYSIZE(serialBuffer);

            frame_reader_t frameReader;
            frameReader.initialize(&testStream, serialBuffer, serialBufferSize);

            frame_sequence_t const* startSequences[] = {&startSeqA, &startSeqB};
            frame_sequence_t const* endSequences[]   = {&endSeqA, &endSeqB};
            frame_data_t            frameData[]      = {frame_data_t(8, 16), frame_data_t(8, 16)};
            frameReader.set_frame_data(startSequences, endSequences, frameData, 2);

            u8 const* outMessage       = nullptr;
            u16       outMessageLen    = 0;
            s8        outSequenceIndex = -1;

            // Read first frame (A)
            bool result = frameReader.read(outMessage, outMessageLen, outSequenceIndex);
            CHECK_TRUE(result);
            CHECK_NOT_NULL(outMessage);
            CHECK_EQUAL(12, outMessageLen);  // header + data + tail
            CHECK_EQUAL(0, outSequenceIndex);
            CHECK_EQUAL(0x10, outMessage[4 + 0]);
            CHECK_EQUAL(0x20, outMessage[4 + 1]);
            CHECK_EQUAL(0x30, outMessage[4 + 2]);
            CHECK_EQUAL(0x40, outMessage[4 + 3]);

            // Read second frame (B)
            result = frameReader.read(outMessage, outMessageLen, outSequenceIndex);
            CHECK_TRUE(result);
            CHECK_NOT_NULL(outMessage);
            CHECK_EQUAL(12, outMessageLen);
            CHECK_EQUAL(1, outSequenceIndex);
            CHECK_EQUAL(0x50, outMessage[4 + 0]);
            CHECK_EQUAL(0x60, outMessage[4 + 1]);
            CHECK_EQUAL(0x70, outMessage[4 + 2]);
            CHECK_EQUAL(0x80, outMessage[4 + 3]);

            // Read third frame (A)
            result = frameReader.read(outMessage, outMessageLen, outSequenceIndex);
            CHECK_TRUE(result);
            CHECK_NOT_NULL(outMessage);
            CHECK_EQUAL(12, outMessageLen);
            CHECK_EQUAL(0, outSequenceIndex);
            CHECK_EQUAL(0x51, outMessage[4 + 0]);
            CHECK_EQUAL(0x61, outMessage[4 + 1]);
            CHECK_EQUAL(0x71, outMessage[4 + 2]);
            CHECK_EQUAL(0x81, outMessage[4 + 3]);

            // Read fourth frame (B)
            result = frameReader.read(outMessage, outMessageLen, outSequenceIndex);
            CHECK_TRUE(result);
            CHECK_NOT_NULL(outMessage);
            CHECK_EQUAL(12, outMessageLen);
            CHECK_EQUAL(1, outSequenceIndex);
            CHECK_EQUAL(0x52, outMessage[4 + 0]);
            CHECK_EQUAL(0x62, outMessage[4 + 1]);
            CHECK_EQUAL(0x72, outMessage[4 + 2]);
            CHECK_EQUAL(0x82, outMessage[4 + 3]);
        }
    }
}
UNITTEST_SUITE_END
