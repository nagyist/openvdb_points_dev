///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015-2016 Double Negative Visual Effects
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of Double Negative Visual Effects nor the names
// of its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
// LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
//
///////////////////////////////////////////////////////////////////////////


#include <cppunit/extensions/HelperMacros.h>
#include <openvdb_points/tools/ArrayCompression.h>

#include <sstream>
#include <iostream>
#include <algorithm> // std::random_shuffle

using namespace openvdb;
using namespace openvdb::tools;

class TestArrayCompression: public CppUnit::TestCase
{
public:
    CPPUNIT_TEST_SUITE(TestArrayCompression);
    CPPUNIT_TEST(testIntegerCompression);
    // CPPUNIT_TEST(testBloscCompression);

    CPPUNIT_TEST_SUITE_END();

    void testIntegerCompression();
    // void testBloscCompression();
}; // class TestArrayCompression

CPPUNIT_TEST_SUITE_REGISTRATION(TestArrayCompression);


////////////////////////////////////////


namespace {

template <typename T>
bool compressionTest(const std::vector<T>& values)
{
    const size_t count = values.size();

    // copy into a unique ptr array
    std::unique_ptr<T[]> input(new T[count]);
    for (int i = 0; i < count; i++)     input[i] = values[i];

    // compute compressed size
    size_t sizeAnalysis = compressedArraySize<T>(input, count);

    // perform compression
    size_t size;
    std::unique_ptr<T[]> compressedBuffer = compressArray<T>(input, count, size);

    // acceptable to fail to compress
    if (!compressedBuffer)  return true;

    // fail if size does not match analysis size
    if (size != sizeAnalysis)   return false;

    // perform decompression
    std::unique_ptr<T[]> uncompressedBuffer = decompressArray<T>(compressedBuffer, count);

    // fail if any element of original buffer does not match the uncompressed one
    for (size_t i = 0; i < count; i++) {
        if (input.get()[i] != uncompressedBuffer.get()[i])  return false;
    }

    return true;
}

} // namespace


void
TestArrayCompression::testIntegerCompression()
{
    // float test

    { // float
        std::vector<float> values;
        values.reserve(10);
        for (int i = 0; i < 10; i++)                                    values.push_back(float(i));
        std::random_shuffle(values.begin(), values.end());

        compressionTest(values);
    }

    // unsigned integer tests

    { // uint8_t
        std::vector<uint8_t> values;
        values.reserve(256);

        const uint8_t max = std::numeric_limits<uint8_t>::max();
        for (uint8_t i = 0; i < max; i++)                               values.push_back(uint8_t(i));     // 0 -> 256
        std::random_shuffle(values.begin(), values.end());

        compressionTest(values);
    }

    { // uint16_t
        std::vector<uint16_t> values;
        values.reserve(2000);

        const uint16_t max = std::numeric_limits<uint16_t>::max();
        for (uint16_t i = 0; i < 1000; i++)                             values.push_back(i);      // 0 -> 1000
        for (uint16_t i = max - 1000; i < max; i++)                     values.push_back(i);      // up to USHRT_MAX
        std::random_shuffle(values.begin(), values.end());

        compressionTest(values);
    }

    { // uint32_t
        std::vector<uint32_t> values;
        values.reserve(4000);

        const uint32_t max = std::numeric_limits<uint32_t>::max();
        for (uint32_t i = 0; i < 1000; i++)                             values.push_back(i);        // 0 -> 1000
        for (uint32_t i = 65000; i < 66000; i++)                        values.push_back(i);        // (2^16 == 65536)
        for (uint32_t i = 16777000; i < 16778000; i++)                  values.push_back(i);        // (2^24 = 16777216)
        for (uint32_t i = max - 1000; i < max; i++)                     values.push_back(i);        // up to UINT_MAX
        std::random_shuffle(values.begin(), values.end());

        compressionTest(values);
    }

    { // uint64_t
        std::vector<uint64_t> values;
        values.reserve(6000);

        const uint64_t max = std::numeric_limits<uint64_t>::max();
        for (uint64_t i = 0; i < 1000; i++)                             values.push_back(i);    // 0 -> 1000
        for (uint64_t i = 65000; i < 66000; i++)                        values.push_back(i);    // (2^16 == 65536)
        for (uint64_t i = 16777000; i < 16778000; i++)                  values.push_back(i);    // (2^24 = 16777216)
        for (uint64_t i = 4294967000; i < 4294968000; i++)              values.push_back(i);    // (2^32 = 4294967296)
        for (uint64_t i = 281474976710000; i < 281474976711000; i++)    values.push_back(i);    // (2^48 = 281474976710656)
        for (uint64_t i = max - 1000; i < max; i++)                     values.push_back(i);    // up to ULONG_MAX
        std::random_shuffle(values.begin(), values.end());

        compressionTest(values);
    }

    { // int8_t
        std::vector<int8_t> values;
        values.reserve(256);

        const int8_t max = std::numeric_limits<int8_t>::max();
        for (int8_t i = -max; i < max; i++)                             values.push_back(i);     // -127 -> 127
        values.push_back(int8_t(-0));
        std::random_shuffle(values.begin(), values.end());

        compressionTest(values);
    }

    { // int16_t
        std::vector<int16_t> values;
        values.reserve(3000);

        const int16_t max = std::numeric_limits<int16_t>::max();
        for (int16_t i = -max; i < -max + 1000; i++)                    values.push_back(i);      // up to -USHRT_MAX
        for (int16_t i = -500; i < 500; i++)                            values.push_back(i);      // -500 -> 500
        for (int16_t i = max - 1000; i < max; i++)                      values.push_back(i);      // up to USHRT_MAX
        values.push_back(int16_t(-0));
        std::random_shuffle(values.begin(), values.end());

        compressionTest(values);
    }

    { // int32_t
        std::vector<int32_t> values;
        values.reserve(7000);

        const int32_t max = std::numeric_limits<int32_t>::max();
        for (int32_t i = -max; i < -max + 1000; i++)                    values.push_back(i);        // up to -UINT_MAX
        for (int32_t i = -8389000; i < -8388000; i++)                   values.push_back(i);        // (-2^23 = -8388608)
        for (int32_t i = -33000; i < -32000; i++)                       values.push_back(i);        // (-2^15 == -32767)
        for (int32_t i = -500; i < 500; i++)                            values.push_back(i);        // 0 -> 1000
        for (int32_t i = 32000; i < 33000; i++)                         values.push_back(i);        // (2^15 == 32767)
        for (int32_t i = 8388000; i < 8389000; i++)                     values.push_back(i);        // (2^23 = 8388608)
        for (int32_t i = max - 1000; i < max; i++)                      values.push_back(i);        // up to UINT_MAX
        values.push_back(int32_t(-0));
        std::random_shuffle(values.begin(), values.end());

        compressionTest(values);
    }

    { // int64_t
        std::vector<int64_t> values;
        values.reserve(11000);

        const int64_t max = std::numeric_limits<int64_t>::max();
        for (int64_t i = -max; i < -max + 1000; i++)                    values.push_back(i);        // up to -ULONG_MAX
        for (int64_t i = -140737488356000; i < -140737488355000; i++)   values.push_back(i);        // (-2^47 = -140737488355328)
        for (int64_t i = -2147484000; i < -2147483000; i++)             values.push_back(i);        // (-2^31 = -2147483648)
        for (int64_t i = -8389000; i < -8388000; i++)                   values.push_back(i);        // (-2^23 = -8388608)
        for (int64_t i = -33000; i < -32000; i++)                       values.push_back(i);        // (-2^15 == -32767)
        for (int64_t i = -500; i < 500; i++)                            values.push_back(i);        // 0 -> 1000
        for (int64_t i = 32000; i < 33000; i++)                         values.push_back(i);        // (2^15 == 32767)
        for (int64_t i = 8388000; i < 8389000; i++)                     values.push_back(i);        // (2^23 = 8388608)
        for (int64_t i = 2147483000; i < 2147484000; i++)               values.push_back(i);        // (2^31 = 2147483648)
        for (int64_t i = 140737488355000; i < 140737488356000; i++)     values.push_back(i);        // (2^47 = 140737488355328)
        for (int64_t i = max - 1000; i < max; i++)                      values.push_back(i);        // up to ULONG_MAX
        values.push_back(int64_t(-0));
        std::random_shuffle(values.begin(), values.end());

        compressionTest(values);
    }

    { // uint32_t (ascending)
        std::vector<uint32_t> values;
        values.reserve(4000);

        for (uint32_t i = 0; i < 4000; i++)                             values.push_back(i);        // 0 -> 4000

        compressionTest(values);
    }

    { // uint32_t (ascending, except first values)
        std::vector<uint32_t> values;
        values.reserve(4000);

        values.push_back(uint32_t(1024*1024));
        for (uint32_t i = 0; i < 4000; i++)                             values.push_back(i);        // 0 -> 4000

        compressionTest(values);
    }

    { // uint32_t (empty)
        std::vector<uint64_t> values;

        compressionTest(values);
    }

    { // uint32_t (one value)
        std::vector<uint64_t> values;

        values.push_back(uint64_t(20000));

        compressionTest(values);
    }

    { // uint32_t (two values, different)
        std::vector<uint64_t> values;

        values.push_back(uint64_t(20000));
        values.push_back(uint64_t(50000));

        compressionTest(values);
    }

    { // uint32_t (two values, matching)
        std::vector<uint64_t> values;

        values.push_back(uint64_t(20000));
        values.push_back(uint64_t(20000));

        compressionTest(values);
    }

    { // uint32_t (uniform)
        std::vector<uint64_t> values;
        values.reserve(4000);

        for (uint64_t i = 0; i < 4000; i++)                              values.push_back(1000);

        compressionTest(values);
    }

    { // uint32_t (uniform, except first value)
        std::vector<uint64_t> values;
        values.reserve(4000);

        values.push_back(uint64_t(1024*1024));
        for (uint64_t i = 0; i < 4000; i++)                              values.push_back(1000);

        compressionTest(values);
    }

    { // uint32_t (repeated)
        std::vector<uint64_t> values;
        values.reserve(4000);

        for (uint64_t i = 0; i < 4000; i++)                              values.push_back(i / 100);

        compressionTest(values);
    }

    { // uint32_t (repeated, alternate)
        std::vector<uint64_t> values;
        values.reserve(4000);

        for (uint64_t i = 0; i < 4000; i++)                              values.push_back(i / 100 + (1UL << 40));

        compressionTest(values);
    }

    { // uint32_t (ascending, pairs)
        std::vector<uint32_t> values;
        values.reserve(4000);

        for (uint32_t i = 0; i < 4000; i++)                              values.push_back(i + ((i % 2) == 0) + (1UL << 28));

        compressionTest(values);
    }
}

#if 0
void
TestArrayCompression::testBloscCompression()
{
    const int count = 256;

    using UniqueIntPtr = std::unique_ptr<int[]>;

    { // invalid buffer (out of range)
#ifdef OPENVDB_USE_BLOSC
        // compress

        UniqueIntPtr smallBuffer(new int[count]);
        for (int i = 0; i < count; i++)   smallBuffer.get()[i] = i;

        size_t limit = INT_MAX - 1;

        size_t testCompressedBytes = bloscCompressSize(smallBuffer, limit);

        CPPUNIT_ASSERT_EQUAL(testCompressedBytes, size_t(0));

        UniqueIntPtr buffer = bloscToBuffer(smallBuffer, limit, testCompressedBytes);

        CPPUNIT_ASSERT(!buffer);
        CPPUNIT_ASSERT_EQUAL(testCompressedBytes, size_t(0));

        // decompress

        UniqueIntPtr compressedBuffer = bloscToBuffer(smallBuffer, count, testCompressedBytes);

        UniqueIntPtr simpleBuffer = bloscFromBuffer<int>(compressedBuffer, limit);

        CPPUNIT_ASSERT(!simpleBuffer);

        CPPUNIT_ASSERT_THROW(bloscFromBuffer<int>(compressedBuffer, count + 1), openvdb::RuntimeError);
#endif
    }

    { // with cleanup
        // compress

        UniqueIntPtr uncompressedBuffer(new int[count]);

        for (int i = 0; i < count; i++) {
            uncompressedBuffer.get()[i] = i / 2;
        }

        size_t uncompressedBytes = count * sizeof(int);
        size_t compressedBytes;

        size_t testCompressedBytes = bloscCompressSize(uncompressedBuffer, count);

        UniqueIntPtr compressedBuffer = bloscToBuffer(uncompressedBuffer, count, compressedBytes);

#ifdef OPENVDB_USE_BLOSC
        CPPUNIT_ASSERT(compressedBytes < uncompressedBytes);
        CPPUNIT_ASSERT(compressedBuffer);
        CPPUNIT_ASSERT_EQUAL(testCompressedBytes, compressedBytes);

        // decompress

        UniqueIntPtr newUncompressedBuffer = bloscFromBuffer<int>(compressedBuffer, count);

        CPPUNIT_ASSERT(newUncompressedBuffer);
#else
        CPPUNIT_ASSERT(!compressedBuffer);
        CPPUNIT_ASSERT_EQUAL(testCompressedBytes, size_t(0));

        // decompress

        CPPUNIT_ASSERT_THROW(bloscFromBuffer<int>(compressedBuffer, count), openvdb::RuntimeError);
#endif
    }

    { // without cleanup
        // compress

        UniqueIntPtr uncompressedBuffer(new int[count]);

        for (int i = 0; i < count; i++) {
            uncompressedBuffer[i] = i / 2;
        }

        size_t uncompressedBytes = count * sizeof(int);
        size_t compressedBytes;

        UniqueIntPtr compressedBuffer = bloscToBuffer(uncompressedBuffer, count, compressedBytes);

#ifdef OPENVDB_USE_BLOSC
        CPPUNIT_ASSERT(compressedBytes < uncompressedBytes);
        CPPUNIT_ASSERT(compressedBuffer);

        // decompress

        UniqueIntPtr newUncompressedBuffer = bloscFromBuffer<int>(compressedBuffer, count);

        CPPUNIT_ASSERT(newUncompressedBuffer);

        for (int i = 0; i < count; i++) {
            CPPUNIT_ASSERT_EQUAL(uncompressedBuffer[i], reinterpret_cast<int*>(newUncompressedBuffer.get())[i]);
        }
#else
        // decompress

        CPPUNIT_ASSERT_THROW(bloscFromBuffer<int>(compressedBuffer, count), openvdb::RuntimeError);
#endif
    }
}
#endif

// Copyright (c) 2015-2016 Double Negative Visual Effects
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
