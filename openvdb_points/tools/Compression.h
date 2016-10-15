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
//
/// @file Compression.h
///
/// @authors Dan Bailey


#ifndef OPENVDB_TOOLS_COMPRESSION_HAS_BEEN_INCLUDED
#define OPENVDB_TOOLS_COMPRESSION_HAS_BEEN_INCLUDED


#include <bitset>


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {

////////////////////////////////////////

namespace io {


enum INTEGER_COMPRESSION
{
    PACK4 = 0x1,
    PACK8 = 0x2,
    DIFFERENTIAL = 0x4,
    RUNLENGTH = 0x8
};


////////////////////////////////////////


namespace integer_compression_internal {


template <bool Analysis>
class BitWriter
{
public:
    BitWriter(char* buffer)
        : mBuffer(buffer)
        , mIndex(0)
        , mBytes(0)
        , mValue(0) { }

    template <int Width>
    inline void push(uint8_t value)
    {
        BOOST_STATIC_ASSERT(Width < CHAR_BIT);

        if (Width == 1)
        {
            // clamp to one-bit
            if (!Analysis && value > uint8_t(0))     value = uint8_t(1);

            if (mIndex == CHAR_BIT) {
                this->flush();
                mIndex = 0;
                if (!Analysis)  mValue = 0;
            }

            if (!Analysis)  mValue |= (value << mIndex);
            mIndex++;
        }
        else
        {
            if (!Analysis && mIndex < CHAR_BIT)      mValue |= (value << mIndex);

            if (mIndex >= CHAR_BIT - Width) {
                this->flush();
                if (!Analysis) {
                    const uint8_t mask = (UCHAR_MAX >> (CHAR_BIT - Width));
                    mValue = ((value & mask) >> (CHAR_BIT - mIndex));
                }
                mIndex -= CHAR_BIT;
            }

            mIndex += Width;
        }
    }

    inline void finalize()
    {
        if (mIndex > 0)  this->flush();
    }

    inline size_t bytes() { return mBytes; }

private:
    inline void flush()
    {
        if (!Analysis) {
            *mBuffer = mValue;
            ++mBuffer;
        }
        ++mBytes;
    }

    char* mBuffer;
    int mIndex;
    size_t mBytes;
    uint8_t mValue;
}; // class BitWriter


class BitReader
{
public:
    BitReader(const char* buffer)
        : mBuffer(buffer)
        , mIndex(CHAR_BIT)
        , mValue(0) { }

    template <int Width>
    inline uint8_t pop()
    {
        BOOST_STATIC_ASSERT(Width < CHAR_BIT);

        if (Width == 1) {
            if (mIndex == CHAR_BIT) {
                mValue = *reinterpret_cast<const uint8_t*>(mBuffer);
                ++mBuffer;
                mIndex = 0;
            }
            const uint8_t result = ((mValue & (uint8_t(1) << mIndex)) >> mIndex);
            mIndex++;
            return result;
        }
        else {
            uint8_t key(0);
            if ((CHAR_BIT - mIndex) < Width)
            {
                if ((CHAR_BIT - mIndex) > 0)  key = (mValue >> mIndex) & (UCHAR_MAX >> mIndex);
                mValue = *reinterpret_cast<const uint8_t*>(mBuffer);
                ++mBuffer;
                const uint8_t mask = (UCHAR_MAX >> (2*CHAR_BIT - mIndex - Width));
                key |= (mValue & mask) << (CHAR_BIT - mIndex);
                mIndex = (mIndex + Width) % CHAR_BIT;
            }
            else
            {
                uint8_t mask = (UCHAR_MAX >> (CHAR_BIT - Width));
                key = (mValue >> mIndex) & mask;
                mIndex += Width;
            }
            return key;
        }
    }

private:
    const char* mBuffer;
    int mIndex;
    uint8_t mValue;
}; // class BitReader


template <bool Analysis>
class BufferWriter
{
public:
    BufferWriter(char* buffer)
        : mBuffer(buffer)
        , mBytes(0) { }

    template <typename T>
    void write(T value, int size)
    {
        int offset = 0;
        for (; size >= 4; size -= 4) {
            if (!Analysis) {
                *reinterpret_cast<uint32_t*>(mBuffer + offset) = uint32_t(value >> (offset * CHAR_BIT));
            }
            offset += 4;
        }
        if (size >= 2) {
            if (!Analysis) {
                *reinterpret_cast<uint16_t*>(mBuffer + offset) = uint16_t(value >> (offset * CHAR_BIT));
            }
            offset += 2;
        }
        if ((size % 2) == 1) {
            if (!Analysis) {
                *reinterpret_cast<uint8_t*>(mBuffer + offset) = uint8_t(value >> (offset * CHAR_BIT));
            }
            offset += 1;
        }

        if (!Analysis)  mBuffer += offset;
        mBytes += offset;
    }

    inline size_t bytes() { return mBytes; }

private:
    char* mBuffer;
    size_t mBytes;
}; // class BufferWriter


class BufferReader
{
public:
    BufferReader(const char* buffer)
        : mBuffer(buffer) { }

    template <typename T>
    T read(int size)
    {
        T value(0);
        int offset = 0;
        for (; size >= 4; size -= 4) {
            value |= T(*reinterpret_cast<const uint32_t*>(mBuffer + offset)) << (offset * CHAR_BIT);
            offset += 4;
        }
        if (size >= 2) {
            value |=  T(*reinterpret_cast<const uint16_t*>(mBuffer + offset)) << (offset * CHAR_BIT);
            offset += 2;
        }
        if ((size % 2) == 1) {
            value |=  T(*reinterpret_cast<const uint8_t*>(mBuffer + offset)) << (offset * CHAR_BIT);
            offset += 1;
        }
        mBuffer += offset;
        return value;
    }

private:
    const char* mBuffer;
};

} // namespace integer_compression_internal


////////////////////////////////////////


template <typename IntT, int bits, bool Differential, bool RunLength, bool Analysis=false>
void encodeInteger( char* headerBuffer, size_t& headerSize,
                    char* data, size_t& dataSize,
                    const IntT* input, const size_t count)
{
    BOOST_STATIC_ASSERT(bits == 1 || bits == 2 || bits == 3);
    typedef typename std::make_unsigned<IntT>::type UIntT;

    struct Local
    {
        static int computeKey(UIntT value)
        {
            uint8_t key(0);
            while (value != UIntT(0))
            {
                const int multiplier = sizeof(UIntT) > 1 ? 1 : 0;
                value >>= multiplier * CHAR_BIT;
                if (++key >= (1UL << bits) - 1)   break;
            }
            return key;
        }

        static int clampKey(uint8_t key)
        {
            if (key >= (1UL << bits) - 1)   return sizeof(IntT);
            else                            return key;
        }

        static UIntT rotateLeftShift(IntT value)
        {
            const UIntT unsignedValue(static_cast<const UIntT&>(value));
            const int signShift = sizeof(UIntT) * CHAR_BIT - 1;
            if (unsignedValue >> signShift == 0)    return unsignedValue << 1;
            return (~unsignedValue << 1) | UIntT(1);
        }
    }; // struct Local

    if (count == 0) {
        headerSize = 0;
        dataSize = 0;
        return;
    }

    integer_compression_internal::BitWriter<Analysis ? true : false> header(headerBuffer);
    integer_compression_internal::BufferWriter<Analysis ? true : false> bufferWriter(data);

    int repeat(0);

    UIntT unsignedValue;
    if (std::is_unsigned<IntT>())   unsignedValue = input[0];
    else                            unsignedValue = Local::rotateLeftShift(input[0]);

    // write key

    UIntT value = unsignedValue;
    uint8_t key = Local::computeKey(value);

    for (size_t i = 1; i < count; i++)
    {
        const UIntT signedValue = Differential ? input[i] - input[i-1] : input[i];
        if (std::is_unsigned<IntT>())   unsignedValue = signedValue;
        else                            unsignedValue = Local::rotateLeftShift(signedValue);

        // write repeat

        if (RunLength)
        {
            if (value == unsignedValue) {
                repeat++;
                continue;
            }

            // write run-length bits (ones followed by terminating zero)

            for (int j = 0; j < repeat; j++)   header.template push<1>(1);
            header.template push<1>(0);

            repeat = 0;
        }

        bufferWriter.template write(value, Local::clampKey(key));

        header.template push<bits>(key);

        value = unsignedValue;
        key = Local::computeKey(value);
    }

    // write run-length bits (ones followed by terminating zero)

    if (RunLength)
    {
        for (int j = 0; j < repeat; j++)   header.template push<1>(1);
        header.template push<1>(0);
    }

    bufferWriter.template write(value, Local::clampKey(key));

    header.template push<bits>(key);
    header.template finalize();

    headerSize = header.template bytes();
    dataSize = bufferWriter.bytes();
}


template <typename IntT, int Bits, bool Differential, bool RunLength>
size_t encodeIntegerAnalysis(const IntT* input, const size_t count)
{
    size_t headerSize;
    size_t bufferSize;

    encodeInteger<IntT, Bits, Differential, RunLength, true>(NULL, headerSize, NULL, bufferSize, input, count);

    return headerSize + bufferSize;
}


template <typename IntT, int Bits, bool Differential, bool RunLength>
void decodeInteger(IntT* output, const size_t count, const char* headerBuffer, const char* data)
{
    BOOST_STATIC_ASSERT(Bits == 1 || Bits == 2 || Bits == 3);

    typedef typename std::make_unsigned<IntT>::type UIntT;

    struct Local
    {
        static int clampKey(uint8_t key)
        {
            if (key >= (1UL << Bits) - 1)   return sizeof(IntT);
            else                            return key;
        }

        static IntT rotateRightShift(UIntT value)
        {
            if ((value & UIntT(1)) == 0)   return static_cast<IntT>(value >> 1);
            return static_cast<IntT>((~value >> 1) | (UIntT(1) << (sizeof(UIntT) * CHAR_BIT - 1)));
        }
    }; // struct Local

    integer_compression_internal::BitReader header(headerBuffer);
    integer_compression_internal::BufferReader buffer(data);

    int index(0);

    while (index < count)
    {
        // extract counter

        int repeat = 1;
        if (RunLength) {
            while (header.pop<1>())   repeat++;
        }

        // read key

        const uint8_t key = header.pop<Bits>();

        // read data

        const UIntT unsignedValue = buffer.read<UIntT>(Local::clampKey(key));
        const IntT signedValue = std::is_unsigned<IntT>() ? unsignedValue :
                                                            Local::rotateRightShift(unsignedValue);

        for (int i = 0; i < repeat; i++)
        {
            output[index] = signedValue;
            if (index > 0 && Differential)      output[index] += output[index-1];
            index++;
        }
    }
}


////////////////////////////////////////


template <typename T, bool Analysis>
size_t writeCompressedIntegers(std::ostream& os, const T* input, const size_t count)
{
    const bool pack8 = sizeof(T) >= 8;

    // allocate temporary buffers

    boost::scoped_array<char> buffer;
    boost::scoped_array<char> header;

    boost::scoped_array<char> tempBuffer;
    boost::scoped_array<char> tempHeader;

    if (!Analysis) {
        buffer.reset(new char[count*sizeof(T)]);
        header.reset(new char[count]);

        tempBuffer.reset(new char[count*sizeof(T)]);
        tempHeader.reset(new char[count]);
    }

    size_t tempHeaderSize;
    size_t tempBufferSize;

    // uncompressed

    uint8_t newFlags(0);

    size_t headerSize(0);
    size_t bufferSize(sizeof(T) * count);
    size_t size = headerSize + bufferSize;

    // packed compression

    encodeInteger<T, (sizeof(T) >= 8) ? 3 : 2, false, false, Analysis>(tempHeader.get(), tempHeaderSize, tempBuffer.get(), tempBufferSize, input, count);

    if (tempHeaderSize + tempBufferSize < size) {
        if (!Analysis) {
            header.swap(tempHeader);
            buffer.swap(tempBuffer);
        }

        newFlags = pack8 ? PACK8 : PACK4;

        headerSize = tempHeaderSize;
        bufferSize = tempBufferSize;
        size = headerSize + bufferSize;
    }

    // packed, differential compression

    encodeInteger<T, (sizeof(T) >= 8) ? 3 : 2, true, false, Analysis>(tempHeader.get(), tempHeaderSize, tempBuffer.get(), tempBufferSize, input, count);

    if (tempHeaderSize + tempBufferSize < size) {
        if (!Analysis) {
            header.swap(tempHeader);
            buffer.swap(tempBuffer);
        }

        newFlags = (pack8 ? PACK8 : PACK4) | DIFFERENTIAL;

        headerSize = tempHeaderSize;
        bufferSize = tempBufferSize;
        size = headerSize + bufferSize;
    }

    // packed, run-length compression

    encodeInteger<T, (sizeof(T) >= 8) ? 3 : 2, false, true, Analysis>(tempHeader.get(), tempHeaderSize, tempBuffer.get(), tempBufferSize, input, count);

    if (tempHeaderSize + tempBufferSize < size) {
        if (!Analysis) {
            header.swap(tempHeader);
            buffer.swap(tempBuffer);
        }

        newFlags = (pack8 ? PACK8 : PACK4) | RUNLENGTH;

        headerSize = tempHeaderSize;
        bufferSize = tempBufferSize;
        size = headerSize + bufferSize;
    }

    // packed, run-length, differential compression

    encodeInteger<T, (sizeof(T) >= 8) ? 3 : 2, true, true, Analysis>(tempHeader.get(), tempHeaderSize, tempBuffer.get(), tempBufferSize, input, count);

    if (tempHeaderSize + tempBufferSize < size) {
        if (!Analysis) {
            header.swap(tempHeader);
            buffer.swap(tempBuffer);
        }

        newFlags = (pack8 ? PACK8 : PACK4) | RUNLENGTH | DIFFERENTIAL;

        headerSize = tempHeaderSize;
        bufferSize = tempBufferSize;
        size = headerSize + bufferSize;
    }

    os.write(reinterpret_cast<const char*>(&size), sizeof(size_t));

    if (Analysis)   return size;

    // write data

    os.write(reinterpret_cast<const char*>(&newFlags), sizeof(uint8_t));

    if (newFlags & RUNLENGTH) {
        os.write(reinterpret_cast<const char*>(&headerSize), sizeof(size_t));
    }

    os.write(reinterpret_cast<const char*>(header.get()), headerSize);

    if (newFlags == 0) {
        os.write(reinterpret_cast<const char*>(input), bufferSize);
    }
    else {
        os.write(reinterpret_cast<const char*>(buffer.get()), bufferSize);
    }

    return size;
}


template <typename T>
void readCompressedIntegers(std::istream& is, T* output, const size_t count)
{
    size_t size;
    is.read(reinterpret_cast<char*>(&size), sizeof(size_t));

    uint8_t flags(0);
    is.read(reinterpret_cast<char*>(&flags), sizeof(uint8_t));

    bool runLength = flags & RUNLENGTH;

    int bits(0);

    if (flags & PACK4)          bits = 2;
    else if (flags & PACK8)     bits = 3;

    size_t headerSize;
    if (runLength) {
        is.read(reinterpret_cast<char*>(&headerSize), sizeof(size_t));
    }
    else {
        headerSize = ((count * bits) + CHAR_BIT - 1) / CHAR_BIT;
    }

    const bool seek = output == NULL;

    // no compression

    if (bits == 0)
    {
        if (seek)   is.seekg(/*bytes=*/sizeof(T) * count, std::ios_base::cur);
        else        is.read(reinterpret_cast<char*>(output), sizeof(T) * count);
        return;
    }


    const bool differential = flags & DIFFERENTIAL;

    boost::scoped_array<char> header;
    if (headerSize > 0) {
        if (seek) {
            is.seekg(/*bytes=*/headerSize, std::ios_base::cur);
        }
        else {
            header.reset(new char[headerSize]);
            is.read(reinterpret_cast<char*>(header.get()), headerSize);
        }
    }

    size_t bufferSize = size - headerSize;

    boost::scoped_array<char> buffer;
    if (bufferSize > 0) {
        if (seek) {
            is.seekg(/*bytes=*/bufferSize, std::ios_base::cur);
        }
        else {
            buffer.reset(new char[bufferSize]);
            is.read(reinterpret_cast<char*>(buffer.get()), bufferSize);
        }
    }

    if (!seek)
    {
        if (runLength && differential)
        {
            if (bits == 2)          decodeInteger<T, 2, true, true>(output, count, header.get(), buffer.get());
            else if (bits == 3)     decodeInteger<T, 3, true, true>(output, count, header.get(), buffer.get());
        }
        else if (runLength)
        {
            if (bits == 2)          decodeInteger<T, 2, false, true>(output, count, header.get(), buffer.get());
            else if (bits == 3)     decodeInteger<T, 3, false, true>(output, count, header.get(), buffer.get());
        }
        else if (differential)
        {
            if (bits == 2)          decodeInteger<T, 2, true, false>(output, count, header.get(), buffer.get());
            else if (bits == 3)     decodeInteger<T, 3, true, false>(output, count, header.get(), buffer.get());
        }
        else
        {
            if (bits == 2)          decodeInteger<T, 2, false, false>(output, count, header.get(), buffer.get());
            else if (bits == 3)     decodeInteger<T, 3, false, false>(output, count, header.get(), buffer.get());
        }
    }
}


} // namespace io


////////////////////////////////////////


} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb


#endif // OPENVDB_TOOLS_COMPRESSION_HAS_BEEN_INCLUDED

// Copyright (c) 2015-2016 Double Negative Visual Effects
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
