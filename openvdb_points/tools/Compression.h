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
///
/// @brief  Array compression schemes
///

#ifndef OPENVDB_TOOLS_COMPRESSION_HAS_BEEN_INCLUDED
#define OPENVDB_TOOLS_COMPRESSION_HAS_BEEN_INCLUDED


#include <boost/static_assert.hpp>
#include <boost/scoped_array.hpp>

#include <cstring> // std::memcpy
#include <climits> // CHAR_BIT
#include <iostream>
#include <sstream>
#include <memory> // std::unique_ptr
#include <type_traits> // std::enable_if

#include <openvdb/version.h>
#include <openvdb/util/logging.h> // OPENVDB_LOG_DEBUG
#include <openvdb/Exceptions.h> // OPENVDB_THROW

#ifdef OPENVDB_USE_BLOSC
#include <blosc.h>
#endif


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace io {


enum INTEGER_COMPRESSION
{
    PACK4 = 0x1,
    PACK8 = 0x2,
    DIFFERENTIAL = 0x4,
    RUNLENGTH = 0x8
};


/// @brief Returns true if compression is available
inline bool canCompress()
{
#ifdef OPENVDB_USE_BLOSC
    return true;
#else
    OPENVDB_LOG_DEBUG("Can't compress array data without the blosc library.");
    return false;
#endif
}


////////////////////////////////////////


// Attribute Compression methods


/// @brief Retrieves the compressed size of buffer when compressed
///
/// @param buffer the uncompressed buffer
/// @param typeSize the size of the data type
/// @param uncompressedBytes number of uncompressed bytes
template <typename T>
size_t bloscCompressSize(const std::unique_ptr<T[]>& buffer, const size_t count, const size_t typeSize = sizeof(T))
{
#ifndef OPENVDB_USE_BLOSC
    OPENVDB_LOG_DEBUG("Can't compress array data without the blosc library.");
    return 0;
#else
    size_t uncompressedBytes = sizeof(T) * count;
    size_t tempBytes = uncompressedBytes + BLOSC_MAX_OVERHEAD;
    const bool outOfRange = tempBytes > BLOSC_MAX_BUFFERSIZE;
    const std::unique_ptr<char[]> outBuf(outOfRange ? new char[1] : new char[tempBytes]);

    int compressedBytes = blosc_compress_ctx(
        /*clevel=*/9, // 0 (no compression) to 9 (maximum compression)
        /*doshuffle=*/true,
        /*typesize=*/typeSize,
        /*srcsize=*/uncompressedBytes,
        /*src=*/buffer.get(),
        /*dest=*/outBuf.get(),
        /*destsize=*/tempBytes,
        BLOSC_LZ4_COMPNAME,
        /*blocksize=*/256,
        /*numthreads=*/1);

    if (compressedBytes <= 0) {
        std::ostringstream ostr;
        ostr << "Blosc failed to compress " << uncompressedBytes << " byte" << (uncompressedBytes == 1 ? "" : "s");
        if (compressedBytes < 0) ostr << " (internal error " << compressedBytes << ")";
        OPENVDB_LOG_DEBUG(ostr.str());
        return 0;
    }

    return size_t(compressedBytes);
#endif
}

/// @brief Compress and return the compressed buffer.
///
/// @param buffer the buffer to compress
/// @param typeSize the size of the data type
/// @param uncompressedBytes number of uncompressed bytes
/// @param compressedBytes number of compressed bytes (written to this variable)
/// @param cleanup if true, the supplied buffer will be deleted prior to allocating new memory
template <typename T>
std::unique_ptr<T[]> bloscToBuffer(  const std::unique_ptr<T[]>& buffer, const size_t count, size_t& compressedBytes, const size_t typeSize = sizeof(T))
{
#ifndef OPENVDB_USE_BLOSC
    OPENVDB_LOG_DEBUG("Can't compress array data without the blosc library.");
    return nullptr;
#else
    size_t uncompressedBytes = sizeof(T) * count;
    size_t tempBytes = uncompressedBytes + BLOSC_MAX_OVERHEAD;
    const bool outOfRange = tempBytes > BLOSC_MAX_BUFFERSIZE;
    const std::unique_ptr<char[]> outBuf(outOfRange ? new char[1] : new char[tempBytes]);

    int _compressedBytes = blosc_compress_ctx(
        /*clevel=*/9, // 0 (no compression) to 9 (maximum compression)
        /*doshuffle=*/true,
        /*typesize=*/typeSize,
        /*srcsize=*/uncompressedBytes,
        /*src=*/reinterpret_cast<char*>(buffer.get()),
        /*dest=*/outBuf.get(),
        /*destsize=*/tempBytes,
        BLOSC_LZ4_COMPNAME,
        /*blocksize=*/256,
        /*numthreads=*/1);

    if (_compressedBytes <= 0) {
        std::ostringstream ostr;
        ostr << "Blosc failed to compress " << uncompressedBytes << " byte" << (uncompressedBytes == 1 ? "" : "s");
        if (_compressedBytes < 0) ostr << " (internal error " << _compressedBytes << ")";
        OPENVDB_LOG_DEBUG(ostr.str());
        return nullptr;
    }

    compressedBytes = size_t(_compressedBytes);

    char* outData = new char[compressedBytes];
    std::memcpy(outData, outBuf.get(), compressedBytes);

    return std::unique_ptr<T[]>(reinterpret_cast<T*>(outData));
#endif
}

/// @brief Decompress and return the uncompressed buffer.
///
/// @param buffer the buffer to decompress
/// @param expectedBytes the number of bytes expected once the buffer is decompressed
/// @param cleanup if true, the supplied buffer will be deleted prior to allocating new memory
template <typename T>
inline std::unique_ptr<T[]> bloscFromBuffer(const std::unique_ptr<T[]>& buffer, const size_t count)
{
#ifndef OPENVDB_USE_BLOSC
    OPENVDB_THROW(RuntimeError, "Can't extract compressed data without the blosc library.");
#else
    size_t expectedBytes = sizeof(T) * count;
    size_t tempBytes = sizeof(T) * count + BLOSC_MAX_OVERHEAD;
    const bool outOfRange = tempBytes > BLOSC_MAX_BUFFERSIZE;
    if (outOfRange)     tempBytes = 1;
    const std::unique_ptr<char[]> tempBuffer(new char[tempBytes]);

    const int _uncompressedBytes = blosc_decompress_ctx(    /*src=*/reinterpret_cast<const char*>(buffer.get()),
                                                            /*dest=*/tempBuffer.get(),
                                                            tempBytes,
                                                            /*numthreads=*/1);

    if (_uncompressedBytes < 1) {
        OPENVDB_LOG_DEBUG("blosc_decompress() returned error code " << _uncompressedBytes);
        return nullptr;
    }

    size_t uncompressedBytes = size_t(_uncompressedBytes);

    if (uncompressedBytes != expectedBytes) {
        OPENVDB_THROW(openvdb::RuntimeError, "Expected to decompress " << expectedBytes
            << " byte" << (expectedBytes == 1 ? "" : "s") << ", got "
            << uncompressedBytes << " byte" << (uncompressedBytes == 1 ? "" : "s"));
    }

    std::unique_ptr<T[]> newBuffer(new T[count]);
    std::memcpy(reinterpret_cast<char*>(newBuffer.get()), tempBuffer.get(), uncompressedBytes);

    return newBuffer;
#endif
}


template <typename T>
size_t compressedSize(const std::unique_ptr<T[]>& buffer, const size_t count)
{
    return bloscCompressSize(buffer, count);
}


template <typename T>
std::unique_ptr<T[]> compress(const std::unique_ptr<T[]>& buffer, const size_t count, size_t& compressedBytes)
{
    return bloscToBuffer(buffer, count, compressedBytes);
}


template <typename T>
std::unique_ptr<T[]> decompress(const std::unique_ptr<T[]>& buffer, const size_t count)
{
    return bloscFromBuffer(buffer, count);
}


////////////////////////////////////////


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


////////////////////////////////////////


template <typename IntT, int Bits, bool Differential, bool RunLength, bool Analysis=false>
void encodeInteger( char* headerBuffer, size_t& headerSize,
                    char* data, size_t& dataSize,
                    const IntT* input, const size_t count)
{
    BOOST_STATIC_ASSERT(Bits == 1 || Bits == 2 || Bits == 3);
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
                if (++key >= (1UL << Bits) - 1)   break;
            }
            return key;
        }

        static int clampKey(uint8_t key)
        {
            if (key >= (1UL << Bits) - 1)   return sizeof(IntT);
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

    if (RunLength && count < 2) {
        OPENVDB_THROW(openvdb::RuntimeError, "Cannot use run-length encoding with fewer than two integers.");
    }

    BitWriter<Analysis ? true : false> header(headerBuffer);
    BufferWriter<Analysis ? true : false> bufferWriter(data);

    int repeat(0);

    UIntT unsignedValue;
    if (std::is_unsigned<IntT>())   unsignedValue = input[0];
    else                            unsignedValue = Local::rotateLeftShift(input[0]);

    // write key

    UIntT value = unsignedValue;
    uint8_t key = Local::computeKey(value);

    bool checkUniform = RunLength;

    for (size_t i = 1; i < count; i++)
    {
        const UIntT signedValue = Differential ? input[i] - input[i-1] : input[i];
        if (std::is_unsigned<IntT>())   unsignedValue = signedValue;
        else                            unsignedValue = Local::rotateLeftShift(signedValue);

        // write repeat

        if (RunLength && !(Differential && i == 1))
        {
            if (value == unsignedValue) {
                repeat++;
                continue;
            }

            if (checkUniform) {
                header.template push<1>(0);
                checkUniform = false;
            }

            for (int j = 0; j < repeat; j++)    header.template push<1>(1);
            header.template push<1>(0);

            repeat = 0;
        }

        bufferWriter.template write(value, Local::clampKey(key));

        header.template push<Bits>(key);

        value = unsignedValue;
        key = Local::computeKey(value);
    }

    // write run-length bits (ones followed by terminating zero)

    if (RunLength)
    {
        // write a one to indicate all values are the same to compact the header
        if (checkUniform) {
            header.template push<1>(1);
        }
        else {
            for (int j = 0; j < repeat; j++)   header.template push<1>(1);
            header.template push<1>(0);
        }
    }

    bufferWriter.template write(value, Local::clampKey(key));

    header.template push<Bits>(key);
    header.template finalize();

    headerSize = header.template bytes();
    dataSize = bufferWriter.bytes();
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

    if (RunLength && count < 2) {
        OPENVDB_THROW(openvdb::RuntimeError, "Cannot decode run-length encoded integer stream \
                                                with fewer than two integers.");
    }

    BitReader header(headerBuffer);
    BufferReader buffer(data);

    int index(0);

    if (Differential) {
        // read key

        const uint8_t key = header.pop<Bits>();

        // read data

        const UIntT unsignedValue = buffer.read<UIntT>(Local::clampKey(key));
        const IntT signedValue = std::is_unsigned<IntT>() ? unsignedValue :
                                                            Local::rotateRightShift(unsignedValue);

        output[index] = signedValue;
        index++;
    }

    bool uniform = false;
    if (RunLength) {
        uniform = bool(header.pop<1>());
    }

    while (index < count)
    {
        // extract counter

        int repeat = 1;
        if (RunLength) {
            if (uniform) {
                repeat = count;
                if (Differential)   repeat--;
            }
            else {
                while (header.pop<1>())     repeat++;
            }
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
            if (Differential)      output[index] += output[index-1];
            index++;
        }
    }
}


////////////////////////////////////////


template<typename T> struct is_compressible_integral {
    static const bool value = std::is_integral<T>::value;
};

template<> struct is_compressible_integral<bool> {
    static const bool value = false;
};

template <typename T, bool Analysis>
typename std::enable_if<!is_compressible_integral<T>::value,std::unique_ptr<char[]>>::type
writeCompressedIntegers(std::ostream&, const T*, const size_t, size_t&)
{
    OPENVDB_THROW(openvdb::RuntimeError, "Invalid Type for Integer Compression.");
}

template <typename T, bool Analysis>
typename std::enable_if<is_compressible_integral<T>::value,std::unique_ptr<char[]>>::type
writeCompressedIntegers(const std::unique_ptr<T[]>& input, const size_t count, size_t& size)
{
    const bool pack8 = sizeof(T) >= 8;

    // allocate temporary buffers

    std::unique_ptr<char[]> buffer;
    std::unique_ptr<char[]> header;

    std::unique_ptr<char[]> tempBuffer;
    std::unique_ptr<char[]> tempHeader;

    const size_t prefix = count + sizeof(size_t) + sizeof(uint8_t);

    if (!Analysis) {
        buffer.reset(new char[count*sizeof(T)+prefix]);
        header.reset(new char[count]);

        tempBuffer.reset(new char[count*sizeof(T)+prefix]);
        tempHeader.reset(new char[count]);
    }

    size_t tempHeaderSize;
    size_t tempBufferSize;

    // uncompressed

    uint8_t newFlags(0);

    size_t headerSize(0);
    size_t bufferSize(sizeof(T) * count);
    size = headerSize + bufferSize;

    // packed compression

    encodeInteger<T, (sizeof(T) >= 8) ? 3 : 2, false, false, Analysis>(tempHeader.get(), tempHeaderSize, tempBuffer.get()+prefix, tempBufferSize, input.get(), count);

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

    encodeInteger<T, (sizeof(T) >= 8) ? 3 : 2, true, false, Analysis>(tempHeader.get(), tempHeaderSize, tempBuffer.get()+prefix, tempBufferSize, input.get(), count);

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

    // run-length encoding not possible with fewer than two integers

    if (count > 1)
    {
        // packed, run-length compression

        encodeInteger<T, (sizeof(T) >= 8) ? 3 : 2, false, true, Analysis>(tempHeader.get(), tempHeaderSize, tempBuffer.get()+prefix, tempBufferSize, input.get(), count);

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

        encodeInteger<T, (sizeof(T) >= 8) ? 3 : 2, true, true, Analysis>(tempHeader.get(), tempHeaderSize, tempBuffer.get()+prefix, tempBufferSize, input.get(), count);

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
    }

    size_t flagSize = sizeof(uint8_t);
    size_t runLengthSize = (newFlags & RUNLENGTH) ? sizeof(size_t) : size_t(0);

    const size_t memSize = bufferSize+headerSize+runLengthSize+flagSize;

    size = memSize;

    if (Analysis)   return nullptr;

    std::memcpy(buffer.get()+prefix-headerSize-runLengthSize-flagSize, reinterpret_cast<const char*>(&newFlags), flagSize);

    if (newFlags & RUNLENGTH) {
        std::memcpy(buffer.get()+prefix-headerSize-runLengthSize, reinterpret_cast<const char*>(&headerSize), sizeof(size_t));
    }

    std::memcpy(buffer.get()+prefix-headerSize, reinterpret_cast<const char*>(header.get()), headerSize);

    if (newFlags == 0) {
        std::memcpy(buffer.get()+prefix, reinterpret_cast<const char*>(input.get()), bufferSize);
    }

    std::unique_ptr<char[]> newBuffer(new char[memSize]);
    std::memcpy(reinterpret_cast<char*>(newBuffer.get()), buffer.get()+prefix-headerSize-runLengthSize-flagSize, memSize);

    return newBuffer;
}


template <typename T>
typename std::enable_if<!is_compressible_integral<T>::value,std::unique_ptr<T[]>>::type
readCompressedIntegers(const std::unique_ptr<char[]>& input, const size_t count, const size_t size)
{
    OPENVDB_THROW(openvdb::RuntimeError, "Invalid Type for Integer Compression.");
}


template <typename T>
typename std::enable_if<is_compressible_integral<T>::value,std::unique_ptr<T[]>>::type
readCompressedIntegers(const std::unique_ptr<char[]>& input, const size_t count, const size_t size)
{
    std::unique_ptr<T[]> output(new T[count]);

    size_t counter = 0;

    size_t flagSize = sizeof(uint8_t);

    uint8_t flags(0);
    std::memcpy(&flags, input.get(), flagSize);

    bool runLength = flags & RUNLENGTH;

    int bits(0);

    if (flags & PACK4)          bits = 2;
    else if (flags & PACK8)     bits = 3;

    size_t runLengthSize = 0;

    size_t headerSize;
    if (runLength) {
        runLengthSize = sizeof(size_t);
        std::memcpy(&headerSize, input.get()+flagSize, runLengthSize);
    }
    else {
        headerSize = ((count * bits) + CHAR_BIT - 1) / CHAR_BIT;
    }

    // no compression

    if (bits == 0)
    {
        std::memcpy(reinterpret_cast<char*>(output.get()), input.get()+flagSize+runLengthSize, sizeof(T) * count);
        return output;
    }

    const bool differential = flags & DIFFERENTIAL;

    std::unique_ptr<char[]> header;
    if (headerSize > 0) {
        header.reset(new char[headerSize]);
        std::memcpy(reinterpret_cast<char*>(header.get()), input.get()+flagSize+runLengthSize, headerSize);
    }

    size_t bufferSize = size - headerSize;

    std::unique_ptr<char[]> buffer;
    if (bufferSize > 0) {
        buffer.reset(new char[bufferSize]);
        std::memcpy(reinterpret_cast<char*>(buffer.get()), input.get()+flagSize+runLengthSize+headerSize, bufferSize);
    }

    if (runLength && differential)
    {
        if (bits == 2)          decodeInteger<T, 2, true, true>(output.get(), count, header.get(), buffer.get());
        else if (bits == 3)     decodeInteger<T, 3, true, true>(output.get(), count, header.get(), buffer.get());
    }
    else if (runLength)
    {
        if (bits == 2)          decodeInteger<T, 2, false, true>(output.get(), count, header.get(), buffer.get());
        else if (bits == 3)     decodeInteger<T, 3, false, true>(output.get(), count, header.get(), buffer.get());
    }
    else if (differential)
    {
        if (bits == 2)          decodeInteger<T, 2, true, false>(output.get(), count, header.get(), buffer.get());
        else if (bits == 3)     decodeInteger<T, 3, true, false>(output.get(), count, header.get(), buffer.get());
    }
    else
    {
        if (bits == 2)          decodeInteger<T, 2, false, false>(output.get(), count, header.get(), buffer.get());
        else if (bits == 3)     decodeInteger<T, 3, false, false>(output.get(), count, header.get(), buffer.get());
    }

    return output;
}


////////////////////////////////////////


} // namespace io
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb


#endif // OPENVDB_TOOLS_COMPRESSION_HAS_BEEN_INCLUDED

// Copyright (c) 2015-2016 Double Negative Visual Effects
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
