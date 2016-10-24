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
/// @file ArrayCompression.h
///
/// @authors Dan Bailey
///
/// @brief  Array compression schemes
///

#ifndef OPENVDB_TOOLS_ARRAY_COMPRESSION_HAS_BEEN_INCLUDED
#define OPENVDB_TOOLS_ARRAY_COMPRESSION_HAS_BEEN_INCLUDED


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

#include <openvdb_points/Types.h>

#ifdef OPENVDB_USE_BLOSC
#include <blosc.h>
#endif


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace tools {


enum INTEGER_COMPRESSION
{
    PACK4 = 0x1,
    PACK8 = 0x2,
    DIFFERENTIAL = 0x4,
    RUNLENGTH = 0x8,
    BLOSC = 0x10,
    ANY = PACK4 | PACK8 | DIFFERENTIAL | RUNLENGTH | BLOSC
};


////////////////////////////////////////


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


template<typename T> struct is_compressible_integral {
    static const bool value = std::is_integral<T>::value;
};

template<> struct is_compressible_integral<bool> {
    static const bool value = false;
};

template<> struct is_compressible_integral<uint8_t> {
    static const bool value = false;
};


template <typename IntT, int Bits, bool Differential, bool RunLength, bool Analysis=false>
typename std::enable_if<!is_compressible_integral<IntT>::value,void>::type
encodeInteger(char*, size_t&, char*, size_t&, const IntT*, const size_t)
{
    OPENVDB_THROW(openvdb::RuntimeError, "Invalid Type for Integer Compression.");
}

template <typename IntT, int Bits, bool Differential, bool RunLength, bool Analysis=false>
typename std::enable_if<is_compressible_integral<IntT>::value,void>::type
encodeInteger( char* headerBuffer, size_t& headerSize,
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
typename std::enable_if<!is_compressible_integral<IntT>::value,void>::type
decodeInteger(IntT*, const size_t, const char*, const char*)
{
    OPENVDB_THROW(openvdb::RuntimeError, "Invalid Type for Integer Compression.");
}

template <typename IntT, int Bits, bool Differential, bool RunLength>
typename std::enable_if<is_compressible_integral<IntT>::value,void>::type
decodeInteger(IntT* output, const size_t count, const char* headerBuffer, const char* data)
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


template <typename T, bool Analysis>
std::unique_ptr<T[]> writeCompressedIntegers(const std::unique_ptr<T[]>& input, const size_t count,
                                             const uint8_t compression, size_t& size)
{
    // no compression options enabled

    if (compression == 0) {
        size = 0;
        return nullptr;
    }

    const bool pack8 = sizeof(T) >= 8;
    const bool pack = ((compression & PACK8) && pack8) || ((compression & PACK4) && !pack8);

    // allocate temporary buffers

    std::unique_ptr<char[]> buffer;
    std::unique_ptr<char[]> header;

    std::unique_ptr<char[]> tempBuffer;
    std::unique_ptr<char[]> tempHeader;

    size_t flagSize = sizeof(uint8_t);
    size_t sizeSize = sizeof(size_t);

    // space is also allocated before the buffer for variable-length header data
    // which is populated to avoid having to copy the data

    const size_t prefix = count + flagSize + sizeSize + sizeSize;

    if (!Analysis && pack) {
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

    char* start = buffer.get()+prefix;

    if (pack) {
        // packed compression

        if (compression & DIFFERENTIAL) {
            encodeInteger<T, (sizeof(T) >= 8) ? 3 : 2, false, false, Analysis>(tempHeader.get(), tempHeaderSize, tempBuffer.get()+prefix, tempBufferSize, input.get(), count);

            if (tempHeaderSize + tempBufferSize < size) {
                if (!Analysis) {
                    header.swap(tempHeader);
                    buffer.swap(tempBuffer);
                }

                newFlags = pack8 ? PACK8 : PACK4;

                headerSize = tempHeaderSize;
                bufferSize = tempBufferSize;
                size = headerSize + bufferSize + sizeSize;
                start = buffer.get()+prefix-headerSize-sizeSize;
            }
        }

        // packed, differential compression

        if (compression & DIFFERENTIAL) {
            encodeInteger<T, (sizeof(T) >= 8) ? 3 : 2, true, false, Analysis>(tempHeader.get(), tempHeaderSize, tempBuffer.get()+prefix, tempBufferSize, input.get(), count);

            if (tempHeaderSize + tempBufferSize < size) {
                if (!Analysis) {
                    header.swap(tempHeader);
                    buffer.swap(tempBuffer);
                }

                newFlags = (pack8 ? PACK8 : PACK4) | DIFFERENTIAL;

                headerSize = tempHeaderSize;
                bufferSize = tempBufferSize;
                size = headerSize + bufferSize + sizeSize;
                start = buffer.get()+prefix-headerSize-sizeSize;
            }
        }

        // run-length encoding not performed with fewer than two integers

        if (count > 1)
        {
            if (compression & RUNLENGTH) {
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
                    size = headerSize + bufferSize + sizeSize + sizeSize;
                    start = buffer.get()+prefix-headerSize-sizeSize-sizeSize;
                }
            }

            if (compression & (RUNLENGTH | DIFFERENTIAL)) {
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
                    size = headerSize + bufferSize + sizeSize + sizeSize;
                    start = buffer.get()+prefix-headerSize-sizeSize-sizeSize;
                }
            }
        }
    }

    if (Analysis) {
        size += flagSize;
        return nullptr;
    }

    // free up temporary buffers

    tempHeader.reset();
    tempBuffer.reset();

    if (newFlags == 0) {
        header.reset();
        buffer.reset();
    }

    // prepend header and run-length metadata to buffer

    const char* headerStart = start;

    if (newFlags & RUNLENGTH) {
        std::memcpy(start, reinterpret_cast<const char*>(&headerSize), sizeSize);
        start += sizeSize;
    }

    if (newFlags != 0) {
        std::memcpy(start, reinterpret_cast<const char*>(&bufferSize), sizeSize);
        start += sizeSize;
        std::memcpy(start, reinterpret_cast<const char*>(header.get()), headerSize);
        start += headerSize;
    }

    bool useBlosc = (compression & BLOSC);
    size_t tempBytes = size + BLOSC_MAX_OVERHEAD;
    int bloscBytes = 0;
    std::unique_ptr<char[]> bloscBuffer;
    if (useBlosc && tempBytes < BLOSC_MAX_BUFFERSIZE) {
        bloscBuffer.reset(new char[tempBytes]);

        const char* bloscStart = newFlags == 0 ? reinterpret_cast<char*>(input.get()) : headerStart;
        bloscBytes = blosc_compress_ctx(/*clevel=*/9, // 0 (no compression) to 9 (maximum compression)
                                        /*doshuffle=*/true,
                                        /*typesize=*/sizeof(T),
                                        /*srcsize=*/size,
                                        /*src=*/bloscStart,
                                        /*dest=*/bloscBuffer.get(),
                                        /*destsize=*/tempBytes,
                                        BLOSC_LZ4_COMPNAME,
                                        /*blocksize=*/256,
                                        /*numthreads=*/1);
    }

    char* outData = nullptr;

    if (bloscBytes > 0 && bloscBytes < size) {
        newFlags |= BLOSC;
        outData = new char[bloscBytes+flagSize];
        std::memcpy(outData, reinterpret_cast<const char*>(&newFlags), flagSize);
        std::memcpy(outData+flagSize, bloscBuffer.get(), bloscBytes);
    }
    else {
        // free up blosc buffer
        bloscBuffer.reset();

        if (newFlags == 0) {
            size = 0;
            return nullptr;
        }

        outData = new char[size+flagSize];
        std::memcpy(outData, reinterpret_cast<const char*>(&newFlags), flagSize);
        if (newFlags == 0) {
            std::memcpy(outData+flagSize, reinterpret_cast<const char*>(input.get()), bufferSize);
        }
        else {
            std::memcpy(outData+flagSize, reinterpret_cast<const char*>(headerStart), size);
        }
    }

    size += flagSize;
    return std::unique_ptr<T[]>(reinterpret_cast<T*>(outData));
}


template <typename T>
std::unique_ptr<T[]> readCompressedIntegers(const std::unique_ptr<T[]>& input, const size_t count)
{
    const char* start = reinterpret_cast<const char*>(input.get());

    size_t flagSize = sizeof(uint8_t);
    size_t sizeSize = sizeof(size_t);

    uint8_t flags(0);
    std::memcpy(&flags, start, flagSize);
    start += flagSize;

    std::unique_ptr<char[]> bloscBuffer;
    size_t bloscSize(0);
    if (flags & BLOSC) {
        size_t _1, _2;
        blosc_cbuffer_sizes(start, &bloscSize, &_1, &_2);
        bloscSize += BLOSC_MAX_OVERHEAD;
        bloscBuffer.reset(new char[bloscSize + BLOSC_MAX_OVERHEAD]);

        blosc_decompress_ctx(   /*src=*/start,
                                /*dest=*/bloscBuffer.get(),
                                bloscSize,
                                /*numthreads=*/1);

        start = bloscBuffer.get();

        flags &= ~BLOSC;
    }

    size_t counter = 0;

    bool runLength = flags & RUNLENGTH;

    int bits(0);

    if (flags & PACK4)          bits = 2;
    else if (flags & PACK8)     bits = 3;

    size_t headerSize;
    if (runLength) {
        size_t sizeSize = sizeof(size_t);
        std::memcpy(&headerSize, start, sizeSize);
        start += sizeSize;
    }
    else {
        headerSize = ((count * bits) + CHAR_BIT - 1) / CHAR_BIT;
    }

    size_t bufferSize;
    if (flags != 0) {
        size_t sizeSize = sizeof(size_t);
        std::memcpy(&bufferSize, start, sizeSize);
        start += sizeSize;
    }
    else {
        bufferSize = count * sizeof(T);
    }

    std::unique_ptr<T[]> output(new T[count]);

    // no compression

    if (bits == 0)
    {
        std::memcpy(reinterpret_cast<char*>(output.get()), start, bufferSize);
        return output;
    }

    const char* header = nullptr;
    if (headerSize > 0) {
        header = start;
        start += headerSize;
    }

    const char* buffer = nullptr;
    if (bufferSize > 0) {
        buffer = start;
    }

    const bool differential = flags & DIFFERENTIAL;

    if (runLength && differential)
    {
        if (bits == 2)          decodeInteger<T, 2, true, true>(output.get(), count, header, buffer);
        else if (bits == 3)     decodeInteger<T, 3, true, true>(output.get(), count, header, buffer);
    }
    else if (runLength)
    {
        if (bits == 2)          decodeInteger<T, 2, false, true>(output.get(), count, header, buffer);
        else if (bits == 3)     decodeInteger<T, 3, false, true>(output.get(), count, header, buffer);
    }
    else if (differential)
    {
        if (bits == 2)          decodeInteger<T, 2, true, false>(output.get(), count, header, buffer);
        else if (bits == 3)     decodeInteger<T, 3, true, false>(output.get(), count, header, buffer);
    }
    else
    {
        if (bits == 2)          decodeInteger<T, 2, false, false>(output.get(), count, header, buffer);
        else if (bits == 3)     decodeInteger<T, 3, false, false>(output.get(), count, header, buffer);
    }

    return output;
}


////////////////////////////////////////


template <typename T>
size_t compressedArraySize(const std::unique_ptr<T[]>& buffer, const size_t count, uint8_t compression = ANY)
{
    // disable integer compression options if applicable
    if (!is_compressible_integral<T>::value) {
        compression &= (~PACK4 & ~PACK8 & ~DIFFERENTIAL & ~RUNLENGTH);
        return bloscCompressSize(buffer, count);
    }

    size_t compressedBytes;
    writeCompressedIntegers<T, /*Analysis=*/true>(buffer, count, compression, compressedBytes);
    return compressedBytes;
}


template <typename T>
std::unique_ptr<T[]> compressArray(const std::unique_ptr<T[]>& buffer, const size_t count,
                              size_t& compressedBytes, uint8_t compression = ANY)
{
    // disable integer compression options if bool or floating-point
    if (!is_compressible_integral<T>::value) {
        compression &= (~PACK4 & ~PACK8 & ~DIFFERENTIAL & ~RUNLENGTH);

        return bloscToBuffer(buffer, count, compressedBytes);
    }

    return writeCompressedIntegers<T, /*Analysis=*/false>(buffer, count, compression, compressedBytes);
}


template <typename T>
std::unique_ptr<T[]> decompressArray(const std::unique_ptr<T[]>& buffer, const size_t count)
{
    if (!is_compressible_integral<T>::value) {
        return bloscFromBuffer(buffer, count);
    }

    return readCompressedIntegers(buffer, count);
}


////////////////////////////////////////


} // namespace tools
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb


#endif // OPENVDB_TOOLS_ARRAY_COMPRESSION_HAS_BEEN_INCLUDED

// Copyright (c) 2015-2016 Double Negative Visual Effects
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
