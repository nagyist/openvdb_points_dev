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
/// @file AttributeArrayString.h
///
/// @authors Dan Bailey
///
/// @brief  Attribute array storage for string data using Descriptor Metadata.
///


#ifndef OPENVDB_TOOLS_ATTRIBUTE_ARRAY_STRING_HAS_BEEN_INCLUDED
#define OPENVDB_TOOLS_ATTRIBUTE_ARRAY_STRING_HAS_BEEN_INCLUDED

#include <openvdb_points/tools/AttributeArray.h>


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace tools {


////////////////////////////////////////


template <typename Codec_ = NullAttributeCodec<StringType> >
class StringTypedAttributeArray : public TypedAttributeArray<StringType, Codec_>
{
public:
    typedef StringType                  ValueType;
    typedef Codec_                      Codec;
    typedef typename Codec::StorageType StorageType;

    /// Default constructor, always constructs a uniform attribute.
    explicit StringTypedAttributeArray(  size_t n = 1,
                                    const ValueType& uniformValue = zeroVal<ValueType>())
        : TypedAttributeArray<StringType, Codec_>(n, uniformValue) { this->setString(true); }

    /// Deep copy constructor (optionally decompress during copy).
    StringTypedAttributeArray(const StringTypedAttributeArray& array,
                        const bool decompress = false)
        : TypedAttributeArray<StringType, Codec_>(array, decompress) { this->setString(true); }

}; // class StringTypedAttributeArray


// Convenience typedef when using with no codec
typedef StringTypedAttributeArray<>     StringAttributeArray;


////////////////////////////////////////


class StringAttributeHandle
{
public:
    typedef boost::shared_ptr<StringAttributeHandle> Ptr;

    static Ptr create(const AttributeArray& array, const MetaMap& metadata, const bool preserveCompression = true);

    StringAttributeHandle(  const AttributeArray& array,
                            const MetaMap& metadata,
                            const bool preserveCompression = true);

    size_t size() const { return mHandle.size(); }
    bool isUniform() const { return mHandle.isUniform(); }

    Name get(Index n) const;
    void get(Name& name, Index n) const;

protected:
    AttributeHandle<StringType>     mHandle;
    const MetaMap&                  mMetadata;
}; // class StringAttributeHandle


////////////////////////////////////////


class StringAttributeWriteHandle : public StringAttributeHandle
{
public:
    typedef boost::shared_ptr<StringAttributeWriteHandle> Ptr;

    static Ptr create(AttributeArray& array, const MetaMap& metadata);

    StringAttributeWriteHandle( AttributeArray& array,
                                const MetaMap& metadata);

    /// @brief  If this array is uniform, replace it with an array of length size().
    /// @param  fill if true, assign the uniform value to each element of the array.
    void expand(bool fill = true);

    /// @brief Set membership for the whole array and attempt to collapse
    ///
    /// @param name Name of the String
    void collapse();
    void collapse(const Name& name);

    /// Compact the existing array to become uniform if all values are identical
    bool compact();

    /// @brief Fill the existing array with the given value.
    /// @note Identical to collapse() except a non-uniform array will not become uniform.
    void fill(const Name& name);

    /// Set the value of the index to @param name
    void set(Index n, const Name& name);

    /// Reset the value cache from the metadata
    void resetCache();

private:
    /// Retrieve the index of this string value from the cache
    /// @note throws if name does not exist in cache
    Index getIndex(const Name& name);

    typedef std::map<std::string, Index>    ValueMap;

    ValueMap                                mCache;
    AttributeWriteHandle<StringType>        mWriteHandle;
}; // class StringAttributeWriteHandle


////////////////////////////////////////


} // namespace tools

} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb


#endif // OPENVDB_TOOLS_ATTRIBUTE_ARRAY_STRING_HAS_BEEN_INCLUDED


// Copyright (c) 2015-2016 Double Negative Visual Effects
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
