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
/// @file IndexFilter.h
///
/// @authors Dan Bailey
///
/// @brief  Index filters primarily designed to be used with a FilterIndexIter.
///


#ifndef OPENVDB_TOOLS_INDEX_FILTER_HAS_BEEN_INCLUDED
#define OPENVDB_TOOLS_INDEX_FILTER_HAS_BEEN_INCLUDED

#include <openvdb/version.h>
#include <openvdb/Types.h>

#include <openvdb/math/Transform.h>

#include <openvdb_points/tools/IndexIterator.h>
#include <openvdb_points/tools/AttributeArray.h>

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace tools {


////////////////////////////////////////


// Random index filtering per leaf
template <typename RandGenT>
class RandomLeafFilter
{
public:
    typedef std::map<openvdb::Coord, Index64> LeafSeedMap;
    typedef boost::uniform_01<RandGenT> Distribution;
    typedef typename Distribution::result_type ResultT;

    struct Data
    {
        Data(const ResultT _factor, const LeafSeedMap& _leafSeedMap)
            : factor(_factor), leafSeedMap(_leafSeedMap) { }
        const ResultT factor;
        const LeafSeedMap& leafSeedMap;
    };

    RandomLeafFilter(const Data& data, const unsigned int seed)
        : mData(data)
        , mDistribution(RandGenT(seed)) { }

    inline ResultT next() const {
        return const_cast<boost::uniform_01<boost::mt11213b>&>(mDistribution)();
    }

    template <typename LeafT>
    static RandomLeafFilter create(const LeafT& leaf, const Data& data) {
        const LeafSeedMap::const_iterator it = data.leafSeedMap.find(leaf.origin());
        if (it == data.leafSeedMap.end()) {
            OPENVDB_THROW(openvdb::KeyError, "Cannot find leaf origin in offset map for random filter");
        }
        return RandomLeafFilter(data, (unsigned int) it->second);
    }

    template <typename IterT>
    bool valid(const IterT&) const {
        return next() < mData.factor;
    }

private:
    const Data mData;
    Distribution mDistribution;
}; // class RandomLeafFilter


struct RandomFilterByAttributeData
{
    RandomFilterByAttributeData(const float _factor, const Name& _attribute)
        : factor(_factor), attribute(_attribute) { }
    const float factor;
    const Name attribute;
};


// Random Threshold Filter using Point Attribute
template <typename SeedT>
class RandomFilterByAttribute
{
public:
    typedef RandomFilterByAttributeData Data;

    RandomFilterByAttribute(   const Data& data,
                    const typename AttributeHandle<SeedT>::Ptr& randomSeedHandle)
        : mData(data)
        , mSeedHandle(randomSeedHandle) { }

    template <typename LeafT>
    static RandomFilterByAttribute create(const LeafT& leaf, const Data& data) {
        return RandomFilterByAttribute(data, AttributeHandle<SeedT>::create(leaf.constAttributeArray(data.attribute)));
    }

    template <typename IterT>
    bool valid(const IterT& iter) const {
        if (mData.factor >= 1.0f)       return true;
        else if (mData.factor <= 0.0f)  return false;

        const SeedT id = mSeedHandle->get(*iter);
        boost::mt19937 generator((long)id);
        boost::uniform_01<boost::mt19937> dist(generator);
        return float(dist()) < mData.factor;
    }

private:
    const Data mData;
    const typename AttributeHandle<SeedT>::Ptr mSeedHandle;
}; // class RandomFilterByAttribute


// BBox index filtering
class BBoxFilter
{
public:
    struct Data
    {
        Data(const openvdb::math::Transform& _transform,
             const openvdb::BBoxd& _bboxWS)
            : transform(_transform)
            , bbox(transform.worldToIndex(_bboxWS)) { }
        const openvdb::math::Transform transform;
        const openvdb::BBoxd bbox;
    };

    BBoxFilter( const Data& data,
                const AttributeHandle<openvdb::Vec3f>::Ptr& positionHandle)
        : mData(data)
        , mPositionHandle(positionHandle) { }

    template <typename LeafT>
    static BBoxFilter create(const LeafT& leaf, const Data& data) {
        return BBoxFilter(data, AttributeHandle<openvdb::Vec3f>::create(leaf.constAttributeArray("P")));
    }

    template <typename IterT>
    bool valid(const IterT& iter) const {
        const openvdb::Coord ijk = iter.getCoord();
        const openvdb::Vec3f voxelIndexSpace = ijk.asVec3d();

        // Retrieve point position in voxel space
        const openvdb::Vec3f& pointVoxelSpace = mPositionHandle->get(*iter);

        // Compute point position in index space
        const openvdb::Vec3f pointIndexSpace = pointVoxelSpace + voxelIndexSpace;

        return mData.bbox.isInside(pointIndexSpace);
    }

private:
    const Data mData;
    const AttributeHandle<openvdb::Vec3f>::Ptr mPositionHandle;
}; // class BBoxFilter


enum BinaryFilterOp
{
    And = 0,
    Or
};


// Index filtering based on evaluating both sub-filters
template <typename T1, typename T2, BinaryFilterOp Op>
class BinaryFilter
{
public:
    typedef BinaryFilter<T1, T2, Op> FilterT;

    struct Data
    {
        Data(const typename T1::Data& _filterData1,
             const typename T2::Data& _filterData2)
            : filterData1(_filterData1)
            , filterData2(_filterData2) { }
        typename T1::Data filterData1;
        typename T2::Data filterData2;
    };

    BinaryFilter(  const T1& filter1,
                const T2& filter2)
        : mFilter1(filter1)
        , mFilter2(filter2) { }

    template <typename LeafT>
    static FilterT create(const LeafT& leaf, const Data& data) {
        return FilterT( T1::create(leaf, data.filterData1),
                        T2::create(leaf, data.filterData2));
    }

    template <typename IterT>
    bool valid(const IterT& iter) const {
        if (Op == And)      return mFilter1.valid(iter) && mFilter2.valid(iter);
        else if (Op == Or)  return mFilter1.valid(iter) || mFilter2.valid(iter);
        return false;
    }

private:
    const T1 mFilter1;
    const T2 mFilter2;
}; // class BinaryFilter


////////////////////////////////////////


template<typename T>
struct FilterTraits {
    static const bool RequiresCoord = true;
};
template<>
struct FilterTraits<BBoxFilter> {
    static const bool RequiresCoord = true;
};
template <typename T0, typename T1, BinaryFilterOp Op>
struct FilterTraits<BinaryFilter<T0, T1, Op> > {
    static const bool RequiresCoord =   FilterTraits<T0>::RequiresCoord ||
                                        FilterTraits<T1>::RequiresCoord;
};


////////////////////////////////////////


} // namespace tools
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb


#endif // OPENVDB_TOOLS_INDEX_FILTER_HAS_BEEN_INCLUDED


// Copyright (c) 2015-2016 Double Negative Visual Effects
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
