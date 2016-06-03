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
/// @file VRAY_OpenVDB_Points.cc
///
/// @author Dan Bailey
///
/// @brief The Delayed Load Mantra Procedural for OpenVDB Points.


#include <UT/UT_DSOVersion.h>
#include <GU/GU_Detail.h>
#include <OP/OP_OperatorTable.h>
#include <UT/UT_BoundingBox.h>
#include <VRAY/VRAY_Procedural.h>

#include <openvdb/io/File.h>

#include <openvdb_points/openvdb.h>
#include <openvdb_points/tools/PointDataGrid.h>
#include <openvdb_points/tools/PointConversion.h>
#include <openvdb_points/tools/PointCount.h>

using namespace openvdb;
using namespace openvdb::tools;
using namespace openvdb::math;


/// Delay load OpenVDB Points and convert them into Houdini points
class VRAY_OpenVDB_Points : public VRAY_Procedural {
public:
	     VRAY_OpenVDB_Points();
    virtual ~VRAY_OpenVDB_Points();

    virtual const char	*className() const;

    virtual int		 initialize(const UT_BoundingBox *);
    virtual void	 getBoundingBox(UT_BoundingBox &box);
    virtual void	 render();

private:
    UT_BoundingBox	 myBox;
    UT_String        mFilename;
}; // class VRAY_OpenVDB_Points


////////////////////////////////////////


template <typename VDBType, typename AttrHandle>
inline void
setAttributeValue(const VDBType value, AttrHandle& handle, const GA_Offset& offset)
{
    handle.set(offset, value);
}


template <typename VDBElementType, typename AttrHandle>
inline void
setAttributeValue(const Vec3<VDBElementType>& v, AttrHandle& handle, const GA_Offset& offset)
{
    handle.set(offset, UT_Vector3(v.x(), v.y(), v.z()));
}


template<typename PointDataTreeType, typename LeafOffsetArray>
struct ConvertPointDataGridPositionOp {

    typedef typename PointDataTreeType::LeafNodeType    PointDataLeaf;
    typedef typename LeafOffsetArray::const_reference   LeafOffsetPair;

    ConvertPointDataGridPositionOp( GU_Detail& detail,
                                    const PointDataTreeType& tree,
                                    const math::Transform& transform,
                                    const size_t index,
                                    const LeafOffsetArray& leafOffsets)
        : mDetail(detail)
        , mTree(tree)
        , mTransform(transform)
        , mIndex(index)
        , mLeafOffsets(leafOffsets) { }

    void operator()(const tbb::blocked_range<size_t>& range) const {

        GA_RWHandleV3 pHandle(mDetail.getP());

        for (size_t n = range.begin(), N = range.end(); n != N; ++n) {

            assert(n < mLeafOffsets.size());

            // extract leaf and offset

            const LeafOffsetPair& leafOffset = mLeafOffsets[n];
            const PointDataLeaf& leaf = leafOffset.first;
            GA_Offset offset = leafOffset.second;

            AttributeHandle<Vec3f>::Ptr handle = AttributeHandle<Vec3f>::create(leaf.template attributeArray(mIndex));

            Vec3d uniformPos;

            const bool uniform = handle->isUniform();

            if (uniform)    uniformPos = handle->get(Index64(0));

            for (PointDataTree::LeafNodeType::ValueOnCIter iter = leaf.cbeginValueOn(); iter; ++iter) {

                Coord ijk = iter.getCoord();
                Vec3d xyz = ijk.asVec3d();

                IndexIter indexIter = leaf.beginIndex(ijk);
                for (; indexIter; ++indexIter) {

                    Vec3d pos = uniform ? uniformPos : Vec3d(handle->get(*indexIter));
                    pos = mTransform.indexToWorld(pos + xyz);
                    setAttributeValue(pos, pHandle, offset++);
                }
            }
        }
    }

    //////////

    GU_Detail&                              mDetail;
    const PointDataTreeType&                mTree;
    const math::Transform&                  mTransform;
    const size_t                            mIndex;
    const LeafOffsetArray&                  mLeafOffsets;
}; // ConvertPointDataGridPositionOp


template <typename LeafOffsetArray>
inline void
convertPointDataGridPosition(   GU_Detail& detail,
                                const PointDataGrid& grid,
                                const LeafOffsetArray& leafOffsets)
{
    const PointDataTree& tree = grid.tree();
    PointDataTree::LeafCIter iter = tree.cbeginLeaf();

    const size_t positionIndex = iter->attributeSet().find("P");

    // perform threaded conversion

    detail.getP()->hardenAllPages();
    ConvertPointDataGridPositionOp<PointDataTree, LeafOffsetArray> convert(
                    detail, tree, grid.transform(), positionIndex, leafOffsets);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, leafOffsets.size()), convert);
    detail.getP()->tryCompressAllPages();
}


////////////////////////////////////////


static VRAY_ProceduralArg	theArgs[] = {
    VRAY_ProceduralArg("file", "string", ""),
    VRAY_ProceduralArg()
};


VRAY_Procedural *
allocProcedural(const char *)
{
    return new VRAY_OpenVDB_Points();
}

const VRAY_ProceduralArg *
getProceduralArgs(const char *)
{
    return theArgs;
}

VRAY_OpenVDB_Points::VRAY_OpenVDB_Points()
{
    myBox.initBounds(0, 0, 0);
}

VRAY_OpenVDB_Points::~VRAY_OpenVDB_Points()
{
}

const char *
VRAY_OpenVDB_Points::className() const
{
    return "VRAY_OpenVDB_Points";
}

int
VRAY_OpenVDB_Points::initialize(const UT_BoundingBox *)
{
    import("file", mFilename);

    myBox.initBounds(-1, -1, -1);
    myBox.enlargeBounds(1, 1, 1);

    return 1;
}

void
VRAY_OpenVDB_Points::getBoundingBox(UT_BoundingBox &box)
{
    // Return the bounding box of the geometry
    box = myBox;
}

void
VRAY_OpenVDB_Points::render()
{
    openvdb::io::File file(mFilename.toStdString());

    /// Allocate geometry and extract the GU_Detail
    VRAY_ProceduralGeo	geo = createGeometry();
    GU_Detail* gdp = geo.get();

    // convert data

    typedef PointDataTree::LeafNodeType             PointDataLeaf;

    try
    {
        file.open();

        for (openvdb::io::File::NameIterator nameIter = file.beginName(); nameIter != file.endName(); ++nameIter)
        {
            openvdb::GridBase::Ptr baseGrid = file.readGrid(nameIter.gridName());
            if (!baseGrid->isType<PointDataGrid>())     continue;

            const PointDataGrid& grid = static_cast<const PointDataGrid&>(*baseGrid);
            const PointDataTree& tree = grid.tree();

            PointDataTree::LeafCIter leafIter = tree.cbeginLeaf();
            if (!leafIter) continue;

            // position attribute is mandatory

            const AttributeSet& attributeSet = leafIter->attributeSet();
            const AttributeSet::Descriptor& descriptor = attributeSet.descriptor();
            const bool hasPosition = descriptor.find("P") != AttributeSet::INVALID_POS;
            if (!hasPosition)   continue;

            // allocate Houdini point array

            gdp->appendPointBlock(pointCount(tree));

            // compute global point offsets for each leaf

            typedef std::pair<const PointDataLeaf&, Index64> LeafOffsetPair;
            typedef boost::ptr_vector<LeafOffsetPair> LeafOffsetArray;

            const Index64 leafCount = tree.leafCount();

            LeafOffsetArray leafOffsets;
            leafOffsets.reserve(leafCount);

            Index64 count = 0;
            for ( ; leafIter; ++leafIter) {
                leafOffsets.push_back(new LeafOffsetPair(*leafIter, count));
                count += leafIter->pointCount();
            }

            // convert the point positions

            convertPointDataGridPosition<LeafOffsetArray>(*gdp, grid, leafOffsets);
        }

        file.close();
    }
    catch (openvdb::Exception& e)
    {
        OPENVDB_LOG_ERROR(e.what() << " (" << mFilename << ")");
    }

    // Create a geometry object in mantra
    VRAY_ProceduralChildPtr	obj = createChild();
    obj->addGeometry(geo);

    // Override the renderpoints setting to always enable points only rendering
    obj->changeSetting("renderpoints", "true");
}


////////////////////////////////////////

// Copyright (c) 2015-2016 Double Negative Visual Effects
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
