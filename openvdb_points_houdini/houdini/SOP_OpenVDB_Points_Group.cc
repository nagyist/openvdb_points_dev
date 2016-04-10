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
/// @file SOP_OpenVDB_Points_Group.cc
///
/// @author Dan Bailey
///
/// @brief Add and remove point groups.


#include <openvdb_points/openvdb.h>
#include <openvdb_points/tools/PointDataGrid.h>
#include <openvdb_points/tools/PointGroup.h>

#include "SOP_NodeVDBPoints.h"

#include <openvdb_houdini/Utils.h>
#include <houdini_utils/geometry.h>
#include <houdini_utils/ParmFactory.h>

using namespace openvdb;
using namespace openvdb::tools;
using namespace openvdb::math;

namespace hvdb = openvdb_houdini;
namespace hutil = houdini_utils;


namespace {

enum Mode {
    OP_ALL,
    OP_BBOX,
    OP_HASHI,
    OP_HASHL,
    OP_LEAF,
    OP_LS,
    OP_BBOX_HASHI,
    OP_BBOX_HASHL,
    OP_BBOX_LS,
    OP_BBOX_LEAF,
    OP_LS_HASHI,
    OP_LS_HASHL,
    OP_LS_LEAF,
    OP_BBOX_LS_HASHI,
    OP_BBOX_LS_HASHL,
    OP_BBOX_LS_LEAF
}; // Mode

struct GroupParms {

    GroupParms()
        : mGroup(NULL)
        , mEnable(false)
        // , mEnableNumber(false)
        // , mEnablePercent(false)
        // , mEnablePercentAttribute(false)
        // , mEnableBounding(false)
        // , mEnableBoundingObject(false)
        , mEnableViewport(false)
        , mPercent(0)
        , mLevelSetGrid(FloatGrid::create(0))
    {
    }

    Mode                                        mMode;
    const GA_PrimitiveGroup *                   mGroup;
    bool                                        mEnable;
    std::string                                 mGroupName;
    bool                                        mCountFilter;
    bool                                        mPercentAttributeFilter;
    bool                                        mPercentLeafFilter;
    bool                                        mBBoxFilter;
    bool                                        mLevelSetFilter;
    bool                                        mEnableLevelSet;
    bool                                        mEnableViewport;
    float                                       mPercent;
    long                                        mCount;
    std::string                                 mPercentAttribute;
    float                                       mSDFMin;
    float                                       mSDFMax;
    size_t                                      mPercentAttributeIndex;
    std::string                                 mViewportGroupName;

    openvdb::BBoxd                              mBBox;
    openvdb::FloatGrid::ConstPtr                mLevelSetGrid;
};

} // namespace


////////////////////////////////////////


class SOP_OpenVDB_Points_Group: public hvdb::SOP_NodeVDBPoints
{
public:
    SOP_OpenVDB_Points_Group(OP_Network*, const char* name, OP_Operator*);
    virtual ~SOP_OpenVDB_Points_Group() {}

    static OP_Node* factory(OP_Network*, const char* name, OP_Operator*);

    virtual int isRefInput(unsigned i) const { return (i > 0); }

    bool updateParmsFlags();

    OP_ERROR evalGroupParms(OP_Context&, GroupParms&);
    OP_ERROR evalGridGroupParms(const openvdb::tools::PointDataGrid& grid, OP_Context& context, GroupParms& parms);

    template <typename FilterT>
    void filter(PointDataTree& tree, const std::string& groupName, typename FilterT::Data& data);
    void createPointsGroup(PointDataGrid& outputGrid, const GroupParms& parms);
    void setViewportMetadata(PointDataGrid& outputGrid, const GroupParms& parms);

protected:
    virtual OP_ERROR cookMySop(OP_Context&);

private:
    hvdb::Interrupter mBoss;
}; // class SOP_OpenVDB_Points_Group



////////////////////////////////////////

inline int
lookupGroupInput(const PRM_SpareData *spare)
{
    const char  *istring;
    if (!spare) return 0;
    istring = spare->getValue("sop_input");
    return istring ? atoi(istring) : 0;
}

static PRM_Default negPointOneDefault(-0.1);
static PRM_Default fiveThousandDefault(5000);

// Build UI and register this operator.
void
newSopOperator(OP_OperatorTable* table)
{
    points::initialize();

    if (table == NULL) return;

    hutil::ParmList parms;

    parms.add(hutil::ParmFactory(PRM_STRING, "group", "Group")
        .setHelpText("Specify a subset of the input VDB grids to be loaded.")
        .setChoiceList(&hutil::PrimGroupMenuInput1));

    parms.add(hutil::ParmFactory(PRM_STRING, "vdbpointsgroup", "VDB Points Group")
        .setHelpText("Specify VDB Points Groups to use as an input."));

    parms.beginSwitcher("tabMenu1");
    parms.addFolder("Create");

    // Toggle to enable creation
    parms.add(hutil::ParmFactory(PRM_TOGGLE, "enablecreate", "Enable")
        .setDefault(PRMoneDefaults)
        .setHelpText("Enable creation of group."));

    parms.add(hutil::ParmFactory(PRM_STRING, "groupname", "Group Name")
        .setDefault(0, ::strdup("group1"))
        .setHelpText("Group name to create."));

    parms.beginSwitcher("tabMenu2");
    parms.addFolder("Number");

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "enablenumber", "Enable")
        .setDefault(PRMzeroDefaults)
        .setHelpText("Enable number filtering."));

    {
        const char* items[] = {
            "percentage",       "Percentage",
            "total",            "Total",
            NULL
        };
        parms.add(hutil::ParmFactory(PRM_ORD, "numbermode", "Mode")
            .setChoiceListItems(PRM_CHOICELIST_SINGLE, items));
    }

    parms.add(hutil::ParmFactory(PRM_FLT, "pointpercent", "Percent")
        .setDefault(PRMtenDefaults)
        .setRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_RESTRICTED, 100)
        .setHelpText("Point percentage to include in the Group."));

    parms.add(hutil::ParmFactory(PRM_INT, "pointcount", "Count")
        .setDefault(&fiveThousandDefault)
        .setRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1000000)
        .setHelpText("Point percentage to include in the Group."));

    parms.add(hutil::ParmFactory(PRM_TOGGLE_J, "enablepercentattribute", "")
        .setDefault(PRMzeroDefaults)
        .setTypeExtended(PRM_TYPE_TOGGLE_JOIN)
        .setHelpText("."));

    parms.add(hutil::ParmFactory(PRM_STRING, "percentattribute", "Attribute Seed")
        .setDefault(0, ::strdup("id"))
        .setHelpText("Point attribute to use as a seed for percent filtering."));

    parms.endSwitcher();

    parms.beginSwitcher("tabMenu3");
    parms.addFolder("Bounding");

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "enableboundingbox", "Enable")
        .setDefault(PRMzeroDefaults)
        .setHelpText("Enable bounding box filtering."));

    {
        const char* items[] = {
            "boundingbox",      "Bounding Box",
            "boundingobject",   "Bounding Object",
            NULL
        };
        parms.add(hutil::ParmFactory(PRM_ORD, "boundingmode", "Mode")
            .setChoiceListItems(PRM_CHOICELIST_SINGLE, items));
    }

    parms.add(hutil::ParmFactory(PRM_STRING, "boundingname", "Name")
        .setChoiceList(&hutil::PrimGroupMenuInput2)
        .setHelpText("Name of the bounding geometry."));

    parms.add(hutil::ParmFactory(PRM_XYZ, "size", "Size")
        .setDefault(PRMoneDefaults)
        .setVectorSize(3));

    parms.add(hutil::ParmFactory(PRM_XYZ, "center", "Center")
        .setVectorSize(3));

    parms.endSwitcher();

    parms.beginSwitcher("tabMenu4");
    parms.addFolder("Level Set");

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "enablelevelset", "Enable")
        .setDefault(PRMzeroDefaults)
        .setHelpText("Enable level set filtering."));

    parms.add(hutil::ParmFactory(PRM_STRING, "levelsetname", "Name")
        .setChoiceList(&hutil::PrimGroupMenuInput2)
        .setHelpText("Name of the level set."));

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "enablesdfmin", "Enable")
        .setDefault(PRMoneDefaults)
        .setTypeExtended(PRM_TYPE_TOGGLE_JOIN)
        .setHelpText("Enable SDF minimum."));

    parms.add(hutil::ParmFactory(PRM_FLT, "sdfmin", "SDF Minimum")
        .setDefault(&negPointOneDefault)
        .setRange(PRM_RANGE_UI, -1, PRM_RANGE_UI, 1)
        .setHelpText("SDF minimum value."));

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "enablesdfmax", "Enable")
        .setDefault(PRMoneDefaults)
        .setTypeExtended(PRM_TYPE_TOGGLE_JOIN)
        .setHelpText("Enable SDF maximum."));

    parms.add(hutil::ParmFactory(PRM_FLT, "sdfmax", "SDF Maximum")
        .setDefault(PRMpointOneDefaults)
        .setRange(PRM_RANGE_UI, -1, PRM_RANGE_UI, 1)
        .setHelpText("SDF maximum value."));

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "sdfinvert", "Invert")
        .setDefault(PRMzeroDefaults)
        .setHelpText("Invert SDF minimum and maximum."));

    parms.endSwitcher();

    parms.addFolder("Viewport");

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "enableviewport", "Enable")
        .setDefault(PRMzeroDefaults)
        .setHelpText("Toggle viewport group."));

    {
        const char* items[] = {
            "addviewportgroup",      "Add Viewport Group",
            "removeviewportgroup",   "Remove Viewport Group",
            NULL
        };
        parms.add(hutil::ParmFactory(PRM_ORD, "viewportoperation", "Operation")
            .setChoiceListItems(PRM_CHOICELIST_SINGLE, items));
    }

    parms.add(hutil::ParmFactory(PRM_STRING, "viewportgroupname", "Name")
        .setDefault(0, ::strdup("chs(\"groupname\")"), CH_OLD_EXPRESSION)
        .setHelpText("Display only this group in the viewport."));

    parms.endSwitcher();

    //////////
    // Register this operator.

    hvdb::OpenVDBOpFactory("OpenVDB Points Group",
        SOP_OpenVDB_Points_Group::factory, parms, *table)
        .addInput("VDB Points")
        .addOptionalInput("Optional bounding geometry or level set");
}


bool
SOP_OpenVDB_Points_Group::updateParmsFlags()
{
    bool changed = false;

    const bool creation = evalInt("enablecreate", 0, 0) != 0;
    const bool number = evalInt("enablenumber", 0, 0) != 0;
    const bool total = evalInt("numbermode", 0, 0) == 1;
    const bool percentattribute = evalInt("enablepercentattribute", 0, 0) != 0;
    const bool bounding = evalInt("enableboundingbox", 0, 0) != 0;
    const bool boundingobject = evalInt("boundingmode", 0, 0) == 1;
    const bool viewport = evalInt("enableviewport", 0, 0) != 0;
    const bool levelset = evalInt("enablelevelset", 0, 0);
    const bool sdfmin = evalInt("enablesdfmin", 0, 0);
    const bool sdfmax = evalInt("enablesdfmax", 0, 0);

    changed |= enableParm("groupname", creation);
    changed |= enableParm("enablenumber", creation);
    changed |= enableParm("numbermode", creation && number);
    changed |= enableParm("pointpercent", creation && number && !total);
    changed |= enableParm("pointcount", creation && number && total);
    changed |= enableParm("enablepercentattribute", creation && number && !total);
    changed |= enableParm("percentattribute", creation && number && percentattribute && !total);
    changed |= enableParm("enableboundingbox", creation);
    changed |= enableParm("boundingmode", creation && bounding);
    changed |= enableParm("boundingname", creation && bounding && boundingobject);
    changed |= enableParm("size", creation && bounding && !boundingobject);
    changed |= enableParm("center", creation && bounding && !boundingobject);
    changed |= enableParm("viewportoperation", viewport);
    changed |= enableParm("viewportgroupname", viewport && evalInt("viewportoperation", 0, 0) == 0);
    changed |= enableParm("levelsetname", levelset);
    changed |= enableParm("enablesdfmin", levelset);
    changed |= enableParm("enablesdfmax", levelset);
    changed |= enableParm("sdfmin", levelset && sdfmin);
    changed |= enableParm("sdfmax", levelset && sdfmax);
    changed |= enableParm("sdfinvert", levelset && sdfmin && sdfmax);

    return changed;
}


////////////////////////////////////////


OP_Node*
SOP_OpenVDB_Points_Group::factory(OP_Network* net,
    const char* name, OP_Operator* op)
{
    return new SOP_OpenVDB_Points_Group(net, name, op);
}


SOP_OpenVDB_Points_Group::SOP_OpenVDB_Points_Group(OP_Network* net,
    const char* name, OP_Operator* op)
    : hvdb::SOP_NodeVDBPoints(net, name, op)
{
}


////////////////////////////////////////


OP_ERROR
SOP_OpenVDB_Points_Group::cookMySop(OP_Context& context)
{
    typedef openvdb::tools::PointDataGrid PointDataGrid;

    try {
        hutil::ScopedInputLock lock(*this, context);

        // This does a shallow copy of VDB-grids and deep copy of native Houdini primitives.
        duplicateSourceStealable(0, context);

        // Evaluate UI parameters
        GroupParms parms;
        if (evalGroupParms(context, parms) >= UT_ERROR_ABORT) return error();

        UT_AutoInterrupt progress("Processing Points Group");

        hvdb::VdbPrimIterator vdbIt(gdp, parms.mGroup);

        for (; vdbIt; ++vdbIt) {

            if (progress.wasInterrupted()) {
                throw std::runtime_error("processing was interrupted");
            }

            GU_PrimVDB* vdbPrim = *vdbIt;
            openvdb::GridBase::Ptr inGrid = vdbPrim->getGridPtr();
            if (!inGrid->isType<PointDataGrid>()) continue;

            if (parms.mEnable) {
                // deep copy the VDB tree if it is not already unique
                vdbPrim->makeGridUnique();
            }

            PointDataGrid::Ptr outputGrid = openvdb::gridPtrCast<PointDataGrid>(inGrid);

            if (!outputGrid) {
                addError(SOP_MESSAGE, "Failed to duplicate VDB Points");
                return error();
            }

            if (evalGridGroupParms(*outputGrid, context, parms) >= UT_ERROR_ABORT) return error();

            // filter and create the point group in the grid

            if (parms.mEnable) {
                createPointsGroup(*outputGrid, parms);
            }

            // attach group viewport metadata to the grid

            if (parms.mEnableViewport) {
                setViewportMetadata(*outputGrid, parms);
            }
        }

        return error();

    } catch (std::exception& e) {
        addError(SOP_MESSAGE, e.what());
    }
    return error();
}


////////////////////////////////////////


OP_ERROR
SOP_OpenVDB_Points_Group::evalGroupParms(OP_Context& context, GroupParms& parms)
{
    const fpreal time = context.getTime();

    // evaluate filter mode

    const bool number = evalInt("enablenumber", 0, time);
    const bool countMode = evalInt("numbermode", 0, time) == 0;
    const bool percentAttribute = evalInt("enablepercentattribute", 0, time);
    const bool percentLeaf = !percentAttribute;
    const bool bounding = evalInt("enableboundingbox", 0, time);
    const bool boundingObject = evalInt("boundingmode", 0, time) == 1;
    const bool levelSet = evalInt("enablelevelset", 0, time);

    parms.mCountFilter = number && countMode;
    parms.mPercentAttributeFilter = number && !countMode && percentAttribute;
    parms.mPercentLeafFilter = number && !countMode && percentLeaf;
    parms.mBBoxFilter = bounding;
    parms.mLevelSetFilter = levelSet;

    std::cerr << "FILTER: " << parms.mCountFilter << " " << parms.mPercentAttributeFilter << " " << parms.mPercentLeafFilter << " " << parms.mBBoxFilter << " " << parms.mLevelSetFilter << std::endl;

    // Get the grids to group.
    UT_String groupStr;
    evalString(groupStr, "group", 0, time);
    parms.mGroup = matchGroup(*gdp, groupStr.toStdString());

    hvdb::VdbPrimIterator vdbIt(gdp, parms.mGroup);

    // Handle no vdbs
    if (!vdbIt) {
        addError(SOP_MESSAGE, "No VDBs found.");
        return error();
    }

    const GU_Detail* refGdp = inputGeo(1);

    // group creation

    parms.mEnable = evalInt("enablecreate", 0, time);

    UT_String groupNameStr;
    evalString(groupNameStr, "groupname", 0, time);
    std::string groupName = groupNameStr.toStdString();

    if (groupName == "") {
        addWarning(SOP_MESSAGE, "Cannot create a group with an empty name, changing to _");
        groupName = "_";
    }

    parms.mGroupName = groupName;

    // number

    if (number) {
        parms.mPercent = evalFloat("pointpercent", 0, time);
        parms.mCount = evalInt("pointcount", 0, time);

        UT_String percentAttributeStr;
        evalString(percentAttributeStr, "percentattribute", 0, time);
        parms.mPercentAttribute = percentAttributeStr.toStdString();
    }

    // bounds

    if (bounding) {
        if (boundingObject) {
            if (!refGdp) {
                addError(SOP_MESSAGE, "Missing second input");
                return error();
            }

            // retrieve bounding object

            const GA_PrimitiveGroup* boundsGroup = NULL;
            UT_String boundingObjectStr;
            evalString(boundingObjectStr, "boundingname", 0, time);
    #if (UT_MAJOR_VERSION_INT >= 15)
            boundsGroup = parsePrimitiveGroups(
                boundingObjectStr.buffer(), GroupCreator(refGdp));
    #else
            boundsGroup = parsePrimitiveGroups(
                boundingObjectStr.buffer(), const_cast<GU_Detail*>(refGdp));
    #endif

            // compute bounds of bounding object

            UT_BoundingBox box;
            box.initBounds();
            if (boundsGroup) {
                GA_Range range = refGdp->getPrimitiveRange(boundsGroup);
                refGdp->enlargeBoundingBox(box, range);
            }
            else {
                refGdp->computeQuickBounds(box);
            }

            parms.mBBox.min()[0] = box.xmin();
            parms.mBBox.min()[1] = box.ymin();
            parms.mBBox.min()[2] = box.zmin();
            parms.mBBox.max()[0] = box.xmax();
            parms.mBBox.max()[1] = box.ymax();
            parms.mBBox.max()[2] = box.zmax();
        }
        else {
            // store bounding box

            openvdb::BBoxd::ValueType size(evalFloat("size", 0, time), evalFloat("size", 1, time), evalFloat("size", 2, time));
            openvdb::BBoxd::ValueType center(evalFloat("center", 0, time), evalFloat("center", 1, time), evalFloat("center", 2, time));
            parms.mBBox = openvdb::BBoxd(center - size/2, center + size/2);
        }
    }

    // level set

    if (levelSet)
    {
        if (!refGdp) {
            addError(SOP_MESSAGE, "Missing second input");
            return error();
        }

        // retrieve level set grid

        UT_String levelSetStr;
        const GA_PrimitiveGroup* levelSetGroup = NULL;
        evalString(levelSetStr, "levelsetname", 0, time);
#if (UT_MAJOR_VERSION_INT >= 15)
        levelSetGroup = parsePrimitiveGroups(
            levelSetStr.buffer(), GroupCreator(refGdp));
#else
        levelSetGroup = parsePrimitiveGroups(
            levelSetStr.buffer(), const_cast<GU_Detail*>(refGdp));
#endif
        for (hvdb::VdbPrimCIterator vdbRefIt(refGdp, levelSetGroup); vdbRefIt; ++vdbRefIt) {
            if (vdbRefIt->getStorageType() == UT_VDB_FLOAT &&
                vdbRefIt->getGrid().getGridClass() == openvdb::GRID_LEVEL_SET) {
                parms.mLevelSetGrid = gridConstPtrCast<FloatGrid>((*vdbRefIt)->getConstGridPtr());
                break;
            }
        }
        if (!parms.mLevelSetGrid) {
            addError(SOP_MESSAGE, "Second input has no float VDB level set");
            return error();
        }

        bool enableSDFMin = evalInt("enablesdfmin", 0, time);
        bool enableSDFMax = evalInt("enablesdfmax", 0, time);

        float sdfMin = enableSDFMin ? evalFloat("sdfmin", 0, time) : -std::numeric_limits<float>::max();
        float sdfMax = enableSDFMax ? evalFloat("sdfmax", 0, time) : std::numeric_limits<float>::max();

        // check level set min and max values

        if ((enableSDFMin || enableSDFMax) && sdfMin > sdfMax) {
            addWarning(SOP_MESSAGE, "SDF minimum is greater than SDF maximum, suggest using the invert toggle instead");
        }
        const float background = parms.mLevelSetGrid->background();
        if (enableSDFMin && sdfMin < -background) {
            addWarning(SOP_MESSAGE, "SDF minimum value is less than the background value of the level set");
        }
        if (enableSDFMax && sdfMax > background) {
            addWarning(SOP_MESSAGE, "SDF maximum value is greater than the background value of the level set");
        }

        const bool sdfInvert = evalInt("sdfinvert", 0, time);
        parms.mSDFMin = sdfInvert ? -sdfMin : sdfMin;
        parms.mSDFMax = sdfInvert ? -sdfMax : sdfMax;
    }

    // viewport

    parms.mEnableViewport = evalInt("enableviewport", 0, time);

    UT_String viewportGroupNameStr;
    evalString(viewportGroupNameStr, "viewportgroupname", 0, time);
    std::string viewportGroupName = viewportGroupNameStr.toStdString();
    if (viewportGroupName == "") {
        addWarning(SOP_MESSAGE, "Cannot create a viewport group with an empty name, changing to _");
        viewportGroupName = "_";
    }

    parms.mViewportGroupName = groupName;

    return error();
}


////////////////////////////////////////


OP_ERROR
SOP_OpenVDB_Points_Group::evalGridGroupParms(const PointDataGrid& grid, OP_Context& context, GroupParms& parms)
{
    // number

    bool hashI = false;
    bool hashL = false;

    if (parms.mPercentAttributeFilter)
    {
        // retrieve percent attribute type (if it exists)

        openvdb::tools::PointDataTree::LeafCIter leafIter = grid.tree().cbeginLeaf();
        if (leafIter) {
            const AttributeSet::Descriptor& descriptor = leafIter->attributeSet().descriptor();

            const size_t index = descriptor.find(parms.mPercentAttribute);

            if (index == AttributeSet::INVALID_POS) {
                addError(SOP_MESSAGE, ("Unable to find attribute - " + parms.mPercentAttribute).c_str());
                return error();
            }

            parms.mPercentAttributeIndex = index;
            const std::string attributeType = descriptor.valueType(index);

            if (attributeType == "int32")       hashI = true;
            else if (attributeType == "int64")  hashL = true;
            else {
                addError(SOP_MESSAGE, ("Unsupported attribute type for percent attribute filtering - " + attributeType).c_str());
                return error();
            }
        }
    }

    std::cerr << "Hash: " << hashI << " " << hashL << std::endl;

    // define filter mode

    if (parms.mBBoxFilter && parms.mLevelSetFilter && hashI)                            parms.mMode = OP_BBOX_LS_HASHI;
    else if (parms.mBBoxFilter && parms.mLevelSetFilter && hashL)                       parms.mMode = OP_BBOX_LS_HASHL;
    else if (parms.mBBoxFilter && parms.mLevelSetFilter && parms.mPercentLeafFilter)    parms.mMode = OP_BBOX_LS_LEAF;
    else if (parms.mBBoxFilter && hashI)                                                parms.mMode = OP_BBOX_HASHI;
    else if (parms.mBBoxFilter && hashL)                                                parms.mMode = OP_BBOX_HASHL;
    else if (parms.mBBoxFilter && parms.mPercentLeafFilter)                             parms.mMode = OP_BBOX_LEAF;
    else if (parms.mBBoxFilter && parms.mLevelSetFilter)                                parms.mMode = OP_BBOX_LS;
    else if (parms.mLevelSetFilter && hashI)                                            parms.mMode = OP_LS_HASHI;
    else if (parms.mLevelSetFilter && hashL)                                            parms.mMode = OP_LS_HASHL;
    else if (parms.mLevelSetFilter && parms.mPercentLeafFilter)                         parms.mMode = OP_LS_LEAF;
    else if (parms.mBBoxFilter)                                                         parms.mMode = OP_BBOX;
    else if (parms.mLevelSetFilter)                                                     parms.mMode = OP_LS;
    else if (hashI)                                                                     parms.mMode = OP_HASHI;
    else if (hashL)                                                                     parms.mMode = OP_HASHL;
    else if (parms.mPercentLeafFilter)                                                  parms.mMode = OP_LEAF;

    if (parms.mMode == OP_LEAF) std::cerr << "LEAF!" << std::endl;

    return error();
}


////////////////////////////////////////


template <typename FilterT>
void
SOP_OpenVDB_Points_Group::filter(PointDataTree& tree, const std::string& groupName, typename FilterT::Data& data)
{
    setGroupByFilter<PointDataTree, FilterT>(tree, groupName, data);
}


void
SOP_OpenVDB_Points_Group::createPointsGroup(PointDataGrid& outputGrid, const GroupParms& parms)
{
    // filter typedefs

    typedef AttributeHashFilter<boost::mt11213b, int> HashIFilter;
    typedef AttributeHashFilter<boost::mt11213b, long> HashLFilter;
    typedef RandomLeafFilter<boost::mt11213b> LeafFilter;
    typedef LevelSetFilter<FloatGrid> LSFilter;

    // composite filter typedefs

    typedef BinaryFilter<BBoxFilter, HashIFilter> BBoxHashIFilter;
    typedef BinaryFilter<BBoxFilter, HashLFilter> BBoxHashLFilter;
    typedef BinaryFilter<BBoxFilter, LSFilter> BBoxLSFilter;
    typedef BinaryFilter<BBoxFilter, LeafFilter> BBoxLeafFilter;
    typedef BinaryFilter<LSFilter, HashIFilter> LSHashIFilter;
    typedef BinaryFilter<LSFilter, HashLFilter> LSHashLFilter;
    typedef BinaryFilter<LSFilter, LeafFilter> LSLeafFilter;

    typedef BinaryFilter<BBoxFilter, LSHashIFilter> BBoxLSHashIFilter;
    typedef BinaryFilter<BBoxFilter, LSHashLFilter> BBoxLSHashLFilter;
    typedef BinaryFilter<BBoxFilter, LSLeafFilter> BBoxLSLeafFilter;

    // grid data

    PointDataTree& tree = outputGrid.tree();
    openvdb::math::Transform& transform = outputGrid.transform();
    const std::string groupName = parms.mGroupName;

    // build filter data

    BBoxFilter::Data bboxFilter(transform, parms.mBBox);
    HashIFilter::Data hashIFilter(parms.mPercentAttributeIndex, parms.mPercent);
    HashLFilter::Data hashLFilter(parms.mPercentAttributeIndex, parms.mPercent);
    LeafFilter::Data leafFilter;
    LSFilter::Data lsFilter(*parms.mLevelSetGrid, transform, parms.mSDFMin, parms.mSDFMax);

    if (parms.mCountFilter)             leafFilter.populateByTargetPoints<PointDataTree>(tree, parms.mCount);
    else if (parms.mPercentLeafFilter)  leafFilter.populateByPercentagePoints<PointDataTree>(tree, parms.mPercent);

    // build composite filter data

    BBoxHashIFilter::Data bboxHashIFilter(bboxFilter, hashIFilter);
    BBoxHashLFilter::Data bboxHashLFilter(bboxFilter, hashLFilter);
    BBoxLSFilter::Data bboxLSFilter(bboxFilter, lsFilter);
    BBoxLeafFilter::Data bboxLeafFilter(bboxFilter, leafFilter);
    LSHashIFilter::Data lsHashIFilter(lsFilter, hashIFilter);
    LSHashLFilter::Data lsHashLFilter(lsFilter, hashLFilter);
    LSLeafFilter::Data lsLeafFilter(lsFilter, leafFilter);

    BBoxLSHashIFilter::Data bboxLSHashIFilter(bboxFilter, lsHashIFilter);
    BBoxLSHashLFilter::Data bboxLSHashLFilter(bboxFilter, lsHashLFilter);
    BBoxLSLeafFilter::Data bboxLSLeafFilter(bboxFilter, lsLeafFilter);

    // append the group

    appendGroup(tree, groupName);

    // perform group filtering

    std::cerr << "Mode: " << parms.mMode << std::endl;

    switch (parms.mMode)
    {
        case OP_BBOX:           filter<BBoxFilter>(tree, groupName, bboxFilter); break;
        case OP_HASHI:          filter<HashIFilter>(tree, groupName, hashIFilter); break;
        case OP_HASHL:          filter<HashLFilter>(tree, groupName, hashLFilter); break;
        case OP_LS:             filter<LSFilter>(tree, groupName, lsFilter); break;
        case OP_LEAF:           filter<LeafFilter>(tree, groupName, leafFilter); break;
        case OP_BBOX_HASHI:     filter<BBoxHashIFilter>(tree, groupName, bboxHashIFilter); break;
        case OP_BBOX_HASHL:     filter<BBoxHashLFilter>(tree, groupName, bboxHashLFilter); break;
        case OP_BBOX_LS:        filter<BBoxLSFilter>(tree, groupName, bboxLSFilter); break;
        case OP_BBOX_LEAF:      filter<BBoxLeafFilter>(tree, groupName, bboxLeafFilter); break;
        case OP_LS_HASHI:       filter<LSHashIFilter>(tree, groupName, lsHashIFilter); break;
        case OP_LS_HASHL:       filter<LSHashLFilter>(tree, groupName, lsHashLFilter); break;
        case OP_LS_LEAF:        filter<LSLeafFilter>(tree, groupName, lsLeafFilter); break;
        case OP_BBOX_LS_HASHI:  filter<BBoxLSHashIFilter>(tree, groupName, bboxLSHashIFilter); break;
        case OP_BBOX_LS_HASHL:  filter<BBoxLSHashLFilter>(tree, groupName, bboxLSHashLFilter); break;
        case OP_BBOX_LS_LEAF:   filter<BBoxLSLeafFilter>(tree, groupName, bboxLSLeafFilter); break;
        default:                setGroup<PointDataTree>(tree, groupName); break;
    }
}


////////////////////////////////////////


void
SOP_OpenVDB_Points_Group::setViewportMetadata(PointDataGrid& outputGrid, const GroupParms& parms)
{
    outputGrid.insertMeta(openvdb::META_GROUP_VIEWPORT, StringMetadata(parms.mViewportGroupName));
}


////////////////////////////////////////

// Copyright (c) 2015-2016 Double Negative Visual Effects
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
