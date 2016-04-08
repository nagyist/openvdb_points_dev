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


////////////////////////////////////////


class SOP_OpenVDB_Points_Group: public hvdb::SOP_NodeVDBPoints
{
public:
    SOP_OpenVDB_Points_Group(OP_Network*, const char* name, OP_Operator*);
    virtual ~SOP_OpenVDB_Points_Group() {}

    static OP_Node* factory(OP_Network*, const char* name, OP_Operator*);

    bool updateParmsFlags();

protected:
    virtual OP_ERROR cookMySop(OP_Context&);

private:
    hvdb::Interrupter mBoss;

    bool mSecondInputConnected;
}; // class SOP_OpenVDB_Points_Group



////////////////////////////////////////


// Build UI and register this operator.
void
newSopOperator(OP_OperatorTable* table)
{
    points::initialize();

    if (table == NULL) return;

    hutil::ParmList parms;

    parms.add(hutil::ParmFactory(PRM_STRING, "group", "Source Group")
        .setHelpText("Specify a subset of the input VDB grids to be loaded.")
        .setChoiceList(&hutil::PrimGroupMenuInput1));

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

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "enablepercent", "Enable")
        .setDefault(PRMzeroDefaults)
        .setHelpText("Enable percent filtering."));

    parms.add(hutil::ParmFactory(PRM_FLT, "pointpercent", "Point Percent")
        .setDefault(PRMtenDefaults)
        .setRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_RESTRICTED, 100)
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
        parms.add(hutil::ParmFactory(PRM_ORD, "boundingtype", "Bounding Type")
            .setChoiceListItems(PRM_CHOICELIST_SINGLE, items));
    }

    parms.add(hutil::ParmFactory(PRM_XYZ, "size", "Size")
        .setDefault(PRMoneDefaults)
        .setVectorSize(3));

    parms.add(hutil::ParmFactory(PRM_XYZ, "center", "Center")
        .setVectorSize(3));

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
        .addOptionalInput("Optional bounding geometry");
}


bool
SOP_OpenVDB_Points_Group::updateParmsFlags()
{
    bool changed = false;

    const bool creation = evalInt("enablecreate", 0, 0) != 0;
    const bool percent = evalInt("enablepercent", 0, 0) != 0;
    const bool percentattribute = evalInt("enablepercentattribute", 0, 0) != 0;
    const bool bounding = evalInt("enableboundingbox", 0, 0) != 0;
    const bool viewport = evalInt("enableviewport", 0, 0) != 0;

    changed |= enableParm("groupname", creation);
    changed |= enableParm("enablepercent", creation);
    changed |= enableParm("pointpercent", creation && percent);
    changed |= enableParm("enablepercentattribute", creation && percent);
    changed |= enableParm("percentattribute", creation && percent && percentattribute);
    changed |= enableParm("enableboundingbox", creation);
    changed |= enableParm("boundingtype", creation && bounding);
    changed |= enableParm("size", creation && bounding && evalInt("boundingtype", 0, 0) == 0);
    changed |= enableParm("center", creation && bounding && evalInt("boundingtype", 0, 0) == 0);
    changed |= enableParm("viewportoperation", viewport);
    changed |= enableParm("viewportgroupname", viewport && evalInt("viewportoperation", 0, 0) == 0);

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
    , mSecondInputConnected(false)
{
}


template <typename T>
void setGroupByRandomFilter(PointDataTree& tree, const Name& groupName,
                            const typename RandomFilterByAttribute<T>::Data& randomFilterData)
{
    setGroupByFilter<PointDataTree, RandomFilterByAttribute<T> >(tree, groupName, randomFilterData);
}


template <typename T>
void setGroupByRandomBBoxFilter(PointDataTree& tree, const Name& groupName,
                                const typename RandomFilterByAttribute<T>::Data& randomFilterData, const BBoxFilter::Data& bboxFilterData)
{
    typedef BinaryFilter<RandomFilterByAttribute<T>, BBoxFilter, And> Filter;
    setGroupByFilter<PointDataTree, Filter>(tree, groupName, typename Filter::Data(randomFilterData, bboxFilterData));
}


////////////////////////////////////////


OP_ERROR
SOP_OpenVDB_Points_Group::cookMySop(OP_Context& context)
{
    typedef openvdb::tools::PointDataGrid PointDataGrid;

    try {
        hutil::ScopedInputLock lock(*this, context);
        const fpreal time = context.getTime();

        // This does a shallow copy of VDB-grids and deep copy of native Houdini primitives.
        duplicateSourceStealable(0, context);

        UT_AutoInterrupt progress("Processing Points Group");

        const GU_Detail* refGdp = inputGeo(1);
        mSecondInputConnected = refGdp != NULL;

        // Get the group of grids to group.
        UT_String groupStr;
        evalString(groupStr, "group", 0, time);
        const GA_PrimitiveGroup* group = matchGroup(*gdp, groupStr.toStdString());

        hvdb::VdbPrimIterator vdbIt(gdp, group);

        const bool createGroup = evalInt("enablecreate", 0, time);
        const float pointPercent = evalFloat("pointpercent", 0, time);
        const bool enablePercent = evalInt("enablepercent", 0, time) && pointPercent < 100.0f;
        const bool enableBounding = evalInt("enableboundingbox", 0, time);
        const bool enableViewport = evalInt("enableviewport", 0, time);
        const bool enablePercentAttribute = evalInt("enablepercentattribute", 0, time);
        UT_String percentAttributeStr;
        evalString(percentAttributeStr, "percentattribute", 0, time);
        const std::string percentAttribute = percentAttributeStr.toStdString();

        // Handle no vdbs
        if (!vdbIt) {
            addError(SOP_MESSAGE, "No VDBs found.");
            return error();
        }

        for (; vdbIt; ++vdbIt) {

            if (progress.wasInterrupted()) {
                throw std::runtime_error("processing was interrupted");
            }

            GU_PrimVDB* vdbPrim = *vdbIt;
            openvdb::GridBase::Ptr inGrid = vdbPrim->getGridPtr();

            if (!inGrid->isType<PointDataGrid>()) continue;
            PointDataGrid::Ptr pointDataGrid = openvdb::gridPtrCast<PointDataGrid>(inGrid);

            openvdb::tools::PointDataTree::LeafCIter leafIter = pointDataGrid->tree().cbeginLeaf();
            if (!leafIter)  continue;

            const AttributeSet::Descriptor& descriptor = leafIter->attributeSet().descriptor();

            if (createGroup) {
                // deep copy the VDB tree if it is not already unique
                vdbPrim->makeGridUnique();
            }

            PointDataGrid::Ptr outputGrid = openvdb::gridPtrCast<PointDataGrid>(vdbPrim->getGridPtr());

            if (!outputGrid) {
                addError(SOP_MESSAGE, "Failed to duplicate VDB Points");
                return error();
            }

            // create the group

            if (createGroup) {
                UT_String groupNameStr;
                evalString(groupNameStr, "groupname", 0, time);
                std::string groupName = groupNameStr.toStdString();
                if (groupName == "") {
                    addWarning(SOP_MESSAGE, "Cannot create a group with an empty name, changing to _");
                    groupName = "_";
                }

                PointDataTree& tree = outputGrid->tree();

                appendGroup(tree, groupName);

                // Create Filter Data

                openvdb::BBoxd::ValueType size(evalFloat("size", 0, time), evalFloat("size", 1, time), evalFloat("size", 2, time));
                openvdb::BBoxd::ValueType center(evalFloat("center", 0, time), evalFloat("center", 1, time), evalFloat("center", 2, time));
                const openvdb::BBoxd bbox(center - size/2, center + size/2);

                typedef RandomLeafFilter<boost::mt11213b> RandomFilterByNumber;

                RandomFilterByNumber::LeafSeedMap leafSeeds;

                // create leaf offsets if an attribute has not been selected for the seed

                if (enablePercent && !enablePercentAttribute) {
                    leafIter = pointDataGrid->tree().cbeginLeaf();

                    Index64 size = 0;

                    for ( ; leafIter; ++leafIter) {
                        leafSeeds[leafIter->origin()] = size;
                        size += leafIter->attributeSet().size();
                    }
                }

                RandomFilterByAttributeData randomFilterData(pointPercent / 100.0f, percentAttribute);
                RandomFilterByNumber::Data randomFilterNumberData(pointPercent / 100.0f, leafSeeds);
                BBoxFilter::Data bboxFilterData(outputGrid->transform(), bbox);

                Name valueType;

                // retrieve percent attribute type (if it exists)

                if (enablePercentAttribute) {
                    const std::string attributeType = "";
                    const size_t index = descriptor.find(percentAttribute);

                    if (index == AttributeSet::INVALID_POS) {
                        addError(SOP_MESSAGE, ("Unable to find attribute - " + percentAttribute).c_str());
                        return error();
                    }

                    valueType = descriptor.valueType(index);
                }

                // Perform group filtering

                if (enablePercent) {
                    if (enablePercentAttribute) {
                        if (enableBounding) {
                            if (valueType == "float") {
                                setGroupByRandomBBoxFilter<float>(tree, groupName, randomFilterData, bboxFilterData);
                            }
                            else if (valueType == "double") {
                                setGroupByRandomBBoxFilter<double>(tree, groupName, randomFilterData, bboxFilterData);
                            }
                            else if (valueType == "int") {
                                setGroupByRandomBBoxFilter<int>(tree, groupName, randomFilterData, bboxFilterData);
                            }
                            else if (valueType == "long") {
                                setGroupByRandomBBoxFilter<long>(tree, groupName, randomFilterData, bboxFilterData);
                            }
                            else {
                                addError(SOP_MESSAGE, ("Unsupported attribute type for random seed - " + valueType).c_str());
                                return error();
                            }
                        }
                        else {
                            if (valueType == "float") {
                                setGroupByRandomFilter<float>(tree, groupName, randomFilterData);
                            }
                            else if (valueType == "double") {
                                setGroupByRandomFilter<double>(tree, groupName, randomFilterData);
                            }
                            else if (valueType == "int") {
                                setGroupByRandomFilter<int>(tree, groupName, randomFilterData);
                            }
                            else if (valueType == "long") {
                                setGroupByRandomFilter<long>(tree, groupName, randomFilterData);
                            }
                            else {
                                addError(SOP_MESSAGE, ("Unsupported attribute type for random seed - " + valueType).c_str());
                                return error();
                            }
                        }
                    }
                    else {
                        if (enableBounding) {
                            typedef BinaryFilter<RandomFilterByNumber, BBoxFilter, And> Filter;
                            setGroupByFilter<PointDataTree, Filter>(tree, groupName, Filter::Data(randomFilterNumberData, bboxFilterData));
                        }
                        else {
                            setGroupByFilter<PointDataTree, RandomFilterByNumber>(tree, groupName, randomFilterNumberData);
                        }
                    }
                }
                else if (enableBounding) {
                    setGroupByFilter<PointDataTree, BBoxFilter>(tree, groupName, bboxFilterData);
                }
                else {
                    setGroup<PointDataTree>(tree, groupName);
                }
            }

            // set the viewport metadata

            if (enableViewport) {
                UT_String groupNameStr;
                evalString(groupNameStr, "viewportgroupname", 0, time);
                std::string groupName = groupNameStr.toStdString();
                if (groupName == "") {
                    addWarning(SOP_MESSAGE, "Cannot create a viewport group with an empty name, changing to _");
                    groupName = "_";
                }

                const std::string META_GROUP_VIEWPORT = "group_viewport";
                outputGrid->insertMeta(META_GROUP_VIEWPORT, StringMetadata(groupName));
            }
        }

        return error();

    } catch (std::exception& e) {
        addError(SOP_MESSAGE, e.what());
    }
    return error();
}


////////////////////////////////////////

// Copyright (c) 2015-2016 Double Negative Visual Effects
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
