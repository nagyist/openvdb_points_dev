///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2012-2016 DreamWorks Animation LLC
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of DreamWorks Animation nor the names of
// its contributors may be used to endorse or promote products derived
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
/// @author Dan Bailey
///
/// @file PointGroup.h
///
/// @brief  Point group manipulation in a VDB Point Grid.
///


#ifndef OPENVDB_POINTS_POINT_GROUP_HAS_BEEN_INCLUDED
#define OPENVDB_POINTS_POINT_GROUP_HAS_BEEN_INCLUDED

#include <openvdb/openvdb.h>

#include <openvdb/points/IndexIterator.h> // FilterTraits
#include <openvdb/points/IndexFilter.h> // FilterTraits
#include <openvdb/points/AttributeSet.h>
#include <openvdb/points/PointDataGrid.h>
#include <openvdb/points/PointAttribute.h>

#include <boost/ptr_container/ptr_vector.hpp>

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace points {

/// @brief Delete any group that is not present in the Descriptor.
///
/// @param groups        the vector of group names.
/// @param descriptor    the descriptor that holds the group map.
inline void deleteMissingPointGroups(   std::vector<std::string>& groups,
                                        const AttributeSet::Descriptor& descriptor);

/// @brief Appends a new empty group to the VDB tree.
///
/// @param tree          the PointDataTree to be appended to.
/// @param group         name of the new group.
template <typename PointDataTree>
inline void appendGroup(PointDataTree& tree,
                        const Name& group);

/// @brief Appends new empty groups to the VDB tree.
///
/// @param tree          the PointDataTree to be appended to.
/// @param groups        names of the new groups.
template <typename PointDataTree>
inline void appendGroups(PointDataTree& tree,
                         const std::vector<Name>& groups);

/// @brief Drops an existing group from the VDB tree.
///
/// @param tree          the PointDataTree to be dropped from.
/// @param group         name of the group.
/// @param compact       compact attributes if possible to reduce memory - if dropping
///                      more than one group, compacting once at the end will be faster
template <typename PointDataTree>
inline void dropGroup(  PointDataTree& tree,
                        const Name& group,
                        const bool compact = true);

/// @brief Drops existing groups from the VDB tree, the tree is compacted after dropping.
///
/// @param tree          the PointDataTree to be dropped from.
/// @param groups        names of the groups.
template <typename PointDataTree>
inline void dropGroups( PointDataTree& tree,
                        const std::vector<Name>& groups);

/// @brief Drops all existing groups from the VDB tree, the tree is compacted after dropping.
///
/// @param tree          the PointDataTree to be dropped from.
template <typename PointDataTree>
inline void dropGroups( PointDataTree& tree);

/// @brief Compacts existing groups of a VDB Tree to use less memory if possible.
///
/// @param tree          the PointDataTree to be compacted.
template <typename PointDataTree>
inline void compactGroups(PointDataTree& tree);

/// @brief Sets group membership from a PointIndexTree-ordered vector.
///
/// @param tree          the PointDataTree.
/// @param indexTree     the PointIndexTree.
/// @param membership    @c 1 if the point is in the group, 0 otherwise.
/// @param group         the name of the group.
/// @param remove        if @c true also perform removal of points from the group.
///
/// @note vector<bool> is not thread-safe on concurrent write, so use vector<short> instead
template <typename PointDataTree, typename PointIndexTree>
inline void setGroup(   PointDataTree& tree,
                        const PointIndexTree& indexTree,
                        const std::vector<short>& membership,
                        const Name& group,
                        const bool remove = false);

/// @brief Sets membership for the specified group for all points (on/off).
///
/// @param tree         the PointDataTree.
/// @param group        the name of the group.
/// @param member       true / false for membership of the group.
template <typename PointDataTree>
inline void setGroup(   PointDataTree& tree,
                        const Name& group,
                        const bool member = true);

/// @brief Sets group membership based on a provided filter.
///
/// @param tree          the PointDataTree.
/// @param group         the name of the group.
/// @param filterData    filter data that is used to create a per-leaf filter
template <typename PointDataTree, typename FilterT>
inline void setGroupByFilter(   PointDataTree& tree,
                                const Name& group,
                                const FilterT& filter);


////////////////////////////////////////


namespace point_group_internal {


/// Copy a group attribute value from one group offset to another
template<typename PointDataTreeType>
struct CopyGroupOp {

    typedef typename tree::LeafManager<PointDataTreeType>       LeafManagerT;
    typedef typename LeafManagerT::LeafRange                    LeafRangeT;
    typedef AttributeSet::Descriptor::GroupIndex                GroupIndex;

    CopyGroupOp(PointDataTreeType& tree,
                const GroupIndex& targetIndex,
                const GroupIndex& sourceIndex)
        : mTree(tree)
        , mTargetIndex(targetIndex)
        , mSourceIndex(sourceIndex) { }

    void operator()(const typename LeafManagerT::LeafRange& range) const {

        for (typename LeafManagerT::LeafRange::Iterator leaf=range.begin(); leaf; ++leaf) {

            GroupHandle sourceGroup = leaf->groupHandle(mSourceIndex);
            GroupWriteHandle targetGroup = leaf->groupWriteHandle(mTargetIndex);

            for (typename PointDataTreeType::LeafNodeType::IndexAllIter iter = leaf->beginIndexAll(); iter; ++iter) {
                const bool groupOn = sourceGroup.get(*iter);
                targetGroup.set(*iter, groupOn);
            }
        }
    }

    //////////

    PointDataTreeType&      mTree;
    const GroupIndex        mTargetIndex;
    const GroupIndex        mSourceIndex;
};


/// Set membership on or off for the specified group
template <typename PointDataTree, bool Member>
struct SetGroupOp
{
    typedef typename tree::LeafManager<PointDataTree>   LeafManagerT;
    typedef AttributeSet::Descriptor::GroupIndex        GroupIndex;

    SetGroupOp(const AttributeSet::Descriptor::GroupIndex& index)
        : mIndex(index) { }

    void operator()(const typename LeafManagerT::LeafRange& range) const
    {
        for (typename LeafManagerT::LeafRange::Iterator leaf=range.begin(); leaf; ++leaf) {

            // obtain the group attribute array

            GroupWriteHandle group(leaf->groupWriteHandle(mIndex));

            // set the group value

            group.collapse(Member);
        }
    }

    //////////

    const GroupIndex        mIndex;
}; // struct SetGroupOp


template <typename PointDataTree, typename PointIndexTree, bool Remove>
struct SetGroupFromIndexOp
{
    typedef typename tree::LeafManager<PointDataTree>   LeafManagerT;
    typedef typename LeafManagerT::LeafRange            LeafRangeT;
    typedef typename PointIndexTree::LeafNodeType       PointIndexLeafNode;
    typedef typename PointIndexLeafNode::IndexArray     IndexArray;
    typedef AttributeSet::Descriptor::GroupIndex        GroupIndex;
    typedef std::vector<short>                          MembershipArray;

    SetGroupFromIndexOp(const PointIndexTree& indexTree,
                        const MembershipArray& membership,
                        const GroupIndex& index)
        : mIndexTree(indexTree)
        , mMembership(membership)
        , mIndex(index) { }

    void operator()(const typename LeafManagerT::LeafRange& range) const
    {
        for (typename LeafManagerT::LeafRange::Iterator leaf=range.begin(); leaf; ++leaf) {

            // obtain the PointIndexLeafNode (using the origin of the current leaf)

            const PointIndexLeafNode* pointIndexLeaf = mIndexTree.probeConstLeaf(leaf->origin());

            if (!pointIndexLeaf)    continue;

            // obtain the group attribute array

            GroupWriteHandle group(leaf->groupWriteHandle(mIndex));

            // initialise the attribute storage

            Index64 index = 0;

            const IndexArray& indices = pointIndexLeaf->indices();

            for (typename IndexArray::const_iterator it = indices.begin(),
                                                     it_end = indices.end(); it != it_end; ++it)
            {
                if (Remove)                                 group.set(index, mMembership[*it]);
                else if (mMembership[*it] == short(1))      group.set(index, short(1));

                index++;
            }

            // attempt to compact the array

            group.compact();
        }
    }

    //////////

    const PointIndexTree& mIndexTree;
    const MembershipArray& mMembership;
    const GroupIndex mIndex;
}; // struct SetGroupFromIndexOp


template <typename PointDataTree, typename FilterT, typename IterT = typename PointDataTree::LeafNodeType::ValueAllCIter>
struct SetGroupByFilterOp
{
    typedef typename tree::LeafManager<PointDataTree>   LeafManagerT;
    typedef typename LeafManagerT::LeafRange            LeafRangeT;
    typedef typename PointDataTree::LeafNodeType        LeafNodeT;
    typedef AttributeSet::Descriptor::GroupIndex        GroupIndex;

    SetGroupByFilterOp( const GroupIndex& index, const FilterT& filter)
        : mIndex(index)
        , mFilter(filter) { }

    void operator()(const typename LeafManagerT::LeafRange& range) const
    {
        for (typename LeafManagerT::LeafRange::Iterator leaf=range.begin(); leaf; ++leaf) {

            // obtain the group attribute array

            GroupWriteHandle group(leaf->groupWriteHandle(mIndex));

            IndexIter<IterT, FilterT> iter = leaf->template beginIndex<IterT, FilterT>(mFilter);

            for (; iter; ++iter) {
                group.set(*iter, true);
            }

            // attempt to compact the array

            group.compact();
        }
    }

    //////////

    const GroupIndex mIndex;
    const FilterT mFilter;
}; // struct SetGroupByFilterOp


////////////////////////////////////////


/// Convenience class with methods for analyzing group data
class GroupInfo
{
public:
    typedef AttributeSet::Descriptor Descriptor;

    GroupInfo(const AttributeSet& attributeSet)
        : mAttributeSet(attributeSet) { }

    /// Return the number of bits in a group (typically 8)
    static size_t groupBits() { return sizeof(GroupType) * CHAR_BIT; }

    /// Return the number of empty group slots which correlates to the number of groups
    /// that can be stored without increasing the number of group attribute arrays
    size_t unusedGroups() const
    {
        const Descriptor& descriptor = mAttributeSet.descriptor();

        // compute total slots (one slot per bit of the group attributes)

        const size_t groupAttributes = descriptor.count(GroupAttributeArray::attributeType());

        if (groupAttributes == 0)   return 0;

        const size_t totalSlots = groupAttributes * this->groupBits();

        // compute slots in use

        const AttributeSet::Descriptor::NameToPosMap& groupMap = mAttributeSet.descriptor().groupMap();
        const size_t usedSlots = groupMap.size();

        return totalSlots - usedSlots;
    }

    /// Return @c true if there are sufficient empty slots to allow compacting
    bool canCompactGroups() const
    {
        // can compact if more unused groups than in one group attribute array

        return this->unusedGroups() >= this->groupBits();
    }

    /// Return the next empty group slot
    size_t nextUnusedOffset() const
    {
        const Descriptor::NameToPosMap& groupMap = mAttributeSet.descriptor().groupMap();

        // build a list of group indices

        std::vector<size_t> indices;
        for (Descriptor::ConstIterator  it = groupMap.begin(),
                                        endIt = groupMap.end(); it != endIt; ++it) {
            indices.push_back(it->second);
        }

        std::sort(indices.begin(), indices.end());

        // return first index not present

        size_t offset = 0;
        for (std::vector<size_t>::const_iterator    it = indices.begin(),
                                                    endIt = indices.end(); it != endIt; ++it) {
            if (*it != offset)     break;
            offset++;
        }

        return offset;
    }

    /// Fill the @p indices vector with the indices correlating to the group attribute arrays
    void populateGroupIndices(std::vector<size_t>& indices) const
    {
        const Descriptor::NameToPosMap& map = mAttributeSet.descriptor().map();

        for (Descriptor::ConstIterator  it = map.begin(),
                                        itEnd = map.end(); it != itEnd; ++it) {

            const AttributeArray* array = mAttributeSet.getConst(it->first);
            if (isGroup(*array)) {
                indices.push_back(it->second);
            }
        }
    }

    /// Determine if a move is required to efficiently compact the data and store the
    /// source name, offset and the target offset in the input parameters
    bool requiresMove(Name& sourceName, size_t& sourceOffset, size_t& targetOffset) const {

        targetOffset = this->nextUnusedOffset();

        const Descriptor::NameToPosMap& groupMap = mAttributeSet.descriptor().groupMap();

        typedef Descriptor::NameToPosMap::const_reverse_iterator ReverseMapIterator;

        for (ReverseMapIterator it = groupMap.rbegin(),
                                itEnd = groupMap.rend(); it != itEnd; ++it) {

            // move only required if source comes after the target

            if (it->second >= targetOffset) {
                sourceName = it->first;
                sourceOffset = it->second;
                return true;
            }
        }

        return false;
    }

private:
    const AttributeSet& mAttributeSet;
}; // class GroupInfo


} // namespace point_group_internal


////////////////////////////////////////


inline void deleteMissingPointGroups(   std::vector<std::string>& groups,
                                        const AttributeSet::Descriptor& descriptor)
{
    for (std::vector<std::string>::iterator it = groups.begin(); it != groups.end(); )
    {
        if (!descriptor.hasGroup(*it))  it = groups.erase(it);
        else                            ++it;
    }
}


////////////////////////////////////////


template <typename PointDataTree>
inline void appendGroup(PointDataTree& tree, const Name& group)
{
    typedef AttributeSet::Descriptor                              Descriptor;

    using point_attribute_internal::AppendAttributeOp;
    using point_group_internal::GroupInfo;

    if (group.empty()) {
        OPENVDB_THROW(KeyError, "Cannot use an empty group name as a key.");
    }

    typename PointDataTree::LeafCIter iter = tree.cbeginLeaf();

    if (!iter)  return;

    const AttributeSet& attributeSet = iter->attributeSet();
    Descriptor::Ptr descriptor = attributeSet.descriptorPtr();
    GroupInfo groupInfo(attributeSet);

    // don't add if group already exists

    if (descriptor->hasGroup(group))    return;

    // add a new group attribute if there are no unused groups

    if (groupInfo.unusedGroups() == 0) {

        // find a new internal group name

        const Name groupName = descriptor->uniqueName("__group");

        descriptor = descriptor->duplicateAppend(groupName, GroupAttributeArray::attributeType());

        const size_t pos = descriptor->find(groupName);

        // insert new group attribute

        AppendAttributeOp<PointDataTree> append(tree, descriptor, pos);
        tbb::parallel_for(typename tree::template LeafManager<PointDataTree>(tree).leafRange(), append);
    }
    else {
        // make the descriptor unique before we modify the group map

        makeDescriptorUnique(tree);
        descriptor = attributeSet.descriptorPtr();
    }

    // ensure that there are now available groups

    assert(groupInfo.unusedGroups() > 0);

    // find next unused offset

    const size_t offset = groupInfo.nextUnusedOffset();

    // add the group mapping to the descriptor

    descriptor->setGroup(group, offset);
}


////////////////////////////////////////


template <typename PointDataTree>
inline void appendGroups(PointDataTree& tree,
                         const std::vector<Name>& groups)
{
    // TODO: could be more efficient by appending multiple groups at once
    // instead of one-by-one, however this is likely not that common a use case

    for (std::vector<Name>::const_iterator  it = groups.begin(),
                                            itEnd = groups.end(); it != itEnd; ++it) {
        appendGroup(tree, *it);
    }
}


////////////////////////////////////////


template <typename PointDataTree>
inline void dropGroup(PointDataTree& tree, const Name& group, const bool compact)
{
    typedef AttributeSet::Descriptor                              Descriptor;

    if (group.empty()) {
        OPENVDB_THROW(KeyError, "Cannot use an empty group name as a key.");
    }

    typename PointDataTree::LeafCIter iter = tree.cbeginLeaf();

    if (!iter)  return;

    const AttributeSet& attributeSet = iter->attributeSet();

    // make the descriptor unique before we modify the group map

    makeDescriptorUnique(tree);
    Descriptor::Ptr descriptor = attributeSet.descriptorPtr();

    // now drop the group

    descriptor->dropGroup(group);

    if (compact) {
        compactGroups(tree);
    }
}


////////////////////////////////////////


template <typename PointDataTree>
inline void dropGroups( PointDataTree& tree,
                        const std::vector<Name>& groups)
{
    for (std::vector<Name>::const_iterator  it = groups.begin(),
                                            itEnd = groups.end(); it != itEnd; ++it) {
        dropGroup(tree, *it, /*compact=*/false);
    }

    // compaction done once for efficiency

    compactGroups(tree);
}


////////////////////////////////////////


template <typename PointDataTree>
inline void dropGroups( PointDataTree& tree)
{
    typedef AttributeSet::Descriptor        Descriptor;

    using point_group_internal::GroupInfo;

    typename PointDataTree::LeafCIter iter = tree.cbeginLeaf();

    if (!iter)  return;

    const AttributeSet& attributeSet = iter->attributeSet();
    GroupInfo groupInfo(attributeSet);

    // make the descriptor unique before we modify the group map

    makeDescriptorUnique(tree);
    Descriptor::Ptr descriptor = attributeSet.descriptorPtr();

    descriptor->clearGroups();

    // find all indices for group attribute arrays

    std::vector<size_t> indices;
    groupInfo.populateGroupIndices(indices);

    // drop these attributes arrays

    dropAttributes(tree, indices);
}


////////////////////////////////////////


template <typename PointDataTree>
inline void compactGroups(PointDataTree& tree)
{
    typedef AttributeSet::Descriptor                              Descriptor;
    typedef Descriptor::GroupIndex                                GroupIndex;

    using point_group_internal::CopyGroupOp;
    using point_group_internal::GroupInfo;

    typename PointDataTree::LeafCIter iter = tree.cbeginLeaf();

    if (!iter)  return;

    const AttributeSet& attributeSet = iter->attributeSet();
    GroupInfo groupInfo(attributeSet);

    // early exit if not possible to compact

    if (!groupInfo.canCompactGroups())    return;

    // make the descriptor unique before we modify the group map

    makeDescriptorUnique(tree);
    Descriptor::Ptr descriptor = attributeSet.descriptorPtr();

    // generate a list of group offsets and move them (one-by-one)
    // TODO: improve this algorithm to move multiple groups per array at once
    // though this is likely not that common a use case

    Name sourceName;
    size_t sourceOffset, targetOffset;

    while (groupInfo.requiresMove(sourceName, sourceOffset, targetOffset)) {

        const GroupIndex sourceIndex = attributeSet.groupIndex(sourceOffset);
        const GroupIndex targetIndex = attributeSet.groupIndex(targetOffset);

        CopyGroupOp<PointDataTree> copy(tree, targetIndex, sourceIndex);
        tbb::parallel_for(typename tree::template LeafManager<PointDataTree>(tree).leafRange(), copy);

        descriptor->setGroup(sourceName, targetOffset);
    }

    // drop unused attribute arrays

    std::vector<size_t> indices;
    groupInfo.populateGroupIndices(indices);

    const size_t totalAttributesToDrop = groupInfo.unusedGroups() / groupInfo.groupBits();

    assert(totalAttributesToDrop <= indices.size());

    std::vector<size_t> indicesToDrop(indices.end() - totalAttributesToDrop, indices.end());

    dropAttributes(tree, indicesToDrop);
}


////////////////////////////////////////


template <typename PointDataTree, typename PointIndexTree>
inline void setGroup(   PointDataTree& tree,
                        const PointIndexTree& indexTree,
                        const std::vector<short>& membership,
                        const Name& group,
                        const bool remove)
{
    typedef AttributeSet::Descriptor Descriptor;
    typedef typename tree::template LeafManager<PointDataTree> LeafManagerT;

    if (membership.size() != pointCount(tree)) {
        OPENVDB_THROW(LookupError, "Membership vector size must match number of points.");
    }

    using point_group_internal::SetGroupFromIndexOp;

    typename PointDataTree::LeafCIter iter = tree.cbeginLeaf();

    if (!iter)  return;

    const AttributeSet& attributeSet = iter->attributeSet();
    const Descriptor& descriptor = attributeSet.descriptor();

    if (!descriptor.hasGroup(group)) {
        OPENVDB_THROW(LookupError, "Group must exist on Tree before defining membership.");
    }

    const Descriptor::GroupIndex index = attributeSet.groupIndex(group);

    // set membership

    if (remove) {
        SetGroupFromIndexOp<PointDataTree,
                            PointIndexTree, false> set(indexTree, membership, index);
        tbb::parallel_for(LeafManagerT(tree).leafRange(), set);
    }
    else {
        SetGroupFromIndexOp<PointDataTree,
                            PointIndexTree, true> set(indexTree, membership, index);
        tbb::parallel_for(LeafManagerT(tree).leafRange(), set);
    }
}


////////////////////////////////////////


template <typename PointDataTree>
inline void setGroup(   PointDataTree& tree,
                        const Name& group,
                        const bool member)
{
    typedef AttributeSet::Descriptor Descriptor;
    typedef typename tree::template LeafManager<PointDataTree> LeafManagerT;

    using point_group_internal::SetGroupOp;

    typename PointDataTree::LeafCIter iter = tree.cbeginLeaf();

    if (!iter)  return;

    const AttributeSet& attributeSet = iter->attributeSet();
    const Descriptor& descriptor = attributeSet.descriptor();

    if (!descriptor.hasGroup(group)) {
        OPENVDB_THROW(LookupError, "Group must exist on Tree before defining membership.");
    }

    const Descriptor::GroupIndex index = attributeSet.groupIndex(group);

    // set membership based on member variable

    if (member)     tbb::parallel_for(LeafManagerT(tree).leafRange(), SetGroupOp<PointDataTree, true>(index));
    else            tbb::parallel_for(LeafManagerT(tree).leafRange(), SetGroupOp<PointDataTree, false>(index));
}


////////////////////////////////////////


template <typename PointDataTree, typename FilterT>
inline void setGroupByFilter(   PointDataTree& tree,
                                const Name& group,
                                const FilterT& filter)
{
    typedef AttributeSet::Descriptor Descriptor;
    typedef typename tree::template LeafManager<PointDataTree> LeafManagerT;

    using point_group_internal::SetGroupByFilterOp;

    typename PointDataTree::LeafCIter iter = tree.cbeginLeaf();

    if (!iter)  return;

    const AttributeSet& attributeSet = iter->attributeSet();
    const Descriptor& descriptor = attributeSet.descriptor();

    if (!descriptor.hasGroup(group)) {
        OPENVDB_THROW(LookupError, "Group must exist on Tree before defining membership.");
    }

    const Descriptor::GroupIndex index = attributeSet.groupIndex(group);

    // set membership using filter

    SetGroupByFilterOp<PointDataTree, FilterT> set(index, filter);
    tbb::parallel_for(LeafManagerT(tree).leafRange(), set);
}


////////////////////////////////////////


template <typename PointDataTree>
inline void setGroupByRandomTarget( PointDataTree& tree,
                                    const Name& group,
                                    const Index64 targetPoints,
                                    const unsigned int seed = 0)
{
    typedef RandomLeafFilter<PointDataTree, std::mt19937> RandomFilter;

    RandomFilter filter(tree, targetPoints, seed);

    setGroupByFilter<PointDataTree, RandomFilter>(tree, group, filter);
}


////////////////////////////////////////


template <typename PointDataTree>
inline void setGroupByRandomPercentage( PointDataTree& tree,
                                        const Name& group,
                                        const float percentage = 10.0f,
                                        const unsigned int seed = 0)
{
    typedef RandomLeafFilter<PointDataTree, std::mt19937> RandomFilter;

    const int currentPoints = pointCount(tree);
    const int targetPoints = int(math::Round((percentage * currentPoints)/100.0f));

    RandomFilter filter(tree, targetPoints, seed);

    setGroupByFilter<PointDataTree, RandomFilter>(tree, group, filter);
}


////////////////////////////////////////


} // namespace points
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb


#endif // OPENVDB_POINTS_POINT_GROUP_HAS_BEEN_INCLUDED


// Copyright (c) 2012-2016 DreamWorks Animation LLC
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
