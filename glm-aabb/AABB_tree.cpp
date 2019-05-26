//
// Created by Xinming Zhang on 5/16/19.
//

#include "AABB_tree.h"

AABB_Tree::AABB_Tree(unsigned intialSize) :
_rootNodeIndex(AABB_NULL_NODE),
_allocatedNodeCount(0),
_nextFreeNodeIndex(0),
_nodeCapacity(intialSize),
_growthSize(intialSize)
{
    _nodes.resize(intialSize);
    for (unsigned nodeIndex = 0; nodeIndex < intialSize; ++nodeIndex) {
        AABBnode& node = _nodes[nodeIndex];
        node.nextNodeIndex = nodeIndex + 1;
    }
    _nodes[intialSize-1].nextNodeIndex = AABB_NULL_NODE;
}

unsigned AABB_Tree::allocateNode() {
    // if we have no free tree nodes space then grow the pool
    if(_nextFreeNodeIndex == AABB_NULL_NODE){
        assert(_allocatedNodeCount == _nodeCapacity);

        _nodeCapacity += _growthSize;
        _nodes.resize(_nodeCapacity);
        for (unsigned nodeIndex = _allocatedNodeCount-1; nodeIndex < _nodeCapacity; ++nodeIndex) {
            AABBnode &node = _nodes[nodeIndex];
            node.nextNodeIndex = nodeIndex + 1;
        }
        _nodes[_nodeCapacity-1].nextNodeIndex = AABB_NULL_NODE;
        _nextFreeNodeIndex = _allocatedNodeCount;
    }

    unsigned nodeIndex = _nextFreeNodeIndex;
    AABBnode& allocatedNode = _nodes[nodeIndex];
    allocatedNode.parentNodeIndex = AABB_NULL_NODE;
    allocatedNode.leftNodeIndex = AABB_NULL_NODE;
    allocatedNode.rightNodeIndex = AABB_NULL_NODE;
    _nextFreeNodeIndex = allocatedNode.nextNodeIndex;
    _allocatedNodeCount++;

    return nodeIndex;
}

void AABB_Tree::deallocateNode(unsigned nodeIndex) {
    AABBnode& deallocatedNode = _nodes[nodeIndex];
    deallocatedNode.nextNodeIndex = _nextFreeNodeIndex;
    _nextFreeNodeIndex = nodeIndex;
    _allocatedNodeCount--;
}

void AABB_Tree::insertObject(Tetrahedra &tetra) {
    // get the free node index to allocate
    unsigned nodeIndex = allocateNode();
    AABBnode& node = _nodes[nodeIndex];

    node.aabb = tetra.get_aabb();
    node.tetra = tetra;

    insertLeaf(nodeIndex);
    _objectNodeIndexMap[tetra.ID] = nodeIndex;
}

void AABB_Tree::removeObject(Tetrahedra &tetra) {
    unsigned nodeIndex = _objectNodeIndexMap[tetra.ID];
    removeLeaf(nodeIndex);
    deallocateNode(nodeIndex);
    _objectNodeIndexMap.erase(tetra.ID);
}

void AABB_Tree::updateObject(Tetrahedra &tetra) {
    unsigned nodeIndex = _objectNodeIndexMap[tetra.ID];
    updateLeaf(nodeIndex, tetra.get_aabb());
}

// queryOverlap

//
void AABB_Tree::insertLeaf(unsigned leafNodeIndex) {
    // make sure we are inserting a new leaf
    assert(_nodes[leafNodeIndex].parentNodeIndex == AABB_NULL_NODE);
    assert(_nodes[leafNodeIndex].leftNodeIndex == AABB_NULL_NODE);
    assert(_nodes[leafNodeIndex].rightNodeIndex == AABB_NULL_NODE);

    // if the tree is empty then we make the root the leaf
    if(_rootNodeIndex == AABB_NULL_NODE){
        _rootNodeIndex = leafNodeIndex;
        return;
    }

    // search for the best place to put the new leaf in the tree
    // we use the surface area and depth as heuristics
    unsigned treeNodeIndex = _rootNodeIndex;
    AABBnode& leafNode = _nodes[leafNodeIndex];
    while(!_nodes[treeNodeIndex].isLeaf()){
        // because of the test in the while loop above we know we are never a leaf inside it
        AABBnode& treeNode = _nodes[treeNodeIndex];

        unsigned leftNodeIndex = treeNode.leftNodeIndex;
        unsigned rightNodeIndex = treeNode.rightNodeIndex;

        const AABBnode &leftNode = _nodes[leftNodeIndex];
        const AABBnode &rightNode = _nodes[rightNodeIndex];

        // extend to new containing aabb for root node
        CPM_GLM_AABB_NS::AABB combinedAabb = treeNode.aabb;
        combinedAabb.extend(leafNode.aabb);

        float newParentNodeCost = 2.0f * combinedAabb.calculate_surface_area();
        float minimumPushDownCost = 2.0f * (combinedAabb.calculate_surface_area() - treeNode.aabb.calculate_surface_area());

        // use the costs to figure out whether to create a new parent here or descend
        float costLeft;
        float costRight;

        if(leftNode.isLeaf()){
            CPM_GLM_AABB_NS::AABB leaf_aabb = leafNode.aabb;
            leaf_aabb.extend(leftNode.aabb);
            costLeft = leaf_aabb.calculate_surface_area() + minimumPushDownCost;
        }
        else{
            CPM_GLM_AABB_NS::AABB leaf_aabb = leafNode.aabb;
            leaf_aabb.extend(leftNode.aabb);
            costLeft = (leaf_aabb.calculate_surface_area() - leftNode.aabb.calculate_surface_area()) + minimumPushDownCost;
        }
        if(rightNode.isLeaf()){
            CPM_GLM_AABB_NS::AABB leaf_aabb = leafNode.aabb;
            leaf_aabb.extend(rightNode.aabb);
            costRight = leaf_aabb.calculate_surface_area() + minimumPushDownCost;
        }
        else{
            CPM_GLM_AABB_NS::AABB leaf_aabb = leafNode.aabb;
            leaf_aabb.extend(rightNode.aabb);
            costRight = (leaf_aabb.calculate_surface_area() - rightNode.aabb.calculate_surface_area()) + minimumPushDownCost;
        }

        // if the cost of creating a new parent node here is less than descending in either direction then
        // we know we need to create a new parent node, here and attach the leaf to that
        if(newParentNodeCost < costLeft && newParentNodeCost < costRight)
            break;

        // otherwise descend in the cheapest direction
        if(costLeft < costRight)
            treeNodeIndex = leftNodeIndex;
        else
            treeNodeIndex = rightNodeIndex;

    }

    // the leafs sibling is going to be the node we found above and we are going to to create a new
    // parent node and attach the leaf and this item
    unsigned leafSiblingIndex = treeNodeIndex;
    AABBnode &leafSibling = _nodes[leafSiblingIndex];
    unsigned oldParentIndex = leafSibling.parentNodeIndex;
    unsigned newParentIndex = allocateNode();
    AABBnode &newParent = _nodes[newParentIndex];
    newParent.parentNodeIndex = oldParentIndex;

    CPM_GLM_AABB_NS::AABB new_aabb = leafNode.aabb;
    new_aabb.extend(leafSibling.aabb);
    newParent.aabb = new_aabb;

    newParent.leftNodeIndex = leafSiblingIndex;
    newParent.rightNodeIndex = leafNodeIndex;
    leafNode.parentNodeIndex = newParentIndex;
    leafSibling.parentNodeIndex = newParentIndex;

    if(oldParentIndex == AABB_NULL_NODE){
        // the old parent was the root and so this is now the root
        _rootNodeIndex = newParentIndex;
    }
    else{
        // the old parent was not the root and so we need to patch the left or right index to
        // point to the new node
        AABBnode& oldParent = _nodes[oldParentIndex];
        if(oldParent.leftNodeIndex == leafSiblingIndex)
            oldParent.leftNodeIndex = newParentIndex;
        else
            oldParent.rightNodeIndex = newParentIndex;
    }

    // finally we need to walk back up the tree fixing heights and areas
    treeNodeIndex = leafNode.parentNodeIndex;
    fixUpwardsTree(treeNodeIndex);
}

void AABB_Tree::removeLeaf(unsigned leafNodeIndex) {
    // if the leaf is the root then we can just clear the root pointer and return
    if(leafNodeIndex == _rootNodeIndex){
        _rootNodeIndex = AABB_NULL_NODE;
        return;
    }

    AABBnode& leafNode = _nodes[leafNodeIndex];
    unsigned parentNodeIndex = leafNode.parentNodeIndex;
    const AABBnode& parentNode = _nodes[parentNodeIndex];
    unsigned grandParentNodeIndex = parentNode.parentNodeIndex;
    unsigned  siblingNodeIndex = parentNode.leftNodeIndex == leafNodeIndex ? parentNode.rightNodeIndex : parentNode.leftNodeIndex;
    // we must have a sibling
    assert(siblingNodeIndex != AABB_NULL_NODE);
    AABBnode &siblingNode = _nodes[siblingNodeIndex];

    if(grandParentNodeIndex != AABB_NULL_NODE){
        // if we have a grand parent
        // (i.e. the parent is not the root)
        // then destroy the parent and connect the sibling to the grandparent in its
        // place
        AABBnode& grandParentNode = _nodes[grandParentNodeIndex];
        if(grandParentNode.leftNodeIndex == parentNodeIndex)
            grandParentNode.leftNodeIndex = siblingNodeIndex;
        else
            grandParentNode.rightNodeIndex = siblingNodeIndex;

        siblingNode.parentNodeIndex = grandParentNodeIndex;
        deallocateNode(parentNodeIndex);

        fixUpwardsTree(grandParentNodeIndex);
    }
    else{
        // if we have no grandparent the the parent is the root and so
        // out sibling becomes the root and has it's parent removed
        _rootNodeIndex = siblingNodeIndex;
        siblingNode.parentNodeIndex = AABB_NULL_NODE;
        deallocateNode(parentNodeIndex);
    }

    leafNode.parentNodeIndex = AABB_NULL_NODE;
}

void AABB_Tree::updateLeaf(unsigned leafNodeIndex, const CPM_GLM_AABB_NS::AABB &newAabb) {
    AABBnode& node = _nodes[leafNodeIndex];

    // if the node contains the new aabb then we just leave things
    //
    if(node.aabb.intersect(newAabb) == CPM_GLM_AABB_NS::AABB::INSIDE){
        return;
    }
    removeLeaf(leafNodeIndex);
    node.aabb = newAabb;
    insertLeaf(leafNodeIndex);
}

void AABB_Tree::fixUpwardsTree(unsigned treenodeIndex) {
    while(treenodeIndex != AABB_NULL_NODE){
        AABBnode& treeNode = _nodes[treenodeIndex];

        // every node should be a parent
        assert(treeNode.leftNodeIndex != AABB_NULL_NODE && treeNode.rightNodeIndex != AABB_NULL_NODE);

        // fix height and area
        const AABBnode& leftNode = _nodes[treeNode.leftNodeIndex];
        const AABBnode& rightNode = _nodes[treeNode.rightNodeIndex];
        CPM_GLM_AABB_NS::AABB tree_aabb = leftNode.aabb;
        tree_aabb.extend(rightNode.aabb);
        treeNode.aabb = tree_aabb;

        treenodeIndex = treeNode.parentNodeIndex;
    }
}

std::vector<AABBnode> AABB_Tree::get_node_list() {
    return _nodes;
}