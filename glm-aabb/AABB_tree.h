//
// Created by Xinming Zhang on 5/16/19.
//

#ifndef PHYSICSSIMULATION_AABB_TREE_H
#define PHYSICSSIMULATION_AABB_TREE_H

#include "AABB.hpp"
#include <vector>
#include <map>
#include <memory>

#define AABB_NULL_NODE 0xffffffff

struct Tetrahedra{
    std::vector<glm::vec3> point_list;
    CPM_GLM_AABB_NS::AABB aabb;
    unsigned int ID;

    Tetrahedra(){ point_list.resize(4);}

    Tetrahedra(glm::vec3 &p1, glm::vec3 &p2, glm::vec3 &p3, glm::vec3 &p4){
        point_list.push_back(p1);
        point_list.push_back(p2);
        point_list.push_back(p3);
        point_list.push_back(p4);
        set_aabb();
    }
    void set_aabb(){
        for (int i = 0; i < 4; ++i) {
            aabb.extend(point_list[i]);
        }
    }
    void insert(glm::vec3 &p1, glm::vec3 &p2, glm::vec3 &p3, glm::vec3 &p4){
        point_list[0] = p1;
        point_list[1] = p2;
        point_list[2] = p3;
        point_list[3] = p4;

    }
    CPM_GLM_AABB_NS::AABB get_aabb(){
        return aabb;
    }
};

struct AABBnode {
    CPM_GLM_AABB_NS::AABB aabb;
    Tetrahedra tetra;
    // tree links
    unsigned parentNodeIndex;
    unsigned leftNodeIndex;
    unsigned rightNodeIndex;
    // node linked list link
    unsigned nextNodeIndex;

    bool isLeaf() const {return leftNodeIndex == AABB_NULL_NODE; }
    AABBnode() :
    tetra(),
    parentNodeIndex(AABB_NULL_NODE),
    leftNodeIndex(AABB_NULL_NODE),
    rightNodeIndex(AABB_NULL_NODE),
    nextNodeIndex(AABB_NULL_NODE){}
};


class AABB_Tree{
public:
    AABB_Tree() = default;
    AABB_Tree(unsigned intialSize);
    ~AABB_Tree() = default;

    void insertObject(Tetrahedra &tetra);
    void removeObject(Tetrahedra &tetra);
    void updateObject(Tetrahedra &tetra);

    unsigned _rootNodeIndex;
    std::vector<AABBnode> get_node_list();

private:
    std::map<unsigned int, unsigned> _objectNodeIndexMap;
    std::vector<AABBnode> _nodes;

    unsigned _allocatedNodeCount;
    unsigned _nextFreeNodeIndex;
    unsigned _nodeCapacity;
    unsigned _growthSize;

    unsigned allocateNode();
    void deallocateNode(unsigned nodeIndex);
    void insertLeaf(unsigned leafNodeIndex);
    void removeLeaf(unsigned leafNodeIndex);
    void updateLeaf(unsigned leafNodeIndex, const CPM_GLM_AABB_NS::AABB& newAabb);
    void fixUpwardsTree(unsigned treenodeIndex);
};

#endif //PHYSICSSIMULATION_AABB_TREE_H
