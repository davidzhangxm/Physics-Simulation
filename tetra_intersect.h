//
// Created by Xinming Zhang on 5/15/19.
//

#ifndef PHYSICSSIMULATION_TETRA_INTERSECT_H
#define PHYSICSSIMULATION_TETRA_INTERSECT_H

#include <iostream>
#include <vector>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <glm/gtx/normal.hpp>

namespace TETRA_INTER {
    // face A---------------------------------
    inline static bool FaceA_1(double* Coord ,int& maskEdges, std::vector<glm::vec3>& P_V1, glm::vec3 &n){
        maskEdges = 000;

        if((Coord[0] = glm::dot(P_V1[0], n)) > 0) maskEdges = 001;
        if((Coord[1] = glm::dot(P_V1[1], n)) > 0) maskEdges |= 002;
        if((Coord[2] = glm::dot(P_V1[2], n)) > 0) maskEdges |= 004;
        if((Coord[3] = glm::dot(P_V1[3], n)) > 0) maskEdges |= 010;
        // if true it means that all of the vertices are out of the halfspace
        // defined by this face
        return (maskEdges == 017);
    }

    // it is the same as FaceA_1 only the values V2[0] -v_ref are used inly for the fourth face

    inline static bool FaceA_2(double* Coord, int& maskEdges, std::vector<glm::vec3>& V1, std::vector<glm::vec3>& V2, glm::vec3& n){
        maskEdges = 000;
        glm::vec3 v_ref = V1[1];

        if(( Coord[0] = glm::dot(V2[0] - v_ref, n)) > 0) maskEdges = 001;
        if(( Coord[1] = glm::dot(V2[1] - v_ref, n)) > 0) maskEdges |= 002;
        if(( Coord[2] = glm::dot(V2[2] - v_ref, n)) > 0) maskEdges |= 004;
        if(( Coord[3] = glm::dot(V2[3] - v_ref, n)) > 0) maskEdges |= 010;

        return (maskEdges == 017);
    }

    // FaceB -------------------------------------
    inline static bool FaceB_1(std::vector<glm::vec3>& P_V2, glm::vec3& n){
        return ((glm::dot(P_V2[0], n)>0) &&
                (glm::dot(P_V2[1], n)>0) &&
                (glm::dot(P_V2[2], n)>0) &&
                (glm::dot(P_V2[3], n)>0));
    }

    inline static bool FaceB_2(std::vector<glm::vec3>& V1,std::vector<glm::vec3>& V2, glm::vec3& n){
        glm::vec3 v_ref = V2[1];
        return ((glm::dot(V1[0] - v_ref, n)>0) &&
                (glm::dot(V1[1] - v_ref, n)>0) &&
                (glm::dot(V1[2] - v_ref, n)>0) &&
                (glm::dot(V1[3] - v_ref, n)>0));
    }
    // EdgeA --------------------------------------
    inline static bool EdgeA(const int& f0, const int &f1, double Coord_1[4][4], int masks[4]){
        double* coord_f0 = &Coord_1[f0][0];
        double* coord_f1 = &Coord_1[f1][0];

        int maskf0 = masks[f0];
        int maskf1 = masks[f1];
        // if there is a vertex of b included in (-,-) return false;
        if( (maskf0 | maskf1) != 017 )
            return false;

        maskf0 &= (maskf0 ^ maskf1); // exclude the vertices in (+, +)
        maskf1 &= (maskf0 ^ maskf1);

        // edge 0: 0--1

        if( ((maskf0 & 001) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 002)) && // the vertex 1 of b is in (+,-)
             // the edge of b (0,1) intersect (-,-)
            (((coord_f0[1] * coord_f1[0]) - (coord_f0[0] * coord_f1[1]) )> 0))
            return false;

        if( ((maskf0 & 002) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 001)) && // the vertex 1 of b is in (+,-)
            // the edge of b (0,1) intersect (-,-)
            (((coord_f0[1] * coord_f1[0]) - (coord_f0[0] * coord_f1[1])) > 0))
            return false;

        // edge 1: 0--2
        if( ((maskf0 & 001) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 004)) && // the vertex 1 of b is in (+,-)
            // the edge of b (0,1) intersect (-,-)
            (((coord_f0[2] * coord_f1[0]) - (coord_f0[0] * coord_f1[2])) > 0))
            return false;

        if( ((maskf0 & 004) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 001)) && // the vertex 1 of b is in (+,-)
            // the edge of b (0,1) intersect (-,-)
            (((coord_f0[2] * coord_f1[0]) - (coord_f0[0] * coord_f1[2])) > 0))
            return false;

        // edge 2: 0--3
        if( ((maskf0 & 001) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 010)) && // the vertex 1 of b is in (+,-)
            // the edge of b (0,1) intersect (-,-)
            (((coord_f0[3] * coord_f1[0]) - (coord_f0[0] * coord_f1[3])) > 0))
            return false;

        if( ((maskf0 & 010) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 001)) && // the vertex 1 of b is in (+,-)
            // the edge of b (0,1) intersect (-,-)
            (((coord_f0[3] * coord_f1[0]) - (coord_f0[0] * coord_f1[3])) > 0))
            return false;

        // edge 3: 1--2
        if( ((maskf0 & 002) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 004)) && // the vertex 1 of b is in (+,-)
            // the edge of b (0,1) intersect (-,-)
            (((coord_f0[2] * coord_f1[1]) - (coord_f0[1] * coord_f1[2])) > 0))
            return false;

        if( ((maskf0 & 004) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 002)) && // the vertex 1 of b is in (+,-)
            // the edge of b (0,1) intersect (-,-)
            (((coord_f0[2] * coord_f1[1]) - (coord_f0[1] * coord_f1[2])) > 0))
            return false;

        // edge 4: 1--3
        if( ((maskf0 & 002) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 010)) && // the vertex 1 of b is in (+,-)
            // the edge of b (0,1) intersect (-,-)
            (((coord_f0[3] * coord_f1[1]) - (coord_f0[1] * coord_f1[3])) > 0))
            return false;

        if( ((maskf0 & 010) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 002)) && // the vertex 1 of b is in (+,-)
            // the edge of b (0,1) intersect (-,-)
            (((coord_f0[3] * coord_f1[1]) - (coord_f0[1] * coord_f1[3])) > 0))
            return false;

        // edge 5: 2--3
        if( ((maskf0 & 004) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 010)) && // the vertex 1 of b is in (+,-)
            // the edge of b (0,1) intersect (-,-)
            (((coord_f0[3] * coord_f1[2]) - (coord_f0[2] * coord_f1[3])) > 0))
            return false;

        if( ((maskf0 & 010) &&  // the vertex 0 of b is in (-,+)
             (maskf1 & 004)) && // the vertex 1 of b is in (+,-)
            // the edge of b (0,1) intersect (-,-)
            (((coord_f0[3] * coord_f1[2]) - (coord_f0[2] * coord_f1[3])) > 0))
            return false;
        // there exists a seperating plane supported by the edge shared by f0 and f1
        return true;
    }

    bool tet_a_tet(std::vector<glm::vec3> V_1, std::vector<glm::vec3> V_2){

        // argument four vertex in the tetrahedra
        std::vector<glm::vec3> V1, V2; // vertex coordinate
        // vectors edge-oriented
        std::vector<glm::vec3> e_v1, e_v2;

        int masks[4]; // for each face of the first tetrahedron
        // stores the halspace each vertex of the
        // second tetrahedron belongs to
        // difference between the verteices of the second tetrahedron and
        // the vertex 0 of the first tetrahedron
        std::vector<glm::vec3> P_V1, P_V2;
        //
        double Coord_1[4][4], Coord_2[4][4]; // verteices coordinates in the affine space

        glm::vec3 n;

        V1 = V_1;
        V2 = V_2;

        P_V1.push_back(V2[0] - V1[0]);
        P_V1.push_back(V2[1] - V1[0]);
        P_V1.push_back(V2[2] - V1[0]);
        P_V1.push_back(V2[3] - V1[0]);

        e_v1.push_back(V1[1] - V1[0]);
        e_v1.push_back(V1[2] - V1[0]);
        // normal point outward
        n = glm::cross(e_v1[1], e_v1[0]);
        // plane 0 in tetrahedron 1
        if(FaceA_1(&Coord_1[0][0], masks[0], P_V1, n))
            return false;

        e_v1.push_back(V1[3] - V1[0]);
        n = glm::cross(e_v1[0], e_v1[2]);
        // plane 1 in tetrahedron 2
        if(FaceA_1(&Coord_1[1][0], masks[1], P_V1, n))
            return false;

        if(EdgeA(0, 1, Coord_1, masks)) return false;

        n = glm::cross(e_v1[2], e_v1[1]);

        if(FaceA_1(&Coord_1[2][0], masks[2], P_V1, n))
            return false;

        if(EdgeA(0, 2, Coord_1, masks)) return false;
        if(EdgeA(1, 2, Coord_1, masks)) return false;

        e_v1.push_back(V1[2] - V1[1]);
        e_v1.push_back(V1[3] - V1[1]);

        n = glm::cross(e_v1[3], e_v1[4]);

        if(FaceA_2(&Coord_1[3][0], masks[3], V1, V2, n))
            return false;

        if(EdgeA(0, 3, Coord_1, masks)) return false;
        if(EdgeA(1, 3, Coord_1, masks)) return false;
        if(EdgeA(2, 3, Coord_1, masks)) return false;
        // pointwise check
        if( (masks[0] | masks[1] | masks[2] | masks[3]) != 017 ) return true;

        // from now on, if there is a separating plane it is parallel to face of b
        P_V2.push_back(V1[0] - V2[0]);
        P_V2.push_back(V1[1] - V2[0]);
        P_V2.push_back(V1[2] - V2[0]);
        P_V2.push_back(V1[3] - V2[0]);

        e_v2.push_back(V2[1] - V2[0]);
        e_v2.push_back(V2[2] - V2[0]);
        // normal point outward
        n = glm::cross(e_v2[1], e_v2[0]);

        if(FaceB_1(P_V2, n)) return false;

        e_v2.push_back(V2[3] - V2[0]);
        n = glm::cross(e_v2[0], e_v2[2]);

        if(FaceB_1(P_V2, n)) return false;

        n = glm::cross(e_v2[2], e_v2[1]);
        if(FaceB_1(P_V2, n)) return false;

        e_v2.push_back(V2[2] - V2[1]);
        e_v2.push_back(V2[3] - V2[1]);

        n = glm::cross(e_v2[3], e_v2[4]);

        if(FaceB_2(V1, V2, n)) return false;
        return true;

    }
}

#endif //PHYSICSSIMULATION_TETRA_INTERSECT_H
