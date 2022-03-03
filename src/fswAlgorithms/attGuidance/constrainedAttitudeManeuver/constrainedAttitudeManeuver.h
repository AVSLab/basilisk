/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef WAYPOINTREFERENCE_H
#define WAYPOINTREFERENCE_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include <map>
#include <iostream>
#include <fstream>
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"

struct constraintStruct {

    double keepOutDir_N[3];
    double keepInDir_N[3];
};

//! @brief The Node class is used to create nodes in the 3D MRP graph
class Node {
public:
    Node();
    Node(double sigma_BN[3], constraintStruct constraints, double keepOutFov, double keepOutBore_B[3]); //, double keepInFov, double keepInBoresight[3]);
    ~Node();

    double sigma_BN[3];
    double keepOutBore_N[3];
    bool isBoundary;
    bool isFree;
    double heuristic;
    double priority;
    Node *neighbors[52];
    int neighborCount;
    void appendNeighbor(Node *node);
    Node *path[20];
    int pathCount;
    void appendPathNode(Node *node);
};

/*
struct NodeProperties {

    std::map<int,std::map<int,std::map<int,Node>>> neighbors;
    Node *path[20];
}; */

/*! @brief waypoint reference module class */
class ConstrainedAttitudeManeuver: public SysModel {
public:
    ConstrainedAttitudeManeuver();
    ConstrainedAttitudeManeuver(int N);
    ~ConstrainedAttitudeManeuver(); 
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void ReadInputs();
    void GenerateGrid(Node startNode, Node goalNode);
         // no need to have maps in the arguments since those are class variables
    void AStar(Node *path[20]);

public:
    int N;                                                                          //!< Fineness level of discretization
    double sigma_BN_goal[3];                                                        //!< Initial S/C attitude
    double omega_BN_B_goal[3];                                                      //!< Initial S/C angular rate
    double keepOutFov;                                                              //|< Field of view of the sensitive instrument
    double keepOutBore_B[3];                                                        //|< Body-frame direction of the boresight of the sensitive instrument
    constraintStruct constraints;                                                   //!< Structure containing the constraint directions in inertial coordinates
    std::map<int,std::map<int,std::map<int,Node>>> NodesMap;                        //!< C++ map from node indices to Node class
    int keyS[3];                                                                    //!< key to Start node in NodesMap
    int keyG[3];                                                                    //!< key to Goal node in NodesMap

    ReadFunctor<SCStatesMsgPayload> scStateInMsg;                                   //!< Spacecraft state input message
    ReadFunctor<SpicePlanetStateMsgPayload> celBodyInMsg;                           //!< Celestial body state msg at which we pointing at
    BSKLogger bskLogger;                                                            //!< BSK Logging

private:
    SCStatesMsgPayload scStateMsgBuffer;
    SpicePlanetStateMsgPayload celBodyMsgBuffer;
};

void mirrorFunction(int indices[3], int mirrorIndices[8][3]);

void neighboringNodes(int indices[3], int neighbors[26][3]);

double distance(Node n1, Node n2);

//! @brief The NodeList class is used in the A* algorithm to handle Open and Closed lists O and C
class NodeList {
public:
    NodeList();
    ~NodeList();

    Node* list[50];
    int N;
    void append(Node* node);
    void pop(int M);
    void swap(int m, int n);
    void sort();

};

#endif
