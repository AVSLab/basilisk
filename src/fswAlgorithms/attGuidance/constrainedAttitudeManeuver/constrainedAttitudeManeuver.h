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
#include "architecture/utilities/BSpline.h"
#include "architecture/messaging/messaging.h"
#include <map>
#include <iostream>
#include <fstream>
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"

struct constraintStruct {

    double keepOutDir_N[3], keepInDir_N[3];
    bool keepOut, keepIn;
};

struct scBoresightStruct {

    double keepOutBoresight_B[10][3], keepInBoresight_B[10][3];
    double keepOutFov[10], keepInFov[10];
    int keepOutBoresightCount = 0;
    int keepInBoresightCount = 0;
};

//! @brief The Node class is used to create nodes in the 3D MRP graph
class Node {
public:
    Node();
    Node(double sigma_BN[3], constraintStruct constraints, scBoresightStruct boresights);
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
    Node *backPointer;
    int pathCount;
    void appendPathNode(Node *node);
};

//! @brief The NodeList class is used in the A* algorithm to handle Open and Closed lists O and C
class NodeList {
public:
    NodeList();
    ~NodeList();

    Node* list[10000];
    int N;
    void append(Node* node);
    void pop(int M);
    void clear();
    void swap(int m, int n);
    void sort();
    bool contains(Node *node);
};

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
    double returnNodeCoord(int key[3], int nodeCoord);
    bool returnNodeState(int key[3]);
    double returnPathCoord(int index, int nodeCoord);
    void AStar();
    void effortBasedAStar();
    void backtrack(Node *p);
    void pathHandle();
    void spline();
    void computeTorque(int n, double I[9], double L[3]);
    double effortEvaluation();

public:
    int N;                                                                          //!< Fineness level of discretization
    int BSplineType;                                                                //!< 0 for interpolation; 1 for LS approximation
    double sigma_BN_goal[3];                                                        //!< Initial S/C attitude
    double omega_BN_B_goal[3];                                                      //!< Initial S/C angular rate
    double avgOmega;
    double keepOutFov;                                                              //|< Field of view of the sensitive instrument
    double keepOutBore_B[3];                                                        //|< Body-frame direction of the boresight of the sensitive instrument
    constraintStruct constraints;                                                   //!< Structure containing the constraint directions in inertial coordinates
    scBoresightStruct boresights;
    void appendKeepOutDirection(double direction[3], double Fov);
    void appendKeepInDirection(double direction[3], double Fov);
    std::map<int,std::map<int,std::map<int,Node>>> NodesMap;                        //!< C++ map from node indices to Node class
    int keyS[3];                                                                    //!< key to Start node in NodesMap
    int keyG[3];                                                                    //!< key to Goal node in NodesMap
    NodeList path;
    InputDataSet Input;
    OutputDataSet Output;

    ReadFunctor<SCStatesMsgPayload> scStateInMsg;                                   //!< Spacecraft state input message
    ReadFunctor<VehicleConfigMsgPayload> vehicleConfigInMsg;                        //!< FSW vehicle configuration input message
    ReadFunctor<SpicePlanetStateMsgPayload> keepOutCelBodyInMsg;                    //!< Celestial body state msg - keep out direction
    ReadFunctor<SpicePlanetStateMsgPayload> keepInCelBodyInMsg;                     //!< Celestial body state msg - keep in direction
    Message<AttRefMsgPayload> attRefOutMsg;
    BSKLogger bskLogger;                                                            //!< BSK Logging

private:
    SCStatesMsgPayload scStateMsgBuffer;
    VehicleConfigMsgPayload vehicleConfigMsgBuffer;
    SpicePlanetStateMsgPayload keepOutCelBodyMsgBuffer;
    SpicePlanetStateMsgPayload keepInCelBodyMsgBuffer;
};

void mirrorFunction(int indices[3], int mirrorIndices[8][3]);

void neighboringNodes(int indices[3], int neighbors[26][3]);

double distance(Node n1, Node n2);

#endif
