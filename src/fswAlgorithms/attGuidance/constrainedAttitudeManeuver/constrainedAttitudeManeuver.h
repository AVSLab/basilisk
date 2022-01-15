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
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include <map>
#include <iostream>
#include <fstream>

/*! @brief waypoint reference module class */
class ConstrainedAttitudeManeuver: public SysModel {
public:
    ConstrainedAttitudeManeuver();
    ConstrainedAttitudeManeuver(int N);
    ~ConstrainedAttitudeManeuver(); 
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);


public:
    int N;                                       //!< Fineness level of discretization

};

//! @brief The Node class is used to create nodes in the 3D MRP graph

class Node {
public:
    Node();
    Node(double sigma_BN[3]); //, double keepOutFov, double keepOutBoresight[3], double keepInFov, double keepInBoresight[3]);
    ~Node();

    double sigma_BN[3];
    bool isBoundary;
    bool isFree;
    double heuristic;
    double priority;

};

class NodeProperties {
public:
    NodeProperties();
    ~NodeProperties();

    Node neighbors[26];
    Node path[20];
};

void mirrorFunction(int indices[3], int mirrorIndices[8][3]);

void neighboringNodes(int indices[3], int neighbors[26][3]);

double distance(Node n1, Node n2);

void generateGrid(Node startNode, Node goalNode, int N, std::map<int,std::map<int,std::map<int,Node>>> NodesMap, std::map<int,std::map<int,std::map<int,Node>>> NodePropertiesMap);

#endif
