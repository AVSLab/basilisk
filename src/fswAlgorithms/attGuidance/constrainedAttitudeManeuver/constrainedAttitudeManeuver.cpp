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
#include "fswAlgorithms/attGuidance/constrainedAttitudeManeuver/constrainedAttitudeManeuver.h"
#include <map>
#include <sstream>
#include <string>
#include <string.h>
#include <math.h>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
ConstrainedAttitudeManeuver::ConstrainedAttitudeManeuver()
{
    return;
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
ConstrainedAttitudeManeuver::ConstrainedAttitudeManeuver(int N)
{
    this->N = N;
    this->scStateMsgBuffer = this->scStateInMsg.zeroMsgPayload;
	this->celBodyMsgBuffer = this->celBodyInMsg.zeroMsgPayload;

    return;
}

/*! Module Destructor.  */
ConstrainedAttitudeManeuver::~ConstrainedAttitudeManeuver()
{
    return;
}

/*! This method is used to reset the module.
 @return void
 */
void ConstrainedAttitudeManeuver::Reset(uint64_t CurrentSimNanos)
{
	ReadInputs();

	Node startNode = Node(this->scStateMsgBuffer.sigma_BN, this->constraints, this->keepOutFov, this->keepOutBore_B);
	Node goalNode = Node(this->sigma_BN_goal, this->constraints, this->keepOutFov, this->keepOutBore_B);
	GenerateGrid(startNode, goalNode);
	// Node *path[20];
	// AStar(path);
    return;

}

/*! This method is the state update.
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void ConstrainedAttitudeManeuver::UpdateState(uint64_t CurrentSimNanos)
{
    return;
}

/*! This method reads the input messages in from the system and sets the
 appropriate parameters
 @return void
 */
void ConstrainedAttitudeManeuver::ReadInputs()
{
    //! - Read the input messages into the correct pointer
	if (this->scStateInMsg.isWritten()) {
		this->scStateMsgBuffer = this->scStateInMsg();
	}
	else {
		std::cout << "SCStateMsg not connected \n";
	}
	if (this->celBodyInMsg.isWritten()) {
		this->celBodyMsgBuffer = this->celBodyInMsg();
	}
	else {
		std::cout << "celBodyMsg not connected \n";
	}

	//! - Compute the inertial direction of the object w.r.t. the S/C
    double relPosVector[3];
	v3Subtract(this->celBodyMsgBuffer.PositionVector, this->scStateMsgBuffer.r_BN_N, relPosVector);
	v3Normalize(relPosVector, this->constraints.keepOutDir_N);
}

/*! This method generates the MRP grid and connects the free neighboring nodes
 @return void
 */
void ConstrainedAttitudeManeuver::GenerateGrid(Node startNode, Node goalNode)
{
	int N = this->N;
    double u[20];
	for (int n = 0; n < N; n++) {
		u[n] = n / ((double)N - 1);
	}
	// add internal nodes (|sigma_BN| < 1)
	int indices[3], mirrorIndices[8][3];
	double sigma_BN[3];
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			for (int k = 0; k < N; k++) {
				if (pow(u[i],2) + pow(u[j],2) + pow(u[k],2) < 1) {
					indices[0] = i; indices[1] = j; indices[2] = k;
					mirrorFunction(indices, mirrorIndices);
					for (int m = 0; m < 8; m++) {
						if (this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(mirrorIndices[m][2]) == 0) {
							for (int p = 0; p < 3; p++) {
								if (indices[p] != 0) { sigma_BN[p] = mirrorIndices[m][p]/indices[p]*u[indices[p]]; } else { sigma_BN[p] = 0; }
							}
							this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] = Node(sigma_BN, this->constraints,
							                                                                               this->keepOutFov, this->keepOutBore_B);
						}
					}
				}
			}
		}
	}
	// add boundary nodes (|sigma_BN| = 1)
	double r;
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			for (int k = 0; k < N; k++) {
				sigma_BN[0] = u[i]; sigma_BN[1] = u[j]; sigma_BN[2] = u[k];
				r = v3Norm(sigma_BN);
				// along i direction
				if (this->NodesMap[i][j].count(k) == 1 && this->NodesMap[i+1][j].count(k) == 0 && r < 1) {
					indices[0] = i+1; indices[1] = j; indices[2] = k;
					mirrorFunction(indices, mirrorIndices);
					for (int m = 0; m < 8; m++) {
						if (this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(mirrorIndices[m][2]) == 0) {
							sigma_BN[0] = mirrorIndices[m][0]/indices[0] * pow(1-pow(u[j],2)-pow(u[k],2),0.5);
							if (indices[1] != 0) { sigma_BN[1] = mirrorIndices[m][1]/indices[1]*u[indices[1]]; } else { sigma_BN[1] = 0; }
							if (indices[2] != 0) { sigma_BN[2] = mirrorIndices[m][2]/indices[2]*u[indices[2]]; } else { sigma_BN[2] = 0; }
							this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] = Node(sigma_BN, this->constraints,
							                                                                               this->keepOutFov, this->keepOutBore_B);
						}
					}
				}
				// along j direction
				if (this->NodesMap[i][j].count(k) == 1 && this->NodesMap[i][j+1].count(k) == 0 && r < 1) {
					indices[0] = i; indices[1] = j+1; indices[2] = k;
					mirrorFunction(indices, mirrorIndices);
					for (int m = 0; m < 8; m++) {
						if (this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(mirrorIndices[m][2]) == 0) {
							if (indices[0] != 0) { sigma_BN[0] = mirrorIndices[m][0]/indices[0]*u[indices[0]]; } else { sigma_BN[0] = 0; }
							sigma_BN[1] = mirrorIndices[m][1]/indices[1] * pow(1-pow(u[i],2)-pow(u[k],2),0.5);
							if (indices[2] != 0) { sigma_BN[2] = mirrorIndices[m][2]/indices[2]*u[indices[2]]; } else { sigma_BN[2] = 0; }
							this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] = Node(sigma_BN, this->constraints,
							                                                                               this->keepOutFov, this->keepOutBore_B);
						}
					}
				}
				// along k direction
				if (this->NodesMap[i][j].count(k) == 1 && this->NodesMap[i][j].count(k+1) == 0 && r < 1) {
					indices[0] = i; indices[1] = j; indices[2] = k+1;
					mirrorFunction(indices, mirrorIndices);
					for (int m = 0; m < 8; m++) {
						if (this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(mirrorIndices[m][2]) == 0) {
							if (indices[0] != 0) { sigma_BN[0] = mirrorIndices[m][0]/indices[0]*u[indices[0]]; } else { sigma_BN[0] = 0; }
							if (indices[1] != 0) { sigma_BN[1] = mirrorIndices[m][1]/indices[1]*u[indices[1]]; } else { sigma_BN[1] = 0; }
							sigma_BN[2] = mirrorIndices[m][2]/indices[2] * pow(1-pow(u[i],2)-pow(u[j],2),0.5);
							this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] = Node(sigma_BN, this->constraints,
							                                                                               this->keepOutFov, this->keepOutBore_B);
						}
					}
				}
			}
		} 
	}
	// link nodes to adjacent neighbors
	int neighbors[26][3];
	for (std::map<int,std::map<int,std::map<int,Node>>>::iterator it1=this->NodesMap.begin(); it1!=this->NodesMap.end(); it1++) {
		for (std::map<int,std::map<int,Node>>::iterator it2=it1->second.begin(); it2!=it1->second.end(); it2++) {
			for (std::map<int,Node>::iterator it3=it2->second.begin(); it3!=it2->second.end(); it3++) {
				indices[0] = it1->first; indices[1] = it2->first; indices[2] = it3->first;
				neighboringNodes(indices, neighbors);
				for (int n = 0; n < 26; n++) {
					if (this->NodesMap[neighbors[n][0]][neighbors[n][1]].count(neighbors[n][2]) == 1) {
						if (this->NodesMap[indices[0]][indices[1]][indices[2]].isFree && this->NodesMap[neighbors[n][0]][neighbors[n][1]][neighbors[n][2]].isFree) {
							this->NodesMap[indices[0]][indices[1]][indices[2]].appendNeighbor(&this->NodesMap[neighbors[n][0]][neighbors[n][1]][neighbors[n][2]]);
						}
					}
				}
			}
		}
	}
	// link boundary nodes to neighbors of shadow set
	bool flag;
	for (std::map<int,std::map<int,std::map<int,Node>>>::iterator it1=this->NodesMap.begin(); it1!=this->NodesMap.end(); it1++) {
		for (std::map<int,std::map<int,Node>>::iterator it2=it1->second.begin(); it2!=it1->second.end(); it2++) {
			for (std::map<int,Node>::iterator it3=it2->second.begin(); it3!=it2->second.end(); it3++) {
				if (it3->second.isBoundary && it3->second.isFree) {
					int i, j, k;
					i = it1->first;  j = it2->first;  k = it3->first;
					for (int n = 0; n < this->NodesMap[-i][-j][-k].neighborCount; n++) {
						flag = true;
						for (int m = 0; m < this->NodesMap[i][j][k].neighborCount; m++) {
							if (this->NodesMap[-i][-j][-k].neighbors[n] == this->NodesMap[i][j][k].neighbors[m]) {
								flag = false;
							}
						}
						if (flag == true) {
							it3->second.appendNeighbor(this->NodesMap[-i][-j][-k].neighbors[n]);
						}
					}
				}
			}
		}
	}
    /*
	for (std::map<int,std::map<int,std::map<int,Node>>>::iterator it1=NodesMap.begin(); it1!=NodesMap.end(); it1++) {
		for (std::map<int,std::map<int,Node>>::iterator it2=it1->second.begin(); it2!=it1->second.end(); it2++) {
			for (std::map<int,Node>::iterator it3=it2->second.begin(); it3!=it2->second.end(); it3++) {
				int i, j, k;
				i = it1->first; j = it2->first; k = it3->first;
				std::cout << NodesMap[i][j][k].sigma_BN[0] << " " << NodesMap[i][j][k].sigma_BN[1] << " " << NodesMap[i][j][k].sigma_BN[2] << " : \n";
				for (int n = 0; n < NodesMap[i][j][k].neighborCount; n++) {
					std::cout << NodesMap[i][j][k].neighbors[n]->sigma_BN[0] << " " << NodesMap[i][j][k].neighbors[n]->sigma_BN[1] << " " << NodesMap[i][j][k].neighbors[n]->sigma_BN[2] << "\n";
				}
			}
		}
	} */
    
	// add start and goal node to grid and connecting them to the neighboring nodes:
	double ds = 10;
	double dg = 10;
	double d1, d2;
	int keyS[3];
	int keyG[3];
	for (std::map<int,std::map<int,std::map<int,Node>>>::iterator it1=this->NodesMap.begin(); it1!=this->NodesMap.end(); it1++) {
		for (std::map<int,std::map<int,Node>>::iterator it2=it1->second.begin(); it2!=it1->second.end(); it2++) {
			for (std::map<int,Node>::iterator it3=it2->second.begin(); it3!=it2->second.end(); it3++) {
				d1 = distance(startNode, it3->second);
				if (d1 < ds) {
					ds = d1;
					keyS[0] = it1->first;   keyS[1] = it2->first;   keyS[2] = it3->first;
				}
				d2 = distance(goalNode, it3->second);
				if (d2 < dg) {
					dg = d2;
					keyG[0] = it1->first;   keyG[1] = it2->first;   keyG[2] = it3->first;
				}
			}
		}
	}
	for (int n = 0; n < this->NodesMap[keyS[0]][keyS[1]][keyS[2]].neighborCount; n++) {
		startNode.appendNeighbor(this->NodesMap[keyS[0]][keyS[1]][keyS[2]].neighbors[n]);
	}
	for (int n = 0; n < this->NodesMap[keyG[0]][keyG[1]][keyG[2]].neighborCount; n++) {
		this->NodesMap[keyG[0]][keyG[1]][keyG[2]].neighbors[n]->appendNeighbor(&goalNode);
	}
	this->NodesMap[keyS[0]][keyS[1]][keyS[2]] = startNode;
	this->NodesMap[keyG[0]][keyG[1]][keyG[2]] = goalNode;
	this->keyS[0] = keyS[0]; this->keyS[1] = keyS[1]; this->keyS[2] = keyS[2];
	this->keyG[0] = keyG[0]; this->keyG[1] = keyG[1]; this->keyG[2] = keyG[2];

}

/*! This method applies standard distance-based A* to find a valid path
 @return void
 */
void ConstrainedAttitudeManeuver::AStar(Node *path[20])
{
	for (std::map<int,std::map<int,std::map<int,Node>>>::iterator it1=NodesMap.begin(); it1!=NodesMap.end(); it1++) {
		for (std::map<int,std::map<int,Node>>::iterator it2=it1->second.begin(); it2!=it1->second.end(); it2++) {
			for (std::map<int,Node>::iterator it3=it2->second.begin(); it3!=it2->second.end(); it3++) {
				it3->second.heuristic = distance(it3->second, NodesMap[this->keyG[0]][this->keyG[1]][this->keyG[2]]);
			}
		}
	}

	NodeList O, C;
	O.append(&NodesMap[this->keyS[0]][this->keyS[1]][this->keyS[2]]);
    int n = 0;
	while (O.list[0] != &NodesMap[this->keyG[0]][this->keyG[1]][this->keyG[2]] && n < 1000) {
		n += 1;
		C.append(O.list[0]);
	}

}

/*! This is the constructor for the Node class.  It sets default variable
    values and initializes the various parts of the model */
Node::Node()
{
    return;
}

/*! The constructor requires the MRP set */
Node::Node(double sigma_BN[3], constraintStruct constraints, 
           double keepOutFov, double keepOutBore_B[3]) //, double keepOutFov, double keepOutBoresight[3], double keepInFov, double keepInBoresight[3])
{
    MRPswitch(sigma_BN, 1, this->sigma_BN);
	this->isBoundary = false;
	if (abs(v3Norm(this->sigma_BN) - 1) < 1e-5) {
		this->isBoundary = true;
	}
	double BN[3][3];
	MRP2C(this->sigma_BN, BN);
	v3tMultM33(keepOutBore_B, BN, this->keepOutBore_N);
	if ( v3Dot(this->keepOutBore_N, constraints.keepOutDir_N) >= cos(keepOutFov) ) {
		this->isFree = false;
	}
	else {
		this->isFree = true;
	}
	this->heuristic = 0;
	this->priority = 0;
	this->neighborCount = 0;
	this->pathCount = 0;

    return;
}

/*! Module Destructor.  */
Node::~Node()
{
    return;
}

/*! This method appends a pointer to neighboring node to the neighbors class variable */
void Node::appendNeighbor(Node *node)
{
	this->neighbors[this->neighborCount] = node;
	this->neighborCount += 1;
    return;
}

/*! This method appends a pointer to neighboring node to the neighbors class variable */
void Node::appendPathNode(Node *node)
{
	this->neighbors[this->pathCount] = node;
	this->pathCount += 1;
    return;
}

void mirrorFunction(int indices[3], int mirrorIndices[8][3]) 
{
	mirrorIndices[0][0] =  indices[0];   mirrorIndices[0][1] =  indices[1];   mirrorIndices[0][2] =  indices[2];
	mirrorIndices[1][0] = -indices[0];   mirrorIndices[1][1] =  indices[1];   mirrorIndices[1][2] =  indices[2];
	mirrorIndices[2][0] =  indices[0];   mirrorIndices[2][1] = -indices[1];   mirrorIndices[2][2] =  indices[2];
	mirrorIndices[3][0] =  indices[0];   mirrorIndices[3][1] =  indices[1];   mirrorIndices[3][2] = -indices[2];
	mirrorIndices[4][0] = -indices[0];   mirrorIndices[4][1] = -indices[1];   mirrorIndices[4][2] =  indices[2];
	mirrorIndices[5][0] = -indices[0];   mirrorIndices[5][1] =  indices[1];   mirrorIndices[5][2] = -indices[2];
	mirrorIndices[6][0] =  indices[0];   mirrorIndices[6][1] = -indices[1];   mirrorIndices[6][2] = -indices[2];
	mirrorIndices[7][0] = -indices[0];   mirrorIndices[7][1] = -indices[1];   mirrorIndices[7][2] = -indices[2];
}

void neighboringNodes(int indices[3], int neighbors[26][3])
{
	neighbors[0][0]  = indices[0]-1;     neighbors[0][1]  = indices[1];       neighbors[0][2]  = indices[2];
	neighbors[1][0]  = indices[0]+1;     neighbors[1][1]  = indices[1];       neighbors[1][2]  = indices[2];
	neighbors[2][0]  = indices[0];       neighbors[2][1]  = indices[1]-1;     neighbors[2][2]  = indices[2];
	neighbors[3][0]  = indices[0];       neighbors[3][1]  = indices[1]+1;     neighbors[3][2]  = indices[2];
	neighbors[4][0]  = indices[0];       neighbors[4][1]  = indices[1];       neighbors[4][2]  = indices[2]-1;
	neighbors[5][0]  = indices[0];       neighbors[5][1]  = indices[1];       neighbors[5][2]  = indices[2]+1;
	neighbors[6][0]  = indices[0]-1;     neighbors[6][1]  = indices[1]-1;     neighbors[6][2]  = indices[2];
	neighbors[7][0]  = indices[0]+1;     neighbors[7][1]  = indices[1]-1;     neighbors[7][2]  = indices[2];
	neighbors[8][0]  = indices[0]-1;     neighbors[8][1]  = indices[1]+1;     neighbors[8][2]  = indices[2];
	neighbors[9][0]  = indices[0]+1;     neighbors[9][1]  = indices[1]+1;     neighbors[9][2]  = indices[2];
	neighbors[10][0] = indices[0]-1;     neighbors[10][1] = indices[1];       neighbors[10][2] = indices[2]-1;
	neighbors[11][0] = indices[0]+1;     neighbors[11][1] = indices[1];       neighbors[11][2] = indices[2]-1;
    neighbors[12][0] = indices[0]-1;     neighbors[12][1] = indices[1];       neighbors[12][2] = indices[2]+1;
	neighbors[13][0] = indices[0]+1;     neighbors[13][1] = indices[1];       neighbors[13][2] = indices[2]+1;
	neighbors[14][0] = indices[0];       neighbors[14][1] = indices[1]-1;     neighbors[14][2] = indices[2]-1;
	neighbors[15][0] = indices[0];       neighbors[15][1] = indices[1]+1;     neighbors[15][2] = indices[2]-1;
	neighbors[16][0] = indices[0];       neighbors[16][1] = indices[1]-1;     neighbors[16][2] = indices[2]+1;
	neighbors[17][0] = indices[0];       neighbors[17][1] = indices[1]+1;     neighbors[17][2] = indices[2]+1;
	neighbors[18][0] = indices[0]-1;     neighbors[18][1] = indices[1]-1;     neighbors[18][2] = indices[2]-1;
	neighbors[19][0] = indices[0]+1;     neighbors[19][1] = indices[1]-1;     neighbors[19][2] = indices[2]-1;
    neighbors[20][0] = indices[0]-1;     neighbors[20][1] = indices[1]+1;     neighbors[20][2] = indices[2]-1;
	neighbors[21][0] = indices[0]-1;     neighbors[21][1] = indices[1]-1;     neighbors[21][2] = indices[2]+1;
	neighbors[22][0] = indices[0]+1;     neighbors[22][1] = indices[1]+1;     neighbors[22][2] = indices[2]-1;
	neighbors[23][0] = indices[0]+1;     neighbors[23][1] = indices[1]-1;     neighbors[23][2] = indices[2]+1;
	neighbors[24][0] = indices[0]-1;     neighbors[24][1] = indices[1]+1;     neighbors[24][2] = indices[2]+1;
	neighbors[25][0] = indices[0]+1;     neighbors[25][1] = indices[1]+1;     neighbors[25][2] = indices[2]+1;
}

double distance(Node n1, Node n2)
{
	double n1n, n2n;
	double D, d[4];
	double dn[3], n1s[3], n2s[3];
	n1n = v3Norm(n1.sigma_BN);
	n2n = v3Norm(n2.sigma_BN);
	v3Subtract(n1.sigma_BN, n2.sigma_BN, dn);
	d[0] = v3Norm(dn);
	if (n2n > 1e-8) {
		MRPshadow(n2.sigma_BN, n2s);
		v3Subtract(n1.sigma_BN, n2s, dn);
		d[1] = v3Norm(dn);
	}
	else {
		d[1] = d[0];
	}
	if (n1n > 1e-8) {
		MRPshadow(n1.sigma_BN, n1s);
		v3Subtract(n2.sigma_BN, n1s, dn);
		d[2] = v3Norm(dn);
	}
	else {
		d[2] = d[0];
	}
	if (n1n > 1e-8 || n2n > 1e-8) {
		v3Subtract(n1s, n2s, dn);
		d[3] = v3Norm(dn);
	}
	else {
		d[3] = d[0];
	}
	D = d[0];
    for (int i = 1; i < 4; i++) {
        if (d[i] < D) { D = d[i]; }
    }
	return D;
}

/*! This is the constructor for the NodeList class. */
NodeList::NodeList()
{
	this-> N = 0;
    return;
}

/*! Class Destructor. */
NodeList::~NodeList()
{
    return;
}

void NodeList::append(Node* node)
{
	this->list[this->N] = node;
	this->N += 1;
}

void NodeList::pop(int M)
{
	for (int m = M; m < N-1; m++) {
		this->list[m] = this->list[m+1];
	}
	this->N -= 1;
}

void NodeList::swap(int m, int n)
{
	Node *p1 = this->list[m];
	Node *p2 = this->list[n];
	this->list[m] = p2;
	this->list[n] = p1;
}

void NodeList::sort()
{
	int M;
	double p;
	for (int n = 0; n < this->N; n++) {
		p = 1e5;
		M = N-1;
		for (int m = n; m < this->N; m++) {
			if (this->list[m]->priority < p) {
				p = this->list[m]->priority;
				M = m;
			}
		}
		swap(n, M);
	}
}