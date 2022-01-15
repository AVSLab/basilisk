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

	std::map<int,std::map<int,std::map<int,Node>>> NodesMap;
	std::map<int,std::map<int,std::map<int,Node>>> NodePropertiesMap;
	double s0[3], sN[3];
	s0[0] = 0; s0[1] = 0; s0[2] = 0;
	sN[0] = 1; sN[1] = 0; sN[2] = 0;
	Node startNode = Node(s0);
	Node goalNode = Node(sN);
	generateGrid(startNode, goalNode, this->N, NodesMap, NodePropertiesMap);

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

/*! This is the constructor for the Node class.  It sets default variable
    values and initializes the various parts of the model */
Node::Node()
{
    return;
}

/*! The constructor requires the MRP set */
Node::Node(double sigma_BN[3]) //, double keepOutFov, double keepOutBoresight[3], double keepInFov, double keepInBoresight[3])
{
    MRPswitch(sigma_BN, 1, this->sigma_BN);
	this->isFree = true;
	this->isBoundary = false;
	if (abs(v3Norm(this->sigma_BN) - 1) < 1e-5) {
		this->isBoundary = true;
	}
	this->heuristic = 0;
	this->priority = 0;

    return;
}

/*! Module Destructor.  */
Node::~Node()
{
    return;
}

/*! This is the constructor for the Node class.  It sets default variable
    values and initializes the various parts of the model */
NodeProperties::NodeProperties()
{
    return;
}

/*! Module Destructor.  */
NodeProperties::~NodeProperties()
{
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

void generateGrid(Node startNode, Node goalNode, int N, 
     std::map<int,std::map<int,std::map<int,Node>>> NodesMap, 
	 std::map<int,std::map<int,std::map<int,Node>>> NodePropertiesMap)
{
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
				// std::cout << i << " " << j << " " << k << "\n";
				if (pow(u[i],2) + pow(u[j],2) + pow(u[k],2) < 1) {
					indices[0] = i; indices[1] = j; indices[2] = k;
					mirrorFunction(indices, mirrorIndices);
					for (int m = 0; m < 8; m++) {
						if (NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(mirrorIndices[m][2]) == 0) {
							for (int p = 0; p < 3; p++) {
								if (indices[p] != 0) { sigma_BN[p] = mirrorIndices[m][p]/indices[p]*u[indices[p]]; } else { sigma_BN[p] = 0; }
							}
							// std::cout << sigma_BN[0] << " " << sigma_BN[1] << " " << sigma_BN[2] << "\n";
							NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] = Node(sigma_BN);
						} /*
						std::cout << NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]].sigma_BN[0] << " " 
						          << NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]].sigma_BN[1] << " " 
								  << NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]].sigma_BN[2] << "\n"; */
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
				if (NodesMap[i][j].count(k) == 1 && NodesMap[i+1][j].count(k) == 0 && r < 1) {
					indices[0] = i+1; indices[1] = j; indices[2] = k;
					mirrorFunction(indices, mirrorIndices);
					for (int m = 0; m < 8; m++) {
						if (NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(mirrorIndices[m][2]) == 0) {
							sigma_BN[0] = mirrorIndices[m][0]/indices[0] * pow(1-pow(u[j],2)-pow(u[k],2),0.5);
							if (indices[1] != 0) { sigma_BN[1] = mirrorIndices[m][1]/indices[1]*u[indices[1]]; } else { sigma_BN[1] = 0; }
							if (indices[2] != 0) { sigma_BN[2] = mirrorIndices[m][2]/indices[2]*u[indices[2]]; } else { sigma_BN[2] = 0; }
							NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] = Node(sigma_BN);
						}
					}
				}
				// along j direction
				if (NodesMap[i][j].count(k) == 1 && NodesMap[i][j+1].count(k) == 0 && r < 1) {
					indices[0] = i; indices[1] = j+1; indices[2] = k;
					mirrorFunction(indices, mirrorIndices);
					for (int m = 0; m < 8; m++) {
						if (NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(mirrorIndices[m][2]) == 0) {
							if (indices[0] != 0) { sigma_BN[0] = mirrorIndices[m][0]/indices[0]*u[indices[0]]; } else { sigma_BN[0] = 0; }
							sigma_BN[1] = mirrorIndices[m][1]/indices[1] * pow(1-pow(u[i],2)-pow(u[k],2),0.5);
							if (indices[2] != 0) { sigma_BN[2] = mirrorIndices[m][2]/indices[2]*u[indices[2]]; } else { sigma_BN[2] = 0; }
							NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] = Node(sigma_BN);
						}
					}
				}
				// along k direction
				if (NodesMap[i][j].count(k) == 1 && NodesMap[i][j].count(k+1) == 0 && r < 1) {
					indices[0] = i; indices[1] = j; indices[2] = k+1;
					mirrorFunction(indices, mirrorIndices);
					for (int m = 0; m < 8; m++) {
						if (NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(mirrorIndices[m][2]) == 0) {
							if (indices[0] != 0) { sigma_BN[0] = mirrorIndices[m][0]/indices[0]*u[indices[0]]; } else { sigma_BN[0] = 0; }
							if (indices[1] != 0) { sigma_BN[1] = mirrorIndices[m][1]/indices[1]*u[indices[1]]; } else { sigma_BN[1] = 0; }
							sigma_BN[2] = mirrorIndices[m][2]/indices[2] * pow(1-pow(u[i],2)-pow(u[j],2),0.5);
							NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] = Node(sigma_BN);
						}
					}
				}
			}
		} 
	}

	for (std::map<int,std::map<int,std::map<int,Node>>>::iterator it1=NodesMap.begin(); it1!=NodesMap.end(); it1++) {
		for (std::map<int,std::map<int,Node>>::iterator it2=it1->second.begin(); it2!=it1->second.end(); it2++) {
			for (std::map<int,Node>::iterator it3=it2->second.begin(); it3!=it2->second.end(); it3++) {
				std::cout << it1->first << " " << it2->first << " " << it3->first << ": "
				          << it3->second.sigma_BN[0] << " " << it3->second.sigma_BN[1] << " " << it3->second.sigma_BN[2] << "\n";
			}
		} 
	}

	return;
}