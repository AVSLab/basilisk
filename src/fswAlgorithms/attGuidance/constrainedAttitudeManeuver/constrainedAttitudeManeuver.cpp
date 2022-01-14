/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
    this->N = 7;

    return;
}

/*! Module Destructor.  */
ConstrainedAttitudeManeuver::~ConstrainedAttitudeManeuver()
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
Node::Node(double sigma_BN[3], double keepOutFov, double keepOutBoresight[3], double keepInFov, double keepInBoresight[3])
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

void mirrorFunction(double indices[3], double mirrorIndices[8][3]) 
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

void neighboringNodes(double indices[3], double neighbors[26][3])
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

void generateGrid(Node startNode, Node goalNode, int N, std::map<const char*,Node> NodesMap, std::map<const char*,NodeProperties> NodePropertiesMap)
{
    double u[20];
	for (int n = 0; n < N; n++) {
		u[n] = n / (N - 1);
	}
    
	double indices[3], mirrorIndices[8][3];
	for (int i = 0; i > N; i++) {
		for (int j = 0; j > N; j++) {
			for (int k = 0; k > N; k++) {
				if (pow(u[i],2) + pow(u[j],2) + pow(u[k],2) < 1) {
					indices[0] = i; indices[1] = j; indices[2] = k;
					mirrorFunction(indices, mirrorIndices);
					for (int m = 0; m < 8; m++) {
						std::stringstream tmp;
						tmp << mirrorIndices[m][0] << " " << mirrorIndices[m][1] << " " << mirrorIndices[m][2];
						const char* key = tmp.str().c_str();
						if (NodesMap.count(key) == 0) {
							std::cout << key;
						}
					}
				}
			}
		}
	}

	return;
}