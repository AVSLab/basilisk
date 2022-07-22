/*
ISC License

Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#ifndef RigidBodyContactEffector_h
#define RigidBodyContactEffector_h

#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include <set>
#include <iostream>
#include <stack>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <tiny_obj_loader.h>
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SCMassPropsMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/rigidBodyKinematics.h"

/*! Struct for holding edge penetration information.*/
typedef struct{
    std::vector<std::vector<int>> edgePair;     //!< -- Vertex indicies for pair of intersecting edges
    double separation;                          //!< [m] Distance between edges
}edgeQuery;

/*! Struct for holding face/vertex penetration information.*/
typedef struct{
    int faceIndex;                              //!< -- Identifier for which face is being intersected
    int supportIndex;                           //!< -- Index for penetrating vertex
    double separation;                          //!< [m] Distance between vertex and face
}faceQuery;

/*! Struct for holding information about a collision.*/
typedef struct{
    double timeToContact;                       //!< [s] Time until collision
    double area;                                //!< [m^2] Area of collision face
    int otherBodyIndex;                         //!< -- Index of external body being collided with
    Eigen::Vector3d mainContactPoint;           //!< -- Point of contact on main body
    Eigen::Vector3d otherContactPoint;          //!< -- Point of contact of external body
    Eigen::Vector3d contactNormal;              //!< -- Normal vector of collision (from main body)
    Eigen::Matrix3d dcm_CB;
    std::vector<Eigen::Matrix3d> MSCPntC_B;
    double critSlideDir;
    double critCoeffFric;
    double slipReverseDir;
    bool slipHitZero;
    Eigen::Vector3d force_N;
    Eigen::Vector3d torque_B;
    double penetrationEnergy;
}contactDetail;

/*! Struct for holding information about a penetration.*/
typedef struct{
    int otherBodyIndex;                         //!< -- Index of external body being penetrated
    int contactCase;                            //!< -- Type of penetration (face/vertex, vertex/face, edge/edge)
    Eigen::Vector3d mainContactPoint;           //!< -- Point of contact on main body
    Eigen::Vector3d otherContactPoint;          //!< -- Point of contact on external body
    Eigen::Vector3d springLine_N;               //!< -- Direction of restoring force
    faceQuery faceData;                         //!< -- Face penetration information
    edgeQuery edgeData;                         //!< -- Edge penetration information
}penetrationDetail;

typedef struct{
    std::tuple<int, int> parentIndices;
    std::vector<std::tuple<int, int, int, int>> overlaps;
}boundingBoxDetail;

/*! Struct for holding the bounds of a vector over a time interval.*/
typedef struct{
    Eigen::Vector3d lower;                      //!< -- Lower time bound
    Eigen::Vector3d upper;                      //!< -- Upper time bound
}vectorInterval;

typedef struct{
    vectorInterval xAxisInterval;
    vectorInterval yAxisInterval;
    vectorInterval zAxisInterval;
    Eigen::Vector3d halfSize;
}indivBoundingBox;

/*! Struct for holding primitive information of a single polyhedron in a half edge format*/
typedef struct{
    std::vector<Eigen::Vector3d> faceNormals;   //!< -- Normal vectors of each face
    std::vector<std::vector<int>> faceTriangles;//!< -- Indices for vertices of each triangle
    std::vector<Eigen::Vector3d> faceCentroids;
    std::vector<Eigen::Vector3d> faceBoundingBoxes;
    std::vector<double> faceBoundingRadius;
    std::vector<std::vector<int>> edgeIndices;  //!< -- Indicies for the verticies of each edge
    std::vector<int> faceIndices;               //!< -- Indicies for each face connecting to an edge
    Eigen::Vector3d centroid;                   //!< [m] Centroid of the polyhedron
    std::vector<int> uniqueVertIndices;
    Eigen::Vector3d boundingBox;
}halfEdge;

/*! Struct for holding dynamics data of each body*/
typedef struct{
    Eigen::Vector3d r_BN_N;                     //!< [m] Position of body wrt to base
    Eigen::Vector3d v_BN_N;                     //!< [m/s] Velocity of body wrt to base
    Eigen::Vector3d nonConservativeAccelpntB_B;
    double m_SC;                                //!< [kg] Mass of body
    Eigen::MatrixXd ISCPntB_B;                  //!<  [kg m^2] Inertia of body about point B in that body's frame
    Eigen::MatrixXd ISCPntB_B_inv;
    Eigen::Vector3d c_B;                        //!< [m] Vector from point B to CoM of body in body's frame
    Eigen::Vector3d omega_BN_B;                 //!< [r/s] Attitude rate of the body wrt base
    Eigen::Vector3d omegaDot_BN_B;
    Eigen::Matrix3d omegaTilde_BN_B;
    Eigen::MRPd sigma_BN;                       //!< -- Attitude of the body wrt base
    Eigen::MRPd sigma_BprimeB;                  //!< -- Linearly propegated attitude of the body wrt base
    Eigen::Matrix3d dcm_BprimeB;
    Eigen::Matrix3d dcm_BN;
    Eigen::Matrix3d dcm_NB;                        
}dynamicData;

/*! Struct for holding the linked states of each body*/
typedef struct{
    StateData *hubPosition;
    StateData *hubVelocity;
    StateData *hubSigma;
    StateData *hubOmega_BN_N;
    Eigen::MatrixXd *r_BN_N;
    Eigen::MatrixXd *v_BN_N;
    Eigen::MatrixXd *m_SC;
    Eigen::MatrixXd *ISCPntB_B;
    Eigen::MatrixXd *c_B;
}linkedStates;

/*! Struct for holding all required information of each body*/
typedef struct{
    double boundingRadius;                      //!< [m] Radius of body bounding sphere
    double coefRestitution;                     //!< -- Coefficient of Restitution between external body and main body
    double coefFriction;
    double springConstant;                      //!< -- Spring constant between external body and main body
    double dampingConstant;                     //!< -- Damping constant between external body and main body
    std::string objFile;                        //!< -- File name for the .obj file pertaining to body
    tinyobj::attrib_t attrib;                   //!< -- Attribute conversion from TinyOBJLoader
    std::vector<Eigen::Vector3d> vertices;      //!< -- All verticies in the body
    std::vector<tinyobj::shape_t> shapes;       //!< -- Polyhedra data from TinyOBJLoader
    std::vector<halfEdge> polyhedron;           //!< -- Half edge converted polyhedra data
    std::vector<std::vector<contactDetail>> collisionPoints;//!< -- Current collision data
    std::vector<penetrationDetail> penetrationData;//!< -- Current penetration data
    boundingBoxDetail coarseSearchList;
    linkedStates hubState;                      //!< -- Linked states for the body
    dynamicData states;                         //!< -- Extracted states for the body
    dynamicData futureStates;
    std::string modelTag;                       //!< -- Name of body's model tag
    ReadFunctor<SpicePlanetStateMsgPayload> planetInMsg;
    SpicePlanetStateMsgPayload plMsg;
    bool isSpice;
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;
    ReadFunctor<SCMassPropsMsgPayload> scMassStateInMsg;
    SCStatesMsgPayload stateInBuffer;           //!< -- Body state buffer
    SCMassPropsMsgPayload massStateInBuffer;    //!< -- Body mass state buffer
    Eigen::Vector3d forceExternal_N;
    Eigen::Vector3d torqueExternalPntB_B;
}geometry;

/*! @brief Rigid Body Contact state effector class */
class RigidBodyContactEffector: public SysModel, public DynamicEffector
{
public:
    RigidBodyContactEffector();
    ~RigidBodyContactEffector();
    
    void Reset();
    void LoadSpacecraftBody(const char *objFile, std::string modelTag, Message<SCStatesMsgPayload> *scStateMsg, Message<SCMassPropsMsgPayload> *scMassStateMsg, double boundingRadius, double coefRestitution, double coefFriction);
    void AddSpiceBody(const char *objFile, Message<SpicePlanetStateMsgPayload> *planetSpiceMsg, double boundingRadius, double coefRestitution, double coefFriction);
    void linkInStates(DynParamManager& states);
    void computeForceTorque(double currentTime, double timeStep);
    void computeStateContribution(double integTime);
    void UpdateState(uint64_t CurrentSimNanos);
    void ReadInputs();
    void ExtractFromBuffer();
    void CheckBoundingSphere();
    void CheckBoundingBox();
    bool Overlap(geometry foriegnBody, int bodyIndex);
    
private:
    double currentSimSeconds;
    std::vector<std::vector<int>> closeBodies;               //!< -- Indicies of all external bodies that the main body is within the bounding sphere of
    int currentBodyInCycle;
    double currentMinError;
    bool responseFound;
    
public:
    geometry mainBody;
    
    std::vector<geometry> Bodies;
    int numBodies;
    int currentBody;
    bool isOverlap;
    bool overlapFace1;
    bool overlapFace2;
    bool overlapEdge;
    double maxPosError;
    double slipTolerance;
    double simTimeStep;
    double collisionIntegrationStep;
    double maxBoundingBoxDim;
    double minBoundingBoxDim;
    double boundingBoxFF;
    
    
    
    
private:
    Eigen::VectorXd CollisionStateDerivative(Eigen::VectorXd X_c, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>> impacts, std::vector<double> f_vals, std::vector<double> phi_vals, Eigen::MatrixXd M_tot, double coefRes, double coefFric);
    bool SeparatingPlane(vectorInterval displacementInterval, vectorInterval candidateInterval, indivBoundingBox box1, indivBoundingBox box2);
    bool IsMinkowskiFace(Eigen::Vector3d edgeA, Eigen::Vector3d edgeB, Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Eigen::Vector3d d);
    std::vector<halfEdge> ComputeHalfEdge(std::vector<Eigen::Vector3d> vertices, std::vector<tinyobj::shape_t> shapes);
    edgeQuery QueryEdgeDirection(std::vector<Eigen::Vector3d> verticesA, halfEdge polyA, dynamicData stateA, std::vector<Eigen::Vector3d> verticesB, halfEdge polyB, dynamicData stateB);
    faceQuery QueryFaceDirection(std::vector<Eigen::Vector3d> verticesA, halfEdge polyA, dynamicData stateA, std::vector<Eigen::Vector3d> verticesB, halfEdge polyB, dynamicData stateB);
    Eigen::Vector3d MakeIntervalValue(Eigen::Vector3d vertex_B, dynamicData dynamics, double time);
    vectorInterval MakeIntervalValues(Eigen::Vector3d vertex_B, dynamicData dynamics, double screwAngle, Eigen::Matrix3d screwRot, Eigen::Vector3d screwOffset, double screwDistance, std::vector<double> timeInterval);
    double FindEdgeIntervalThreshold(Eigen::Vector3d vertexA0_B, Eigen::Vector3d vertexA1_B, dynamicData dynamicsA, Eigen::Vector3d vertexB0_B, Eigen::Vector3d vertexB1_B, dynamicData dynamicsB, double maxError);
    double FindEdgeIntervalThresholds(Eigen::Vector3d vertexA0_B, Eigen::Vector3d vertexA1_B, dynamicData dynamicsA, double screwAngleA, Eigen::Matrix3d screwRotA, Eigen::Vector3d screwOffsetA, double screwDistanceA, Eigen::Vector3d vertexB0_B, Eigen::Vector3d vertexB1_B, dynamicData dynamicsB, double screwAngleB, Eigen::Matrix3d screwRotB, Eigen::Vector3d screwOffsetB, double screwDistanceB, double maxError);
    double FindFaceIntervalThreshold(Eigen::Vector3d faceVertex0_B, Eigen::Vector3d faceVertex1_B, Eigen::Vector3d faceVertex2_B, dynamicData dynamicsA, Eigen::Vector3d supportVertex_B, dynamicData dynamicsB, double maxError);
    std::vector<double> IntervalSine(double a, double b);
    std::vector<double> IntervalCosine(double a, double b);
    std::vector<double> IntervalDotProduct(vectorInterval vectorA, vectorInterval vectorB);
    double FindFaceIntervalThresholds(Eigen::Vector3d faceVertex0_B, Eigen::Vector3d faceVertex1_B, Eigen::Vector3d faceVertex2_B, dynamicData dynamicsA,  double screwAngleA, Eigen::Matrix3d screwRotA, Eigen::Vector3d screwOffsetA, double screwDistanceA, Eigen::Vector3d supportVertex_B, dynamicData dynamicsB,  double screwAngleB, Eigen::Matrix3d screwRotB, Eigen::Vector3d screwOffsetB, double screwDistanceB, double maxError);
    vectorInterval IntervalCrossProduct(vectorInterval vectorA, vectorInterval vectorB);
    int LineLineDistance(Eigen::Vector3d vertex1, Eigen::Vector3d vertex2, Eigen::Vector3d vertex3, Eigen::Vector3d vertex4, Eigen::Vector3d *pointA, Eigen::Vector3d *pointB);
    int PointInTriangle(Eigen::Vector3d supportPoint, Eigen::Vector3d triVertex0, Eigen::Vector3d triVertex1, Eigen::Vector3d triVertex2, Eigen::Vector3d *contactPoint, double *distance);
    double WhenEdgeContact(std::vector<std::vector<int>> edgePair, std::vector<Eigen::Vector3d> verticesA, std::vector<Eigen::Vector3d> verticesB, dynamicData dynamicsA, dynamicData dynamicsB, double dt, Eigen::Vector3d *edgeAContact_N, Eigen::Vector3d *edgeBContact_N, int *coLin);
    double WhenFaceContact(std::vector<int> trianglePoints, std::vector<Eigen::Vector3d> verticesA, int supportPoint, std::vector<Eigen::Vector3d> verticesB, dynamicData dynamicsA, dynamicData dynamicsB, double dt, Eigen::Vector3d *faceContactPoint_N);
    Eigen::Vector3d CalcImpluse(contactDetail collisionData, dynamicData otherDynamics, double coefRestitution);
    void C2Screw(Eigen::Matrix3d DCM, Eigen::Vector3d displacement, double *screwAngle, Eigen::Matrix3d *screwRot, Eigen::Vector3d *screwOffset, double *screwDistance);
    Eigen::Vector3d SecondTop(std::stack<Eigen::Vector3d> &stk);
    bool ComparePoints(const std::vector<Eigen::Vector3d> &point1, const std::vector<Eigen::Vector3d> &point2);
    std::vector<Eigen::Vector3d> findConvexHull(std::vector<Eigen::Vector3d> points);
};


#endif /* RigidBodyContactEffector_h */
