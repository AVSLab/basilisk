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



#include "RigidBodyContactEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/QR>




/*! This is the constructor, setting variables to default values */
RigidBodyContactEffector::RigidBodyContactEffector()
{
    this->isOverlap = false;
    this->mainBody.boundingRadius = 0.0;
    this->mainBody.collisionPoints.clear();
    return;
}

/*! This is the destructor, nothing to report here */
RigidBodyContactEffector::~RigidBodyContactEffector()
{
    return;
}


void RigidBodyContactEffector::Reset()
{
    
    this->forceExternal_N.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    
}

/*! This method uses TinyOBJLoader to load the primary body primitives.
@return void
@param objFile The .obj file associated with the primary body
 */
void RigidBodyContactEffector::LoadSpacecraftBody(const char *objFile, std::string modelTag, double boundingRadius, double coefRestitution, double coefFriction)
{
    geometry body;
    body.boundingRadius = boundingRadius;
    body.coefRestitution = coefRestitution;
    body.coefFriction = coefFriction;
    body.modelTag = modelTag;
    body.isSpice = false;
    
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;
    
    // - Use TinyOBJLoader
    bool ret = tinyobj::LoadObj(&body.attrib, &body.shapes, &materials, &err, objFile);
    // - Organize the vertices into a useful format
    for (int vertIt=0; vertIt<body.attrib.vertices.size()/3; ++vertIt)
    {
        Eigen::Vector3d v(body.attrib.vertices[3*vertIt + 0], body.attrib.vertices[3*vertIt + 1], body.attrib.vertices[3*vertIt + 2]);
        body.vertices.push_back(v);
    }
    // - Orgazine the shape information into a half edge format
    body.polyhedron = this->ComputeHalfEdge(body.vertices, body.shapes);
    this->Bodies.push_back(body);
    this->numBodies++;
    return;
}

/*! This method uses TinyOBJLoader to load non-primary body primitives.
@return void
@param objFile The .obj file associated with this non-primary body
@param modelTag Name of the model associated with this body
@param scStateInMsgName The SpacecraftPlus message name for this body
@param scMassStateInMsgName The mass properties message name for this body
@param boundingRadius The radius of this body's bounding sphere, for primary collision detection
@param coefRestitution The Coefficient of Restitution between this body and the primary body
*/
void RigidBodyContactEffector::AddSpiceBody(const char *objFile, Message<SpicePlanetStateMsgPayload> *planetSpiceMsg, double boundingRadius, double coefRestitution, double coefFriction)
{
    geometry body;
    body.boundingRadius = boundingRadius;
    body.coefRestitution = coefRestitution;
    body.coefFriction = coefFriction;
    body.planetInMsg = planetSpiceMsg->addSubscriber();
    body.isSpice = true;
    
//    externalBody.modelTag = modelTag;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;
    // - Use TinyOBJLoader
    bool ret = tinyobj::LoadObj(&body.attrib, &body.shapes, &materials, &err, objFile);
    // - Organize the vertices into a useful format
    for (int vertIt=0; vertIt<body.attrib.vertices.size()/3; ++vertIt)
    {
        Eigen::Vector3d v(body.attrib.vertices[3*vertIt + 0], body.attrib.vertices[3*vertIt + 1], body.attrib.vertices[3*vertIt + 2]);
        body.vertices.push_back(v);
    }
    // - Orgazine the shape information into a half edge format
    body.polyhedron = this->ComputeHalfEdge(body.vertices, body.shapes);
    // - Add this body to the list of all external bodies
    this->Bodies.push_back(body);
    this->numBodies++;
    return;
}

/*! This method organizes primitive information from a .obj file into the half edge format.
@return The body in half edge format
@param vertices An ordered list of all vertices in the body
@param shapes The .obj formatted information of each polyhedron in the body
*/
std::vector<halfEdge> RigidBodyContactEffector::ComputeHalfEdge(std::vector<Eigen::Vector3d> vertices, std::vector<tinyobj::shape_t> shapes)
{
    std::vector<halfEdge> polyhedron;
    halfEdge boundingGroup;
    int indexOffset;
    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d faceNormal;
    Eigen::Vector3d centroid;
    std::vector<int> tempTriangle;
    std::vector<int> searchEdge(2,0);
    std::vector<std::vector<int>> allFaces;
    std::vector<Eigen::Vector3d> allNormals;
    std::vector<Eigen::Vector3d> allBoundingBoxes;
    std::vector<Eigen::Vector3d> allCentroids;
    std::vector<std::vector<int>> allConnections;
    std::vector<int> unconnectedFaces;
    std::vector<int> ungroupedFaces;
    std::vector<int> facesInGroup;
    std::vector<Eigen::Vector3d> verticesInGroup;
    std::vector<Eigen::Vector3d> convHullPoints;
    std::vector<int> adjacentFacesToGroup;
    std::vector<int> dummyVec(3, -1);
    std::vector<double> faceMaxDist;
    std::vector<double> adjacentDistsToGroup;
    std::vector<double> indivMaxDists;
    int faceCounter = 0;
    bool searchingForCloseFace;
    double maxX;
    double maxY;
    double maxZ;
    
    for (int shapeIt=0; shapeIt<shapes.size(); ++shapeIt)
    {
        indexOffset = 0;
        for (int faceIt=0; faceIt<shapes[shapeIt].mesh.num_face_vertices.size(); ++faceIt)
        {
            tempTriangle.clear();
            v1 =  vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index] - vertices[shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index];
            v2 = vertices[shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index] - vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index];
            faceNormal = v1.cross(v2);
            faceNormal.normalize();
            tempTriangle.push_back(shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index);
            tempTriangle.push_back(shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index);
            tempTriangle.push_back(shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index);
            centroid = (vertices[shapes[shapeIt].mesh.indices[indexOffset].vertex_index] + vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index] + vertices[shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index]) / 3.0;
            maxX = 0;
            maxY = 0;
            maxZ = 0;
            for (int ii=0; ii < 3; ++ii)
            {
                v1 = vertices[tempTriangle[ii]] - centroid;
                if (abs(v1[0]) > maxX)
                {
                    maxX = abs(v1[0]);
                }
                if (abs(v1[1]) > maxY)
                {
                    maxY = abs(v1[1]);
                }
                if (abs(v1[2]) > maxZ)
                {
                    maxZ = abs(v1[2]);
                }
            }
            v2 <<maxX, maxY, maxZ;
            allCentroids.push_back(centroid);
            allBoundingBoxes.push_back(v2);
            allFaces.push_back(tempTriangle);
            allNormals.push_back(faceNormal);
            allConnections.push_back(dummyVec);
            unconnectedFaces.push_back(faceCounter);
            ungroupedFaces.push_back(faceCounter);
            faceMaxDist.push_back(std::max({vertices[shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index].norm(),
                vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index].norm(),
                vertices[shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index].norm()}));
            faceCounter++;
            
            indexOffset = indexOffset + shapes[shapeIt].mesh.num_face_vertices[faceIt];
        }
        
    }
    
    for ( int ii=0; ii<allConnections.size(); ++ii)
    {
        if (std::find(allConnections[ii].begin(), allConnections[ii].end(), -1) != allConnections[ii].end())
        {
            for ( int jj=1; jj<unconnectedFaces.size(); ++jj)
            {
                for ( int kk=0; kk<3; ++kk)
                {
                    for ( int gg=0; gg<3; ++gg)
                    {
                        if (allFaces[ii][kk] == allFaces[unconnectedFaces[jj]][gg])
                        {
                            *std::find(allConnections[ii].begin(), allConnections[ii].end(), -1) = unconnectedFaces[jj];
                            *std::find(allConnections[unconnectedFaces[jj]].begin(), allConnections[unconnectedFaces[jj]].end(), -1) = ii;
                            goto endloop;
                        }
                    }
                }
                endloop:
                if (std::find(allConnections[ii].begin(), allConnections[ii].end(), -1) != allConnections[ii].end())
                {
                    break;
                }
            }
            
            for ( int jj=0; jj<3; ++jj)
            {
                if (std::find(allConnections[allConnections[ii][jj]].begin(), allConnections[allConnections[ii][jj]].end(), -1) == allConnections[allConnections[ii][jj]].end())
                {
                    unconnectedFaces.erase(std::find(unconnectedFaces.begin(), unconnectedFaces.end(),allConnections[ii][jj]));
                }
            }
            unconnectedFaces.erase(std::find(unconnectedFaces.begin(), unconnectedFaces.end(), ii));
        }
    }
    
    std::sort(ungroupedFaces.begin(), ungroupedFaces.end(),
              [faceMaxDist] (int const& f1, int const& f2) -> bool
              {
                return faceMaxDist[f1] > faceMaxDist[f2];
              });
    
    while (!ungroupedFaces.empty())
    {
        boundingGroup.faceTriangles.clear();
        boundingGroup.faceNormals.clear();
        facesInGroup.clear();
        verticesInGroup.clear();
        
        boundingGroup.faceTriangles.push_back(allFaces[ungroupedFaces[0]]);
        boundingGroup.faceNormals.push_back(allNormals[ungroupedFaces[0]]);
        facesInGroup.push_back(ungroupedFaces[0]);
        for (int ii=0; ii < 3; ++ii)
        {
            verticesInGroup.push_back(vertices[allFaces[ungroupedFaces[0]][ii]]);
        }
        ungroupedFaces.erase(ungroupedFaces.begin());
        
        searchingForCloseFace = true;
        while (searchingForCloseFace)
        {
            adjacentFacesToGroup.clear();
            adjacentDistsToGroup.clear();
            for (int ii=0; ii < facesInGroup.size(); ++ii)
            {
                for (int jj=0; jj < 3; ++jj)
                {
                    if ((std::find(facesInGroup.begin(), facesInGroup.end(), allConnections[facesInGroup[ii]][jj]) == facesInGroup.end()) &&
                        (std::find(adjacentFacesToGroup.begin(), adjacentFacesToGroup.end(), allConnections[facesInGroup[ii]][jj]) == adjacentFacesToGroup.end()))
                    {
                        adjacentFacesToGroup.push_back(allConnections[facesInGroup[ii]][jj]);
                    }
                }
            }
            
            adjacentDistsToGroup.resize(adjacentFacesToGroup.size());
            for (int ii=0; ii < adjacentFacesToGroup.size(); ++ii)
            {
                adjacentDistsToGroup[ii] = 0;
                for (int jj=0; jj < 3; ++jj)
                {
                    indivMaxDists.clear();
                    for (int kk=0; kk < verticesInGroup.size(); ++kk)
                    {
                        indivMaxDists.push_back((verticesInGroup[kk] - vertices[allFaces[adjacentFacesToGroup[ii]][jj]]).norm());
                    }
                    if (*std::max_element(indivMaxDists.begin(), indivMaxDists.end()) > adjacentDistsToGroup[ii])
                    {
                        adjacentDistsToGroup[ii] = *std::max_element(indivMaxDists.begin(), indivMaxDists.end());
                    }
                }
            }
            
            faceCounter = std::distance(adjacentDistsToGroup.begin(), std::min_element(adjacentDistsToGroup.begin(), adjacentDistsToGroup.end()));
            boundingGroup.faceTriangles.push_back(allFaces[adjacentFacesToGroup[faceCounter]]);
            boundingGroup.faceNormals.push_back(allNormals[adjacentFacesToGroup[faceCounter]]);
            boundingGroup.faceCentroids.push_back(allCentroids[adjacentFacesToGroup[faceCounter]]);
            boundingGroup.faceBoundingBoxes.push_back(allBoundingBoxes[adjacentFacesToGroup[faceCounter]]);
            facesInGroup.push_back(adjacentFacesToGroup[faceCounter]);
            for (int ii=0; ii < 3; ++ii)
            {
                verticesInGroup.push_back(vertices[allFaces[adjacentFacesToGroup[faceCounter]][ii]]);
            }
            ungroupedFaces.erase(std::find(ungroupedFaces.begin(), ungroupedFaces.end(), adjacentFacesToGroup[faceCounter]));
            
            if (ungroupedFaces.empty())
            {
                searchingForCloseFace = false;
            }
            indivMaxDists.clear();
            for (int ii=0; ii < verticesInGroup.size()-1; ++ii)
            {
                for (int jj=ii+1; jj < verticesInGroup.size(); ++jj)
                {
                    indivMaxDists.push_back((verticesInGroup[ii] - verticesInGroup[jj]).norm());
                }
            }
            if (*std::max_element(indivMaxDists.begin(), indivMaxDists.end()) >= this->maxBoundingBoxDim)
            {
                searchingForCloseFace = false;
            }
        }
        
        polyhedron.push_back(boundingGroup);
    }
    
    for (int shapeIt=0; shapeIt<polyhedron.size(); ++shapeIt)
    {
        facesInGroup.clear();
        verticesInGroup.clear();
        for (int ii=0; ii < polyhedron[shapeIt].faceTriangles.size(); ++ii)
        {
            for (int jj=0; jj < 3; ++jj)
            {
                if (std::find(facesInGroup.begin(), facesInGroup.end(), polyhedron[shapeIt].faceTriangles[ii][jj]) == facesInGroup.end())
                {
                    facesInGroup.push_back(polyhedron[shapeIt].faceTriangles[ii][jj]);
                    verticesInGroup.push_back(vertices[polyhedron[shapeIt].faceTriangles[ii][jj]]);
                }
            }
        }
        
        convHullPoints = findConvexHull(verticesInGroup);
        centroid << 0.0, 0.0, 0.0;
        
        for (int ii=0; ii < convHullPoints.size(); ++ii)
        {
            centroid += convHullPoints[ii];
        }
        centroid = centroid / convHullPoints.size();
        
        maxX = 0;
        maxY = 0;
        maxZ = 0;
        for (int ii=0; ii < convHullPoints.size(); ++ii)
        {
            v1 = convHullPoints[ii] - centroid;
            if (abs(v1[0]) > maxX)
            {
                maxX = abs(v1[0]);
            }
            if (abs(v1[1]) > maxY)
            {
                maxY = abs(v1[1]);
            }
            if (abs(v1[2]) > maxZ)
            {
                maxZ = abs(v1[2]);
            }
        }
        
        polyhedron[shapeIt].centroid = centroid;
        polyhedron[shapeIt].boundingBox << maxX, maxY, maxZ;
        
        
        for (int faceIt=0; faceIt<polyhedron[shapeIt].faceTriangles.size(); ++faceIt)
        {
            
            for ( int inx=0; inx<2; ++inx)
            {
                std::vector<int> edgeGroup = {polyhedron[shapeIt].faceTriangles[faceIt][inx], polyhedron[shapeIt].faceTriangles[faceIt][inx+1]};
                polyhedron[shapeIt].edgeIndices.push_back(edgeGroup);
                polyhedron[shapeIt].faceIndices.push_back(faceIt);
            }
            std::vector<int> edgeGroup = { polyhedron[shapeIt].faceTriangles[faceIt][2], polyhedron[shapeIt].faceTriangles[faceIt][0]};
            polyhedron[shapeIt].edgeIndices.push_back(edgeGroup);
            polyhedron[shapeIt].faceIndices.push_back(faceIt);
        }
        
        for ( int edgeIt=0; edgeIt<polyhedron[shapeIt].edgeIndices.size(); ++edgeIt)
        {
            polyhedron[shapeIt].uniqueVertIndices.push_back(polyhedron[shapeIt].edgeIndices[edgeIt][0]);
            searchEdge[0] = polyhedron[shapeIt].edgeIndices[edgeIt][1];
            searchEdge[1] = polyhedron[shapeIt].edgeIndices[edgeIt][0];
            for ( int searchIt=edgeIt+1; searchIt<polyhedron[shapeIt].edgeIndices.size(); ++searchIt)
            {
                if (polyhedron[shapeIt].edgeIndices[searchIt]==searchEdge)
                {
                    std::vector<std::vector<int>>::iterator itEdge1 = polyhedron[shapeIt].edgeIndices.begin() + edgeIt+1;
                    std::vector<std::vector<int>>::iterator itSearch1 = polyhedron[shapeIt].edgeIndices.begin() + searchIt+1;
                    std::vector<int>::iterator itEdge2 = polyhedron[shapeIt].faceIndices.begin() + edgeIt+1;
                    std::vector<int>::iterator itSearch2 = polyhedron[shapeIt].faceIndices.begin() + searchIt+1;
                    polyhedron[shapeIt].edgeIndices.emplace(itEdge1, searchEdge);
                    polyhedron[shapeIt].edgeIndices.erase(itSearch1);
                    polyhedron[shapeIt].faceIndices.emplace(itEdge2, polyhedron[shapeIt].faceIndices[searchIt]);
                    polyhedron[shapeIt].faceIndices.erase(itSearch2);
                    edgeIt++;
                    continue;
                }
            }
        }
        
        std::set<int> uniqueSet(polyhedron[shapeIt].uniqueVertIndices.begin(), polyhedron[shapeIt].uniqueVertIndices.end());
        polyhedron[shapeIt].uniqueVertIndices.assign(uniqueSet.begin(), uniqueSet.end());
        
    }
    
    
    
    return polyhedron;
}

/*! This method allows the RB Contact state effector to have access to the hub states and gravity*/
void RigidBodyContactEffector::linkInStates(DynParamManager& statesIn)
{
    this->mainBody.hubState.hubPosition = statesIn.getStateObject("hubPosition");
    this->mainBody.hubState.hubSigma = statesIn.getStateObject("hubSigma");
    this->mainBody.hubState.hubOmega_BN_N = statesIn.getStateObject("hubOmega");
    this->mainBody.hubState.hubVelocity = statesIn.getStateObject("hubVelocity");
    this->mainBody.hubState.r_BN_N = statesIn.getPropertyReference( "r_BN_N");
    this->mainBody.hubState.v_BN_N = statesIn.getPropertyReference( "v_BN_N");
    this->mainBody.hubState.m_SC = statesIn.getPropertyReference( "m_SC");
    this->mainBody.hubState.ISCPntB_B = statesIn.getPropertyReference( "inertiaSC");
    this->mainBody.hubState.c_B = statesIn.getPropertyReference( "centerOfMassSC");
}

/*! This method computes the Forces on Torque on the Spacecraft Body.
@return void
@param integTime Integration time
@ToDo Distribute the mass at each contact point
*/
void RigidBodyContactEffector::computeForceTorque(double currentTime, double timeStep, std::string modelTag)
{
    this->forceExternal_N.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    
    
    dynamicData body1Current;
    dynamicData body2Current;
    dynamicData body1Future;
    dynamicData body2Future;
    
    vectorInterval tempVecInter;
    
    std::vector<vectorInterval> body1VertInter;
    std::vector<vectorInterval> body2VertInter;
    
    vectorInterval faceLegInterval1;
    vectorInterval faceLegInterval2;
    vectorInterval supportInterval;
    int tempNextVert1;
    int tempNextVert2;
    
    std::vector<double> elemTest;
    int intersectFlag;
    Eigen::Vector3d contactPoint;
    Eigen::Vector3d contactPoint2;
    int numImpacts;
    
    Eigen::Vector3d cHat_1;
    Eigen::Vector3d cHat_2;
    Eigen::Vector3d cHat_3;
    Eigen::Vector3d zDirection;
    Eigen::Vector3d xDirection;
    zDirection << 0, 0, 1;
    xDirection << 1, 0, 0;
    std::vector<Eigen::Matrix3d> dcm_CN;
    std::vector<Eigen::Matrix3d> dcm_CB1;
    std::vector<Eigen::Matrix3d> dcm_CB2;
    Eigen::Matrix3d tempDCM;
    Eigen::Matrix3d M_C;
    Eigen::MatrixXd M_tot;
    Eigen::MatrixXd M_inv;
    Eigen::MatrixXd M_inv_red1;
    Eigen::MatrixXd M_inv_red2;
    std::vector<double> f_vals;
    std::vector<double> phi_vals;
    int tempSel;
    Eigen::VectorXd dv3dj;
    Eigen::VectorXd tempVec;
    
    Eigen::VectorXd X_c;
    bool energyMet;
    int currLoop;
    Eigen::VectorXd k1;
    Eigen::VectorXd k2;
    Eigen::VectorXd k3;
    Eigen::VectorXd k4;
    Eigen::Vector3d impulse_Body1_N;
    
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>> impacts;
    
    for (int groupIt1=0; groupIt1 < this->closeBodies.size(); ++groupIt1)
    {
        // Fine collision detection begin
        if (this->Bodies[this->closeBodies[groupIt1][0]].isSpice == true)
        {
            body1Current.r_BN_N = this->Bodies[this->closeBodies[groupIt1][0]].states.r_BN_N + this->Bodies[this->closeBodies[groupIt1][0]].states.v_BN_N * (currentTime*NANO2SEC - this->currentSimSeconds);
            body1Current.dcm_BN = ((-this->Bodies[this->closeBodies[groupIt1][0]].states.omegaTilde_BN_B * this->Bodies[this->closeBodies[groupIt1][0]].states.dcm_BN) * (currentTime*NANO2SEC - this->currentSimSeconds)) + this->Bodies[this->closeBodies[groupIt1][0]].states.dcm_BN;
            body1Current.dcm_NB = body1Current.dcm_BN.transpose();
            body1Current.omegaTilde_BN_B = this->Bodies[this->closeBodies[groupIt1][0]].states.omegaTilde_BN_B;
            
            body1Future.r_BN_N = body1Current.r_BN_N + this->Bodies[this->closeBodies[groupIt1][0]].states.v_BN_N * timeStep*NANO2SEC ;
            body1Future.dcm_BN = ((-this->Bodies[this->closeBodies[groupIt1][0]].states.omegaTilde_BN_B * body1Current.dcm_BN) * timeStep*NANO2SEC) + body1Current.dcm_BN;
            body1Future.dcm_NB = body1Future.dcm_BN.transpose();
        }else
        {
            body1Current.r_BN_N = this->Bodies[this->closeBodies[groupIt1][0]].states.r_BN_N + this->Bodies[this->closeBodies[groupIt1][0]].states.v_BN_N * (currentTime*NANO2SEC - this->currentSimSeconds) + this->Bodies[this->closeBodies[groupIt1][0]].states.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][0]].states.nonConservativeAccelpntB_B * (currentTime*NANO2SEC - this->currentSimSeconds) * (currentTime*NANO2SEC - this->currentSimSeconds));
            body1Current.v_BN_N = this->Bodies[this->closeBodies[groupIt1][0]].states.v_BN_N + this->Bodies[this->closeBodies[groupIt1][0]].states.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][0]].states.nonConservativeAccelpntB_B * (currentTime*NANO2SEC - this->currentSimSeconds));
            body1Current.omega_BN_B = this->Bodies[this->closeBodies[groupIt1][0]].states.omega_BN_B + this->Bodies[this->closeBodies[groupIt1][0]].states.omegaDot_BN_B * (currentTime*NANO2SEC - this->currentSimSeconds);
            body1Current.sigma_BN = (0.25 * this->Bodies[this->closeBodies[groupIt1][0]].states.sigma_BN.Bmat() * body1Current.omega_BN_B * (currentTime*NANO2SEC - this->currentSimSeconds)) + ((Eigen::Vector3d) this->Bodies[this->closeBodies[groupIt1][0]].states.sigma_BN.coeffs());
            body1Current.dcm_NB = body1Current.sigma_BN.toRotationMatrix();
            body1Current.dcm_BN = body1Current.dcm_NB.transpose();
            body1Current.omegaTilde_BN_B = eigenTilde(body1Current.omega_BN_B);
            
            body1Future.r_BN_N = body1Current.r_BN_N + body1Current.v_BN_N * timeStep*NANO2SEC + body1Current.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][0]].states.nonConservativeAccelpntB_B * timeStep*NANO2SEC * timeStep*NANO2SEC);
            body1Future.omega_BN_B = body1Current.omega_BN_B + this->Bodies[this->closeBodies[groupIt1][0]].states.omegaDot_BN_B * timeStep*NANO2SEC;
            body1Future.sigma_BN = (0.25 * body1Current.sigma_BN.Bmat() * body1Future.omega_BN_B * timeStep*NANO2SEC) + ((Eigen::Vector3d) body1Current.sigma_BN.coeffs());
            body1Future.dcm_NB = body1Future.sigma_BN.toRotationMatrix();
            body1Future.dcm_BN = body1Future.dcm_NB.transpose();
        }
        
        if (this->Bodies[this->closeBodies[groupIt1][1]].isSpice == true)
        {
            body2Current.r_BN_N = this->Bodies[this->closeBodies[groupIt1][1]].states.r_BN_N + this->Bodies[this->closeBodies[groupIt1][1]].states.v_BN_N * (currentTime*NANO2SEC - this->currentSimSeconds);
            body2Current.dcm_BN = ((-this->Bodies[this->closeBodies[groupIt1][1]].states.omegaTilde_BN_B * this->Bodies[this->closeBodies[groupIt1][1]].states.dcm_BN) * (currentTime*NANO2SEC - this->currentSimSeconds)) + this->Bodies[this->closeBodies[groupIt1][1]].states.dcm_BN;
            body2Current.dcm_NB = body2Current.dcm_BN.transpose();
            body2Current.omegaTilde_BN_B = this->Bodies[this->closeBodies[groupIt1][1]].states.omegaTilde_BN_B;
            
            body2Future.r_BN_N = body2Current.r_BN_N + this->Bodies[this->closeBodies[groupIt1][1]].states.v_BN_N * timeStep*NANO2SEC ;
            body2Future.dcm_BN = ((-this->Bodies[this->closeBodies[groupIt1][1]].states.omegaTilde_BN_B * body2Current.dcm_BN) * timeStep*NANO2SEC) + body2Current.dcm_BN;
            body2Future.dcm_NB = body2Future.dcm_BN.transpose();
        }else
        {
            body2Current.r_BN_N = this->Bodies[this->closeBodies[groupIt1][1]].states.r_BN_N + this->Bodies[this->closeBodies[groupIt1][1]].states.v_BN_N * (currentTime*NANO2SEC - this->currentSimSeconds) + this->Bodies[this->closeBodies[groupIt1][1]].states.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][1]].states.nonConservativeAccelpntB_B * (currentTime*NANO2SEC - this->currentSimSeconds) * (currentTime*NANO2SEC - this->currentSimSeconds));
            body2Current.v_BN_N = this->Bodies[this->closeBodies[groupIt1][1]].states.v_BN_N + this->Bodies[this->closeBodies[groupIt1][1]].states.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][1]].states.nonConservativeAccelpntB_B * (currentTime*NANO2SEC - this->currentSimSeconds));
            body2Current.omega_BN_B = this->Bodies[this->closeBodies[groupIt1][1]].states.omega_BN_B + this->Bodies[this->closeBodies[groupIt1][1]].states.omegaDot_BN_B * (currentTime*NANO2SEC - this->currentSimSeconds);
            body2Current.sigma_BN = (0.25 * this->Bodies[this->closeBodies[groupIt1][1]].states.sigma_BN.Bmat() * body2Current.omega_BN_B * (currentTime*NANO2SEC - this->currentSimSeconds)) + ((Eigen::Vector3d) this->Bodies[this->closeBodies[groupIt1][1]].states.sigma_BN.coeffs());
            body2Current.dcm_NB = body2Current.sigma_BN.toRotationMatrix();
            body2Current.dcm_BN = body2Current.dcm_NB.transpose();
            body2Current.omegaTilde_BN_B = eigenTilde(body2Current.omega_BN_B);
            
            body2Future.r_BN_N = body2Current.r_BN_N + body2Current.v_BN_N * timeStep*NANO2SEC + body2Current.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][1]].states.nonConservativeAccelpntB_B * timeStep*NANO2SEC * timeStep*NANO2SEC);
            body2Future.omega_BN_B = body2Current.omega_BN_B + this->Bodies[this->closeBodies[groupIt1][0]].states.omegaDot_BN_B * timeStep*NANO2SEC;
            body2Future.sigma_BN = (0.25 * body2Current.sigma_BN.Bmat() * body2Future.omega_BN_B * timeStep*NANO2SEC) + ((Eigen::Vector3d) body2Current.sigma_BN.coeffs());
            body2Future.dcm_NB = body2Future.sigma_BN.toRotationMatrix();
            body2Future.dcm_BN = body2Future.dcm_NB.transpose();
        }
        
        // Begin looping through contactable triangles
        
        for (int triPairInd=0; triPairInd < this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps.size(); triPairInd++)
        {
            
            tempVecInter.lower = body1Current.r_BN_N + body1Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][0]];
            tempVecInter.upper = body1Future.r_BN_N + body1Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][0]];
            
            body1VertInter.push_back(tempVecInter);
            
            tempVecInter.lower = body1Current.r_BN_N + body1Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][1]];
            tempVecInter.upper = body1Future.r_BN_N + body1Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][1]];
            
            body1VertInter.push_back(tempVecInter);
            
            tempVecInter.lower = body1Current.r_BN_N + body1Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][2]];
            tempVecInter.upper = body1Future.r_BN_N + body1Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][2]];
            
            body1VertInter.push_back(tempVecInter);
            
            tempVecInter.lower = body2Current.r_BN_N + body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<2>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<3>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][0]];
            tempVecInter.upper = body2Future.r_BN_N + body2Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<2>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<3>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][0]];
            
            body2VertInter.push_back(tempVecInter);
            
            tempVecInter.lower = body2Current.r_BN_N + body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<2>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<3>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][1]];
            tempVecInter.upper = body2Future.r_BN_N + body2Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<2>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<3>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][1]];
            
            body2VertInter.push_back(tempVecInter);
            
            tempVecInter.lower = body2Current.r_BN_N + body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<2>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<3>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][2]];
            tempVecInter.upper = body2Future.r_BN_N + body2Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<2>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceTriangles[std::get<3>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])][2]];
            
            body2VertInter.push_back(tempVecInter);
            
            // Face of triange 1 with each vertex of triangle 2
            faceLegInterval1.lower = body1VertInter[0].lower - body1VertInter[1].lower;
            faceLegInterval1.upper = body1VertInter[0].upper - body1VertInter[1].upper;
            faceLegInterval2.lower = body1VertInter[0].lower - body1VertInter[2].lower;
            faceLegInterval2.upper = body1VertInter[0].upper - body1VertInter[2].upper;
            
            for (int vertInd=0; vertInd < 3; vertInd++)
            {
                supportInterval.lower = body2VertInter[vertInd].lower - body1VertInter[0].lower;
                supportInterval.upper = body2VertInter[vertInd].upper - body1VertInter[0].upper;
                
                elemTest = this->IntervalDotProduct(supportInterval, this->IntervalCrossProduct(faceLegInterval1, faceLegInterval2));
                
                if (((elemTest[0] <= -1e-12) && (elemTest[1] >= 1e-12)) || ((elemTest[0] >= 1e-12) && (elemTest[1] <= -1e-12)))
                {
                    intersectFlag = this->PointInTriangle(body2VertInter[vertInd].lower, body1VertInter[0].lower, body1VertInter[1].lower, body1VertInter[2].lower, &contactPoint);
                    
                    if (intersectFlag == 1)
                    {
                        impacts.push_back(std::make_tuple(contactPoint, body2VertInter[vertInd].lower, body1Current.dcm_NB * -this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceNormals[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])]));
                    }
                }
            }
            
            // Face of triange 2 with each vertex of triangle 1
            faceLegInterval1.lower = body2VertInter[0].lower - body2VertInter[1].lower;
            faceLegInterval1.upper = body2VertInter[0].upper - body2VertInter[1].upper;
            faceLegInterval2.lower = body2VertInter[0].lower - body2VertInter[2].lower;
            faceLegInterval2.upper = body2VertInter[0].upper - body2VertInter[2].upper;
            
            for (int vertInd=0; vertInd < 3; vertInd++)
            {
                supportInterval.lower = body1VertInter[vertInd].lower - body2VertInter[0].lower;
                supportInterval.upper = body1VertInter[vertInd].upper - body2VertInter[0].upper;
                
                elemTest = this->IntervalDotProduct(supportInterval, this->IntervalCrossProduct(faceLegInterval1, faceLegInterval2));
                
                if (((elemTest[0] <= -1e-12) && (elemTest[1] >= 1e-12)) || ((elemTest[0] >= 1e-12) && (elemTest[1] <= -1e-12)))
                {
                    intersectFlag = this->PointInTriangle(body1VertInter[vertInd].lower, body2VertInter[0].lower, body2VertInter[1].lower, body2VertInter[2].lower, &contactPoint);
                    
                    if (intersectFlag == 1)
                    {
                        impacts.push_back(std::make_tuple(body1VertInter[vertInd].lower, contactPoint, body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<2>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])].faceNormals[std::get<3>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[triPairInd])]));
                    }
                }
            }
            
            // Each edge of triangle 1 with each edge of triangle 2 (not avoiding duplicate edges)
            for (int vertInd1=0; vertInd1 < 3; vertInd1++)
            {
                if (vertInd1 == 2)
                {
                    tempNextVert1 = 0;
                } else
                {
                    tempNextVert1 = vertInd1 + 1;
                }
                // Reuse the faceLegInterval variables, but these should be called edgeInterval
                faceLegInterval1.lower = body1VertInter[tempNextVert1].lower - body1VertInter[vertInd1].lower;
                faceLegInterval1.upper = body1VertInter[tempNextVert1].upper - body1VertInter[vertInd1].upper;
                
                for (int vertInd2=0; vertInd2 < 3; vertInd2++)
                {
                    if (vertInd2 == 2)
                    {
                        tempNextVert2 = 0;
                    } else
                    {
                        tempNextVert2 = vertInd2 + 1;
                    }
                    faceLegInterval2.lower = body2VertInter[tempNextVert2].lower - body2VertInter[vertInd2].lower;
                    faceLegInterval2.upper = body2VertInter[tempNextVert2].upper - body2VertInter[vertInd2].upper;
                    // Reuse supportInterval, but it should be called edgeIntervalMixed
                    supportInterval.lower = body2VertInter[vertInd2].lower - body1VertInter[vertInd1].lower;
                    supportInterval.upper = body2VertInter[vertInd2].upper - body1VertInter[vertInd1].upper;
                    
                    elemTest = this->IntervalDotProduct(supportInterval, this->IntervalCrossProduct(faceLegInterval1, faceLegInterval2));
                    
                    if (((elemTest[0] <= -1e-12) && (elemTest[1] >= 1e-12)) || ((elemTest[0] >= 1e-12) && (elemTest[1] <= -1e-12)))
                    {
                        intersectFlag = this->LineLineDistance(body1VertInter[vertInd1].lower, body1VertInter[tempNextVert1].lower, body2VertInter[vertInd2].lower, body2VertInter[tempNextVert2].lower, &contactPoint, &contactPoint2);
                        
                        if (intersectFlag == 0)
                        {
                            impacts.push_back(std::make_tuple(contactPoint, contactPoint2, ((contactPoint - contactPoint2) * 1e6).normalized));
                        }else if (intersectFlag == 1)
                        {
                            impacts.push_back(std::make_tuple(contactPoint, contactPoint2, (eigenTilde(faceLegInterval1.lower) * faceLegInterval2.lower).normalized));
                        }
                    }
                }
            }
        } // Fine collision detection end
        
        // Calculate total impact begin
        numImpacts = impacts.size();
        for (int impNum=0; impNum < numImpacts; impNum++)
        {
            // Create local contact frame
            cHat_3 = (std::get<2>(impacts[impNum])).normalized();
            cHat_1 = cHat_3.cross(body2Current.dcm_NB * zDirection);
            if (cHat_1.norm() < 1e-9)
            {
                cHat_1 = cHat_3.cross(body2Current.dcm_NB * xDirection);
            }
            cHat_1 = cHat_1.normalized();
            cHat_2 = (cHat_3.cross(cHat_1)).normalized();
            
            // Create DCMs to rotate between inertial, contact, and the body frames
            tempDCM <<  cHat_1[0], cHat_1[1], cHat_1[2],
                        cHat_2[0], cHat_2[1], cHat_2[2],
                        cHat_3[0], cHat_3[1], cHat_3[2];
            dcm_CN.push_back(tempDCM);
            dcm_CB1.push_back(dcm_CN[impNum] * body1Current.dcm_NB);
            dcm_CB2.push_back(dcm_CN[impNum] * body2Current.dcm_NB);
        }
        
        // Create the "inverse inertia matrix"
        M_tot = Eigen::MatrixXd::Zero(3*numImpacts, 3*numImpacts);
        for (int ii=0; ii < numImpacts; ii++)
        {
            for (int jj=0; jj < numImpacts; jj++)
            {
                M_C = ((1.0 / this->Bodies[this->closeBodies[groupIt1][0]].states.m_SC) * Eigen::MatrixXd::Identity(3, 3) - eigenTilde(dcm_CN[ii] * std::get<0>(impacts[ii])) * (dcm_CB1[ii] * this->Bodies[this->closeBodies[groupIt1][0]].states.ISCPntB_B_inv * dcm_CB1[ii].transpose()) * eigenTilde(dcm_CN[ii] * std::get<0>(impacts[jj]))) + ((1.0 / this->Bodies[this->closeBodies[groupIt1][1]].states.m_SC) * Eigen::MatrixXd::Identity(3, 3) - eigenTilde(dcm_CN[ii] * std::get<1>(impacts[ii])) * (dcm_CB2[ii] * this->Bodies[this->closeBodies[groupIt1][1]].states.ISCPntB_B_inv * dcm_CB2[ii].transpose()) * eigenTilde(dcm_CN[ii] * std::get<1>(impacts[jj])));
                
                if (ii == jj)
                {
                    M_tot.block(ii*3, jj*3, 3, 3) = M_C;
                }else
                {
                    M_tot.block(ii*3, jj*3, 3, 3) = M_C * (dcm_CB1[ii] * dcm_CB1[jj].transpose());
                }
            }
        }
        
        // Solve for the critical values of friction and sliding direction
        if (impacts.size() == 1)
        {
            f_vals.push_back(sqrt((pow(M_tot(0,1) * M_tot(1,2) - M_tot(1,1) * M_tot(0,2), 2) + pow(M_tot(1,0) * M_tot(0,2) - M_tot(0,0) * M_tot(1,2), 2)) / pow(M_tot(0,0) * M_tot(1,1) - M_tot(0,1) * M_tot(1,0), 2)));
            
            phi_vals.push_back(atan2(M_tot(0,0) * M_tot(1,2) - M_tot(1,0) * M_tot(0,2), M_tot(1,1) * M_tot(0,2) - M_tot(0,1) * M_tot(1,3)));
        }else
        {
            M_inv = M_tot.completeOrthogonalDecomposition().pseudoInverse();
            M_inv_red1 = Eigen::MatrixXd::Zero(3*numImpacts, numImpacts);
            tempSel = 2;
            for (int ii=0; ii < numImpacts; ii++)
            {
                M_inv_red1.block(0, ii, 3*numImpacts, 1) = M_inv.block(0, tempSel, 3*numImpacts, 1);
                tempSel += 3;
            }
            M_inv_red2 = Eigen::MatrixXd::Zero(numImpacts, numImpacts);
            tempSel = 2;
            for (int ii=0; ii < numImpacts; ii++)
            {
                M_inv_red2.block(ii, 0, 1, numImpacts) = M_inv_red1.block(tempSel, 0, 1, numImpacts);
                tempSel += 3;
            }
            
            dv3dj = M_inv_red2.completeOrthogonalDecomposition().pseudoInverse() * Eigen::VectorXd::Ones(numImpacts);
            tempVec = Eigen::VectorXd::Zero(3*numImpacts);
            tempSel = 2;
            for (int ii=0; ii < numImpacts; ii++)
            {
                tempVec(tempSel) = dv3dj(ii);
                tempSel += 3;
            }
            
            tempVec = M_inv * tempVec;
            tempSel = 0;
            for (int ii=0; ii < numImpacts; ii++)
            {
                f_vals.push_back(sqrt(pow(tempVec(tempSel), 2) + pow(tempVec(tempSel+1), 2)));
                phi_vals.push_back(atan2(tempVec(tempSel+1), tempVec(tempSel)));
                tempSel += 3;
            }
        }
        
        // Create the initial collision state
        X_c = Eigen::VectorXd::Zero(numImpacts * 8);
        for (int impNum=0; impNum < numImpacts; impNum++)
        {
            // Velocity of the contact point on body 1 relative to the contact point on body 2, in the local contact frame
            X_c.segment(impNum * 3, 3) = dcm_CN[impNum] * ((body1Current.v_BN_N + body1Current.dcm_NB * (body1Current.omegaTilde_BN_B * (body1Current.dcm_BN * std::get<0>(impacts[impNum])))) - (body2Current.v_BN_N + body2Current.dcm_NB * (body2Current.omegaTile_BN_B * (body2Current.dcm_BN * std::get<1>(impacts[impNum])))));
            
            // Add initial perturbation to energy states for numerical robustness
            if (X_c(impNum * 3) < 0)
            {
                X_c(numImpacts * 6 + impNum * 2 + 1) = -1e-14;
            } else
            {
                X_c(numImpacts * 6 + impNum * 2 + 1) = 1e-14;
            }
        }
        
        // Integrate the collision state until the restitution energy conditions are met
        energyMet = false;
        currLoop = 0;
        while (!energyMet)
        {
            currLoop++;
            // Use RK4 to integrate the collision state
            k1 = this->CollisionStateDerivative(X_c, impacts, f_vals, phi_vals, M_tot, this->Bodies[this->closeBodies[groupIt1][0]].coefRestitution, this->Bodies[this->closeBodies[groupIt1][0]].coefFriction);
            k2 = this->CollisionStateDerivative(X_c + this->collisionIntegrationStep * (k1 / 2.0), impacts, f_vals, phi_vals, M_tot, this->Bodies[this->closeBodies[groupIt1][0]].coefRestitution, this->Bodies[this->closeBodies[groupIt1][0]].coefFriction);
            k3 = this->CollisionStateDerivative(X_c + this->collisionIntegrationStep * (k2 / 2.0), impacts, f_vals, phi_vals, M_tot, this->Bodies[this->closeBodies[groupIt1][0]].coefRestitution, this->Bodies[this->closeBodies[groupIt1][0]].coefFriction);
            k4 = this->CollisionStateDerivative(X_c + this->collisionIntegrationStep * k3, impacts, f_vals, phi_vals, M_tot, this->Bodies[this->closeBodies[groupIt1][0]].coefRestitution, this->Bodies[this->closeBodies[groupIt1][0]].coefFriction);
            X_c = X_c + (this->collisionIntegrationStep / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
            
            energyMet = true;
            // Check if there are any contact points that have not met the restitution energy requirements
            for (int impNum=0; impNum < numImpacts; impNum++)
            {
                if (X_c(numImpacts * 6 + impNum * 2 + 1) < (-1 * (pow(this->Bodies[this->closeBodies[groupIt1][0]].coefRestitution, 2) * X_c(numImpacts * 6 + impNum * 2))))
                {
                    energyMet = false;
                    break;
                }
            }
            
            // Hard cap on number of loops, which should never be reached.
            if (currLoop > 1e9)
            {
                energyMet = true;
            }
        }
        
        // Extract resulting force and torque
        for (int impNum=0; impNum < numImpacts; impNum++)
        {
            impulse_Body1_N = dcm_CN[impNum].transpose() * X_c.segment(numImpacts * 3 + impNum * 3, 3);
            this->Bodies[this->closeBodies[groupIt1][0]].forceExternal_N += impulse_Body1_N / timeStep;
            this->Bodies[this->closeBodies[groupIt1][1]].forceExternal_N -= impulse_Body1_N / timeStep;
            this->Bodies[this->closeBodies[groupIt1][0]].torqueExternalPntB_B += body1Current.dcm_BN * (std::get<0>(impacts[impNum]) - body1Current.r_BN_N).cross(impulse_Body1_N / timeStep);
            this->Bodies[this->closeBodies[groupIt1][1]].torqueExternalPntB_B -= body2Current.dcm_BN * (std::get<1>(impacts[impNum]) - body2Current.r_BN_N).cross(impulse_Body1_N / timeStep);
        }
        
    }
    

}

Eigen::VectorXd RigidBodyContactEffector::CollisionStateDerivative( Eigen::VectorXd X_c, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>> impacts, std::vector<double> f_vals, std::vector<double> phi_vals, Eigen::MatrixXd M_tot, double coefRes, double coefFric)
{
    Eigen::VectorXd Xdot_c = Eigen::VectorXd::Zero(X_c.size());
    double phi;
    int numImpacts = impacts.size();

    // - Loop through every collision point
    for (int impNum=0; impNum < numImpacts; impNum++)
    {
        if (X_c(numImpacts * 6 + impNum * 2 + 1) < (-1 * (pow(coefRes, 2) * X_c(numImpacts * 6 + impNum * 2))))
        {
            if (sqrt(pow(X_c(impNum * 3), 2) + pow(X_c(impNum * 3 + 1), 2)) < 1e-6)
            {
                Xdot_c(numImpacts * 3 + impNum * 3) = -f_vals[impNum] * cos(phi_vals[impNum]);
                Xdot_c(numImpacts * 3 + impNum * 3 + 1) = -f_vals[impNum] * sin(phi_vals[impNum]);
                Xdot_c(numImpacts * 3 + impNum * 3 + 2) = 1.0;
            }else
            {
                phi = atan2(X_c(impNum * 3 + 1), X_c(impNum * 3));
                Xdot_c(numImpacts * 3 + impNum * 3) = -coefFric * cos(phi);
                Xdot_c(numImpacts * 3 + impNum * 3 + 1) = -coefFric * sin(phi);
                Xdot_c(numImpacts * 3 + impNum * 3 + 2) = 1.0;
            }
        }
        
        if (X_c(impNum * 3 + 2) < 0)
        {
            Xdot_c(numImpacts * 6 + impNum * 2) = X_c(impNum * 3 + 2);
        }else if (X_c(numImpacts * 6 + impNum * 2 + 1) < (-1 * (pow(coefRes, 2) * X_c(numImpacts * 6 + impNum * 2))))
        {
            Xdot_c(numImpacts * 6 + impNum * 2 + 1) = X_c(impNum * 3 + 2);
        }
    }
    
    Xdot_c.head(numImpacts * 3) = M_tot * Xdot_c.segment(numImpacts * 3, numImpacts * 3);

    return Xdot_c;
}


//void RigidBodyContactEffector::CalcCollisionProps()
//{
//    Eigen::Vector3d cHat_1;
//    Eigen::Vector3d cHat_2;
//    Eigen::Vector3d cHat_3;
//    Eigen::Vector3d zDirection;
//    Eigen::Vector3d xDirection;
//    zDirection << 0, 0, 1;
//    xDirection << 1, 0, 0;
//    Eigen::Matrix3d MSCPntC_C;
//    Eigen::Matrix3d dcm_CN;
//    double slipReverseDirp1;
//    double h;
//    double hp;
//    double hpp;
//    double hppp;
//    // - Check if any new collisions will happen during this time step, and calculate the resulting forces and torques
//    if (this->mainBody.collisionPoints.empty() == false)
//    {
//        if (this->mainBody.collisionPoints[0].empty() == false)
//        {
//            // - Loop through every collision point
//            for ( int bodyIt = 0; bodyIt < this->mainBody.collisionPoints.size(); ++bodyIt)
//            {
//                this->mainBody.collisionPoints[bodyIt].back().slipHitZero = false;
//                cHat_3 = - this->mainBody.collisionPoints[bodyIt].back().contactNormal;
//                cHat_1 = cHat_3.cross( this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].states.dcm_NB * zDirection);
//                cHat_1.normalize();
//                if (int(ceil(cHat_1.norm() * pow(10.0, 9))) == 0 )
//                {
//                    cHat_1 = this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].states.dcm_NB * xDirection;
//                    cHat_1.normalize();
//                }
//                cHat_2 = cHat_3.cross(cHat_1);
//                cHat_2.normalize();
//                dcm_CN << cHat_1[0], cHat_1[1], cHat_1[2], cHat_2[0], cHat_2[1], cHat_2[2], cHat_3[0], cHat_3[1], cHat_3[2];
//                this->mainBody.collisionPoints[bodyIt].back().dcm_CB = dcm_CN * this->mainBody.states.dcm_NB;
//                for ( int bodyIt2 = 0; bodyIt2 < this->mainBody.collisionPoints.size(); ++bodyIt2)
//                {
//                    this->mainBody.collisionPoints[bodyIt].back().MSCPntC_B.push_back(((1.0 / this->mainBody.states.m_SC) * Eigen::Matrix3d::Identity()) - (eigenTilde(this->mainBody.collisionPoints[bodyIt].back().mainContactPoint) * (this->mainBody.states.ISCPntB_B_inv * eigenTilde(this->mainBody.collisionPoints[bodyIt2].back().mainContactPoint))));
//                }
//                MSCPntC_C = (this->mainBody.collisionPoints[bodyIt].back().dcm_CB) * this->mainBody.collisionPoints[bodyIt].back().MSCPntC_B[bodyIt] * (this->mainBody.collisionPoints[bodyIt].back().dcm_CB).transpose();
//                this->mainBody.collisionPoints[bodyIt].back().critSlideDir = atan2((MSCPntC_C(0,0) * MSCPntC_C(1,2) - MSCPntC_C(1,0) * MSCPntC_C(0,2)) , (MSCPntC_C(1,1) * MSCPntC_C(0,2) - MSCPntC_C(0,1) * MSCPntC_C(1,2)));
//                this->mainBody.collisionPoints[bodyIt].back().critCoeffFric = sqrt((pow(MSCPntC_C(0,1) * MSCPntC_C(1,2) - MSCPntC_C(1,1) * MSCPntC_C(0,2), 2.0) + pow(MSCPntC_C(1,0) * MSCPntC_C(0,2) - MSCPntC_C(0,0) * MSCPntC_C(1,2), 2.0)) / pow(MSCPntC_C(0,0) * MSCPntC_C(1,1) - MSCPntC_C(0,1) * MSCPntC_C(1,0), 2.0));
//                std::cout << MSCPntC_C(0,0) * MSCPntC_C(1,1) - MSCPntC_C(0,1) * MSCPntC_C(1,0) << '\n' << '\n';
//
//                if (this->mainBody.collisionPoints[bodyIt].back().critCoeffFric > this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction)
//                {
//                    slipReverseDirp1 = this->mainBody.collisionPoints[bodyIt].back().critSlideDir;
//                    do
//                    {
//                        this->mainBody.collisionPoints[bodyIt].back().slipReverseDir = slipReverseDirp1;
//                        h = -MSCPntC_C(0,2) * sin(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + MSCPntC_C(1,2) * cos(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + 0.5 *  this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * (MSCPntC_C(0,0) - MSCPntC_C(1,1)) * sin(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * cos(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir);
//                        hp = -MSCPntC_C(0,2) * cos(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - MSCPntC_C(1,2) * sin(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * (MSCPntC_C(0,0) - MSCPntC_C(1,1)) * cos(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + 2.0 * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * sin(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir);
//
//                        hpp = MSCPntC_C(0,2) * sin(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - MSCPntC_C(1,2) * cos(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - 2.0 * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * (MSCPntC_C(0,0) - MSCPntC_C(1,1)) * sin(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + 4.0 * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * cos(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir);
//
//                        hppp = MSCPntC_C(0,2) * cos(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + MSCPntC_C(1,2) * sin(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - 4.0 * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * (MSCPntC_C(0,0) - MSCPntC_C(1,1)) * cos(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - 8.0 * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * sin(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir);
//
//                        slipReverseDirp1 = this->mainBody.collisionPoints[bodyIt].back().slipReverseDir - (6.0 * h * pow(hpp, 2.0) -3.0 * pow(h, 2.0) * hpp) / (6.0 * pow(hp, 3.0) - 6.0 * h * hp * hpp + pow(h, 2.0) * hppp);
//                    }
//                    while (abs(slipReverseDirp1 - this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) > 1e-6);
//                    this->mainBody.collisionPoints[bodyIt].back().slipReverseDir = slipReverseDirp1;
//                }
//            }
//
//        }
//    }
//    return;
//}

void RigidBodyContactEffector::computeStateContribution(double integTime)
{
    
}

/*! This method asks for all current messages, and runs all methods related to collision detection.
@return void
@param CurrentSimNanos The current simulation time in nanoseconds
*/
void RigidBodyContactEffector::UpdateState(uint64_t CurrentSimNanos)
{
    this->currentSimSeconds = CurrentSimNanos*NANO2SEC;

    
    this->ReadInputs();
    this->ExtractFromBuffer();
    
    for (int bodyIt=0; bodyIt<this->numBodies; ++bodyIt)
    {
        
        if (this->Bodies[bodyIt].isSpice == true)
        {
            this->Bodies[bodyIt].futureStates.r_BN_N = this->Bodies[bodyIt].states.r_BN_N + this->Bodies[bodyIt].states.v_BN_N * this->simTimeStep;
            this->Bodies[bodyIt].futureStates.dcm_BN = ((-this->Bodies[bodyIt].states.omegaTilde_BN_B * this->Bodies[bodyIt].states.dcm_BN) * this->simTimeStep) + this->Bodies[bodyIt].states.dcm_BN;
            this->Bodies[bodyIt].futureStates.dcm_NB = this->Bodies[bodyIt].futureStates.dcm_BN.transpose();
        }else
        {
            this->Bodies[bodyIt].futureStates.r_BN_N = this->Bodies[bodyIt].states.r_BN_N + this->Bodies[bodyIt].states.v_BN_N * this->simTimeStep + this->Bodies[bodyIt].states.dcm_NB * (this->Bodies[bodyIt].states.nonConservativeAccelpntB_B * this->simTimeStep * this->simTimeStep);
            this->Bodies[bodyIt].futureStates.sigma_BN = (0.25 * this->Bodies[bodyIt].states.sigma_BN.Bmat() * (this->Bodies[bodyIt].states.omega_BN_B + this->Bodies[bodyIt].states.omegaDot_BN_B * this->simTimeStep) * this->simTimeStep) + ((Eigen::Vector3d) this->Bodies[bodyIt].states.sigma_BN.coeffs());
            this->Bodies[bodyIt].futureStates.dcm_NB = this->Bodies[bodyIt].futureStates.sigma_BN.toRotationMatrix();
            this->Bodies[bodyIt].futureStates.dcm_BN = this->Bodies[bodyIt].futureStates.dcm_NB.transpose();
        }
    }
    
    this->closeBodies.clear();
    this->CheckBoundingSphere();
    this->CheckBoundingBox();
    this->isOverlap = false;
    
//    for ( std::vector<int>::iterator bodyIt = this->closeBodies.begin(); bodyIt != this->closeBodies.end(); ++bodyIt){
//        this->isOverlap = this->Overlap(this->Bodies[*bodyIt], *bodyIt);
//    }
    this->CalcCollisionProps();
    
    return;
}

/*! This method is used to read the messages pertaining to all external bodies.
@return void
*/
void RigidBodyContactEffector::ReadInputs()
{

    for (int bodyIt=0; bodyIt<this->numBodies; ++bodyIt)
    {
        if (this->Bodies[bodyIt].isSpice == true)
        {
            this->Bodies[bodyIt].plMsg = this->Bodies[bodyIt].planetInMsg();
        }else
        {
            this->Bodies[bodyIt].stateInBuffer = this->Bodies[bodyIt].scStateInMsg();
            this->Bodies[bodyIt].massStateInBuffer = this->Bodies[bodyIt].scMassStateInMsg();
        }
        
    }

    return;
}

/*! This method extracts all important information for each external body.
@return void
*/
void RigidBodyContactEffector::ExtractFromBuffer()
{
    Eigen::Matrix3d dcm_BN_dot;
    for( int bodyIt=0; bodyIt<this->Bodies.size(); ++bodyIt)
    {
        if (this->Bodies[bodyIt].isSpice == true)
        {
            this->Bodies[bodyIt].states.r_BN_N = cArray2EigenVector3d(this->Bodies[bodyIt].plMsg.PositionVector);
            this->Bodies[bodyIt].states.v_BN_N = cArray2EigenVector3d(this->Bodies[bodyIt].plMsg.VelocityVector);
            this->Bodies[bodyIt].states.dcm_BN = cArray2EigenMatrix3d(*this->Bodies[bodyIt].plMsg.J20002Pfix);
            dcm_BN_dot = cArray2EigenMatrix3d(*this->Bodies[bodyIt].plMsg.J20002Pfix_dot);
            this->Bodies[bodyIt].states.omegaTilde_BN_B = - dcm_BN_dot * this->Bodies[bodyIt].states.dcm_BN.transpose();
            this->Bodies[bodyIt].states.sigma_BN = eigenC2MRP(this->Bodies[bodyIt].states.dcm_BN);
            this->Bodies[bodyIt].states.dcm_NB = this->Bodies[bodyIt].states.dcm_BN.transpose();
        }else
        {
            this->Bodies[bodyIt].states.r_BN_N = cArray2EigenVector3d(this->Bodies[bodyIt].stateInBuffer.r_BN_N);
            this->Bodies[bodyIt].states.v_BN_N = cArray2EigenVector3d(this->Bodies[bodyIt].stateInBuffer.v_BN_N);
            this->Bodies[bodyIt].states.nonConservativeAccelpntB_B = cArray2EigenVector3d(this->Bodies[bodyIt].stateInBuffer.nonConservativeAccelpntB_B);
            this->Bodies[bodyIt].states.m_SC = this->Bodies[bodyIt].massStateInBuffer.massSC;
            this->Bodies[bodyIt].states.c_B = cArray2EigenVector3d(this->Bodies[bodyIt].massStateInBuffer.c_B);
            this->Bodies[bodyIt].states.omega_BN_B = cArray2EigenVector3d(this->Bodies[bodyIt].stateInBuffer.omega_BN_B);
            this->Bodies[bodyIt].states.omegaDot_BN_B = cArray2EigenVector3d(this->Bodies[bodyIt].stateInBuffer.omegaDot_BN_B);
            this->Bodies[bodyIt].states.ISCPntB_B = cArray2EigenMatrix3d(this->Bodies[bodyIt].massStateInBuffer.ISC_PntB_B);
            this->Bodies[bodyIt].states.ISCPntB_B_inv = this->Bodies[bodyIt].states.ISCPntB_B.inverse();
            this->Bodies[bodyIt].states.sigma_BN = cArray2EigenVector3d(this->Bodies[bodyIt].stateInBuffer.sigma_BN);
            this->Bodies[bodyIt].states.dcm_NB = this->Bodies[bodyIt].states.sigma_BN.toRotationMatrix();
            this->Bodies[bodyIt].states.dcm_BN = this->Bodies[bodyIt].states.dcm_NB.transpose();
            this->Bodies[bodyIt].states.omegaTilde_BN_B = eigenTilde(this->Bodies[bodyIt].states.omega_BN_B);
        }
        
        
    }
    return;
}



/*! This method checks if the primary body is within the bounding sphere of any external bodies.
@return void
*/
void RigidBodyContactEffector::CheckBoundingSphere()
{
    this->closeBodies.clear();
    std::vector<int> bodyPair(2,0);
    vectorInterval bodyDifference;
    std::vector<double> bodyDistance;
    
    for(int bodyIt1=0; bodyIt1 < this->numBodies-1; ++bodyIt1)
    {
        bodyPair[0] = bodyIt1;
        for(int bodyIt2=bodyIt1+1; bodyIt2 < this->numBodies; ++bodyIt2)
        {
            bodyPair[1] = bodyIt2;
            bodyDifference.lower = this->Bodies[bodyIt1].states.r_BN_N - this->Bodies[bodyIt2].states.r_BN_N;
            bodyDifference.upper = this->Bodies[bodyIt1].futureStates.r_BN_N - this->Bodies[bodyIt2].futureStates.r_BN_N;
            bodyDistance = this->IntervalDotProduct(bodyDifference, bodyDifference);
            
            if ((sqrt(abs(bodyDistance[0])) < (this->Bodies[bodyIt1].boundingRadius + this->Bodies[bodyIt2].boundingRadius)) || (sqrt(abs(bodyDistance[1])) < (this->Bodies[bodyIt1].boundingRadius + this->Bodies[bodyIt2].boundingRadius)))
                this->closeBodies.push_back(bodyPair);
        }
        
    }
    return;
}


/*! This method checks if the primary body is within the bounding sphere of any external bodies.
@return void
*/
void RigidBodyContactEffector::CheckBoundingBox()
{
    boundingBoxDetail layer1Box;
    indivBoundingBox box1;
    indivBoundingBox box2;
    Eigen::Vector3d xAxis;
    Eigen::Vector3d yAxis;
    Eigen::Vector3d zAxis;
    xAxis << 1, 0, 0;
    yAxis << 0, 1, 0;
    zAxis << 0, 0, 1;
    vectorInterval displacementInterval;
    
    
    for (int groupIt1=0; groupIt1 < this->closeBodies.size(); ++groupIt1)
    {
        layer1Box.parentIndices = std::make_tuple(this->closeBodies[groupIt1][0], this->closeBodies[groupIt1][1]);
        layer1Box.overlaps.clear();
        
        for (int boxIt1=0; boxIt1 < this->Bodies[std::get<0>(layer1Box.parentIndices)].polyhedron.size(); ++boxIt1)
        {
            for (int boxIt2=0; boxIt2 < this->Bodies[std::get<1>(layer1Box.parentIndices)].polyhedron.size(); ++boxIt2)
            {
                displacementInterval.lower = (this->Bodies[std::get<0>(layer1Box.parentIndices)].states.r_BN_N + this->Bodies[std::get<0>(layer1Box.parentIndices)].states.dcm_NB * this->Bodies[std::get<0>(layer1Box.parentIndices)].polyhedron[boxIt1].centroid) - (this->Bodies[std::get<1>(layer1Box.parentIndices)].states.r_BN_N + this->Bodies[std::get<1>(layer1Box.parentIndices)].states.dcm_NB * this->Bodies[std::get<1>(layer1Box.parentIndices)].polyhedron[boxIt2].centroid);
                
                displacementInterval.upper = (this->Bodies[std::get<0>(layer1Box.parentIndices)].futureStates.r_BN_N + this->Bodies[std::get<0>(layer1Box.parentIndices)].futureStates.dcm_NB * this->Bodies[std::get<0>(layer1Box.parentIndices)].polyhedron[boxIt1].centroid) - (this->Bodies[std::get<1>(layer1Box.parentIndices)].futureStates.r_BN_N + this->Bodies[std::get<1>(layer1Box.parentIndices)].futureStates.dcm_NB * this->Bodies[std::get<1>(layer1Box.parentIndices)].polyhedron[boxIt2].centroid);
                
                box1.xAxisInterval.lower = this->Bodies[std::get<0>(layer1Box.parentIndices)].states.dcm_NB * xAxis;
                box1.xAxisInterval.upper = this->Bodies[std::get<0>(layer1Box.parentIndices)].futureStates.dcm_NB * xAxis;
                
                box1.yAxisInterval.lower = this->Bodies[std::get<0>(layer1Box.parentIndices)].states.dcm_NB * yAxis;
                box1.yAxisInterval.upper = this->Bodies[std::get<0>(layer1Box.parentIndices)].futureStates.dcm_NB * yAxis;
                
                box1.zAxisInterval.lower = this->Bodies[std::get<0>(layer1Box.parentIndices)].states.dcm_NB * zAxis;
                box1.zAxisInterval.upper = this->Bodies[std::get<0>(layer1Box.parentIndices)].futureStates.dcm_NB * zAxis;
                
                box1.halfSize = this->Bodies[std::get<0>(layer1Box.parentIndices)].polyhedron[boxIt1].boundingBox * this->boundingBoxFF;
                
                box2.xAxisInterval.lower = this->Bodies[std::get<1>(layer1Box.parentIndices)].states.dcm_NB * xAxis;
                box2.xAxisInterval.upper = this->Bodies[std::get<1>(layer1Box.parentIndices)].futureStates.dcm_NB * xAxis;
                
                box2.yAxisInterval.lower = this->Bodies[std::get<1>(layer1Box.parentIndices)].states.dcm_NB * yAxis;
                box2.yAxisInterval.upper = this->Bodies[std::get<1>(layer1Box.parentIndices)].futureStates.dcm_NB * yAxis;
                
                box2.zAxisInterval.lower = this->Bodies[std::get<1>(layer1Box.parentIndices)].states.dcm_NB * zAxis;
                box2.zAxisInterval.upper = this->Bodies[std::get<1>(layer1Box.parentIndices)].futureStates.dcm_NB * zAxis;
                
                box2.halfSize = this->Bodies[std::get<1>(layer1Box.parentIndices)].polyhedron[boxIt2].boundingBox * this->boundingBoxFF;
                
                if (this->SeparatingPlane(displacementInterval, box1.xAxisInterval, box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, box1.yAxisInterval, box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, box1.zAxisInterval, box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, box2.xAxisInterval, box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, box2.yAxisInterval, box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, box2.zAxisInterval, box1, box2)) continue;
                
                if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.xAxisInterval, box2.xAxisInterval), box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.xAxisInterval, box2.yAxisInterval), box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.xAxisInterval, box2.zAxisInterval), box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.yAxisInterval, box2.xAxisInterval), box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.yAxisInterval, box2.yAxisInterval), box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.yAxisInterval, box2.zAxisInterval), box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.zAxisInterval, box2.xAxisInterval), box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.zAxisInterval, box2.yAxisInterval), box1, box2)) continue;
                if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.zAxisInterval, box2.zAxisInterval), box1, box2)) continue;
                
                
                for (int subBoxIt1=0; subBoxIt1 < this->Bodies[std::get<0>(layer1Box.parentIndices)].polyhedron[boxIt1].faceBoundingBoxes.size(); ++subBoxIt1)
                {
                    for (int subBoxIt2=0; subBoxIt2 < this->Bodies[std::get<1>(layer1Box.parentIndices)].polyhedron[boxIt2].faceBoundingBoxes.size(); ++subBoxIt2)
                    {
                        displacementInterval.lower = (this->Bodies[std::get<0>(layer1Box.parentIndices)].states.r_BN_N + this->Bodies[std::get<0>(layer1Box.parentIndices)].states.dcm_NB * this->Bodies[std::get<0>(layer1Box.parentIndices)].polyhedron[boxIt1].faceCentroids[subBoxIt1]) - (this->Bodies[std::get<1>(layer1Box.parentIndices)].states.r_BN_N + this->Bodies[std::get<1>(layer1Box.parentIndices)].states.dcm_NB * this->Bodies[std::get<1>(layer1Box.parentIndices)].polyhedron[boxIt2].faceCentroids[subBoxIt2]);
                        
                        displacementInterval.upper = (this->Bodies[std::get<0>(layer1Box.parentIndices)].futureStates.r_BN_N + this->Bodies[std::get<0>(layer1Box.parentIndices)].futureStates.dcm_NB * this->Bodies[std::get<0>(layer1Box.parentIndices)].polyhedron[boxIt1].faceCentroids[subBoxIt1]) - (this->Bodies[std::get<1>(layer1Box.parentIndices)].futureStates.r_BN_N + this->Bodies[std::get<1>(layer1Box.parentIndices)].futureStates.dcm_NB * this->Bodies[std::get<1>(layer1Box.parentIndices)].polyhedron[boxIt2].faceCentroids[subBoxIt2]);
                        
                        box1.halfSize = this->Bodies[std::get<0>(layer1Box.parentIndices)].polyhedron[boxIt1].faceBoundingBoxes[subBoxIt1] * this->boundingBoxFF;
                        box2.halfSize = this->Bodies[std::get<1>(layer1Box.parentIndices)].polyhedron[boxIt2].faceBoundingBoxes[subBoxIt2] * this->boundingBoxFF;
                        
                        if (this->SeparatingPlane(displacementInterval, box1.xAxisInterval, box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, box1.yAxisInterval, box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, box1.zAxisInterval, box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, box2.xAxisInterval, box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, box2.yAxisInterval, box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, box2.zAxisInterval, box1, box2)) continue;
                        
                        if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.xAxisInterval, box2.xAxisInterval), box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.xAxisInterval, box2.yAxisInterval), box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.xAxisInterval, box2.zAxisInterval), box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.yAxisInterval, box2.xAxisInterval), box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.yAxisInterval, box2.yAxisInterval), box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.yAxisInterval, box2.zAxisInterval), box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.zAxisInterval, box2.xAxisInterval), box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.zAxisInterval, box2.yAxisInterval), box1, box2)) continue;
                        if (this->SeparatingPlane(displacementInterval, this->IntervalCrossProduct(box1.zAxisInterval, box2.zAxisInterval), box1, box2)) continue;
                        
                        layer1Box.overlaps.push_back(std::make_tuple(boxIt1, subBoxIt1, boxIt2, subBoxIt2));
                    }
                }
            }
        }
        
        if (layer1Box.overlaps.size() > 0) this->Bodies[std::get<0>(layer1Box.parentIndices)].coarseSearchList = layer1Box;
    }
    return;
}
                
bool RigidBodyContactEffector::SeparatingPlane(vectorInterval displacementInterval, vectorInterval candidateInterval, indivBoundingBox box1, indivBoundingBox box2)
{
    double lhs;
    double rhs;
    std::vector<double> tempInterval;
    
    tempInterval = this->IntervalDotProduct(candidateInterval, displacementInterval);
    lhs = std::max(abs(tempInterval[0]), abs(tempInterval[1]));
    
    tempInterval = this->IntervalDotProduct(candidateInterval, box1.xAxisInterval);
    rhs = box1.halfSize[0] * std::max(abs(tempInterval[0]), abs(tempInterval[1]));
    tempInterval = this->IntervalDotProduct(candidateInterval, box1.yAxisInterval);
    rhs += box1.halfSize[1] * std::max(abs(tempInterval[0]), abs(tempInterval[1]));
    tempInterval = this->IntervalDotProduct(candidateInterval, box1.zAxisInterval);
    rhs += box1.halfSize[2] * std::max(abs(tempInterval[0]), abs(tempInterval[1]));
    
    tempInterval = this->IntervalDotProduct(candidateInterval, box2.xAxisInterval);
    rhs += box2.halfSize[0] * std::max(abs(tempInterval[0]), abs(tempInterval[1]));
    tempInterval = this->IntervalDotProduct(candidateInterval, box2.yAxisInterval);
    rhs += box2.halfSize[1] * std::max(abs(tempInterval[0]), abs(tempInterval[1]));
    tempInterval = this->IntervalDotProduct(candidateInterval, box2.zAxisInterval);
    rhs += box2.halfSize[2] * std::max(abs(tempInterval[0]), abs(tempInterval[1]));
    
    return lhs > rhs;
}




/*! This method is the hub for the collision detection algorithm. It applies the Separating Axis Theorem between every convex shape on the main body and the external body to calcuate where and when there will be a collision. If there is already penetration, it detects that too and calculates the needed information to compute the forces and torques.
@return If there is any collision/penetration
@param foriegnBody The geometry information of the external body
@param bodyIndex Identifier for which external body is being checked
*/
bool RigidBodyContactEffector::Overlap(geometry foriegnBody, int bodyIndex)
{
    // - Declare oh-so-many variables
    faceQuery faceA;
    faceQuery faceB;
    edgeQuery edge;
    std::vector<std::vector<int>> edgePair;
    double timeToContact;
    Eigen::Vector3d faceContactPoint_N;
    Eigen::Vector3d faceAContactPoint_N;
    Eigen::Vector3d faceBContactPoint_N;
    Eigen::Vector3d mainEdgeContact_N;
    Eigen::Vector3d otherEdgeContact_N;
    Eigen::Vector3d sigma_BprimeN;
    Eigen::Matrix3d dcm_BprimeN;
    Eigen::Vector3d sigma_BN;
    Eigen::Vector3d colPlaneVec1;
    Eigen::Vector3d colPlaneVec2;
    Eigen::Vector3d colPlaneNormal;
    Eigen::Vector3d aggNormals;
    Eigen::Vector3d zDirection;
    Eigen::Vector3d colinCenter;
    Eigen::Vector3d colinTestVec;
    Eigen::Vector3d colinExtreme1;
    Eigen::Vector3d colinExtreme2;
    zDirection << 0, 0, 1;
    Eigen::MRPd sigma_ColPlaneToXY;
    std::vector<Eigen::Vector3d> colPlanePoints;
    std::vector<Eigen::Vector3d> colConvexArea;
    double colZcoord;
    int colPlaneVecIter;
    int colinear;
    int inTriangleA;
    int inTriangleB;
    contactDetail newCollisionPoint;
    penetrationDetail newPenetration;
    std::vector<contactDetail> currentCollisionList;
    newCollisionPoint.otherBodyIndex = bodyIndex;
    newPenetration.otherBodyIndex = bodyIndex;
    bool contactPointTaken;
    
    // - Predict the main body's attitude at the end of this time step
    sigma_BprimeN = 0.25 * this->mainBody.states.sigma_BN.Bmat() * this->mainBody.states.omega_BN_B * this->simTimeStep;
    sigma_BN = (Eigen::Vector3d) this->mainBody.states.sigma_BN.coeffs();
    sigma_BprimeN = sigma_BN + sigma_BprimeN;
    if (sigma_BprimeN.norm() > 1.0)
    {
        sigma_BprimeN = -sigma_BprimeN / sigma_BprimeN.dot(sigma_BprimeN);
    }
    this->mainBody.states.sigma_BprimeB = ((1.0 - this->mainBody.states.sigma_BN.squaredNorm()) *  sigma_BprimeN - (1 - sigma_BprimeN.squaredNorm()) * sigma_BN + 2 * sigma_BprimeN.cross( sigma_BN)) / (1 + this->mainBody.states.sigma_BN.squaredNorm() * sigma_BprimeN.squaredNorm() + 2 * sigma_BN.dot(sigma_BprimeN));
    this->mainBody.states.dcm_BprimeB = this->mainBody.states.sigma_BprimeB.toRotationMatrix().transpose();
    
    // - Predict the external body's attitude at the end of this time step
    dcm_BprimeN = ((-foriegnBody.states.omegaTilde_BN_B * foriegnBody.states.dcm_BN) * this->simTimeStep) + foriegnBody.states.dcm_BN;
    foriegnBody.states.dcm_BprimeB = dcm_BprimeN * foriegnBody.states.dcm_BN.transpose();
    foriegnBody.states.sigma_BprimeB = eigenC2MRP(foriegnBody.states.dcm_BprimeB);
    
    // - Loop through every combination of main body and external body polyhedra
    for (int mainShapeIt=0; mainShapeIt<this->mainBody.polyhedron.size(); ++mainShapeIt)
    {
        //        currentCollisionList.clear();
        //        newCollisionPoint.area = 0;
        for (int otherShapeIt=0; otherShapeIt<foriegnBody.polyhedron.size(); ++otherShapeIt)
        {
            currentCollisionList.clear();
            newCollisionPoint.area = 0;
            newCollisionPoint.penetrationEnergy = 0.0;
            
            // - Find the possible penetration of a vertex on the external polyhedron with a face on the main polyhedron
            faceA = this->QueryFaceDirection(this->mainBody.vertices, this->mainBody.polyhedron[mainShapeIt], this->mainBody.states, foriegnBody.vertices, foriegnBody.polyhedron[otherShapeIt], foriegnBody.states);
            
            // - Find the possible penetration of a vertex on the main polyhedron with a face on the external polyhedron
            faceB = this->QueryFaceDirection(foriegnBody.vertices, foriegnBody.polyhedron[otherShapeIt], foriegnBody.states, this->mainBody.vertices, this->mainBody.polyhedron[mainShapeIt], this->mainBody.states);
            
            // - Find the possible penetration of an edge from each polyhedron
            edge = this->QueryEdgeDirection(this->mainBody.vertices, this->mainBody.polyhedron[mainShapeIt], this->mainBody.states, foriegnBody.vertices, foriegnBody.polyhedron[otherShapeIt], foriegnBody.states);
            
            // - Check if the polyhedra are not already intersecting
            if ((faceA.separation > 0.0) || (faceB.separation > 0.0) || (edge.separation > 0.0) || ((faceA.supportIndex == -1) && (faceB.supportIndex == -1) && edge.edgePair.empty()) )
            {
                // - Loop through each face on the main polyhedra, and each vertex on the external polyhedra
                for (int mainFaceIt=0; mainFaceIt<this->mainBody.polyhedron[mainShapeIt].faceTriangles.size(); ++mainFaceIt)
                {
                    for (int otherVertIt=0; otherVertIt<foriegnBody.polyhedron[otherShapeIt].uniqueVertIndices.size(); ++otherVertIt)
                    {
                        // - Find the possible collision of a vertex on the external polyhedron with a face on the main polyhedron
                        timeToContact = this->WhenFaceContact(this->mainBody.polyhedron[mainShapeIt].faceTriangles[mainFaceIt], this->mainBody.vertices, foriegnBody.polyhedron[otherShapeIt].uniqueVertIndices[otherVertIt], foriegnBody.vertices, this->mainBody.states, foriegnBody.states, this->simTimeStep, &faceContactPoint_N);
                        // - If there will be a collision, add it to the list of all collision points
                        if (timeToContact > -1)
                        {
                            newCollisionPoint.timeToContact = timeToContact;
                            newCollisionPoint.mainContactPoint = this->mainBody.states.dcm_BN * (faceContactPoint_N  - this->mainBody.states.r_BN_N);
                            newCollisionPoint.otherContactPoint = foriegnBody.vertices[foriegnBody.polyhedron[otherShapeIt].uniqueVertIndices[otherVertIt]];
                            newCollisionPoint.contactNormal = this->mainBody.states.dcm_NB * this->mainBody.polyhedron[mainShapeIt].faceNormals[mainFaceIt];
                            newCollisionPoint.contactNormal = newCollisionPoint.contactNormal.normalized();
                            if (newCollisionPoint.contactNormal.dot((this->mainBody.states.v_BN_N + eigenTilde(this->mainBody.states.dcm_NB * this->mainBody.states.omega_BN_B) * (faceContactPoint_N  - this->mainBody.states.r_BN_N)).normalized()) > 0.0)
                            {
                                currentCollisionList.push_back(newCollisionPoint);
                            }
                            //                            currentCollisionList.push_back(newCollisionPoint);
                        }
                    }
                }
                // - Loop through each face on the external polyhedra, and each vertex on the main polyhedra
                for (int otherFaceIt=0; otherFaceIt<foriegnBody.polyhedron[otherShapeIt].faceTriangles.size(); ++otherFaceIt)
                {
                    for (int mainVertIt=0; mainVertIt<this->mainBody.polyhedron[mainShapeIt].uniqueVertIndices.size(); ++mainVertIt)
                    {
                        // - Find the possible collision of a vertex on the main polyhedron with a face on the external polyhedron
                        timeToContact = this->WhenFaceContact(foriegnBody.polyhedron[otherShapeIt].faceTriangles[otherFaceIt], foriegnBody.vertices, this->mainBody.polyhedron[mainShapeIt].uniqueVertIndices[mainVertIt], this->mainBody.vertices, foriegnBody.states, this->mainBody.states, this->simTimeStep, &faceContactPoint_N);
                        // - If there will be a collision, add it to the list of all collision points
                        if (timeToContact > -1)
                        {
                            newCollisionPoint.timeToContact = timeToContact;
                            newCollisionPoint.mainContactPoint = this->mainBody.vertices[this->mainBody.polyhedron[mainShapeIt].uniqueVertIndices[mainVertIt]];
                            newCollisionPoint.otherContactPoint = foriegnBody.states.dcm_BN  * (faceContactPoint_N  - foriegnBody.states.r_BN_N);
                            newCollisionPoint.contactNormal = -(foriegnBody.states.dcm_NB * foriegnBody.polyhedron[otherShapeIt].faceNormals[otherFaceIt]);
                            //                            if ((this->mainBody.states.dcm_NB * this->mainBody.polyhedron[mainShapeIt].centroid - (faceContactPoint_N  - foriegnBody.states.r_BN_N)).dot(newCollisionPoint.contactNormal) > 0.0)
                            //                            {
                            //                                newCollisionPoint.contactNormal = -newCollisionPoint.contactNormal.normalized();
                            //                            }
                            //                            currentCollisionList.push_back(newCollisionPoint);
                            if (newCollisionPoint.contactNormal.dot((this->mainBody.states.v_BN_N + eigenTilde(this->mainBody.states.dcm_NB * this->mainBody.states.omega_BN_B) * (faceContactPoint_N  - foriegnBody.states.r_BN_N)).normalized()) > 0.0)
                            {
                                currentCollisionList.push_back(newCollisionPoint);
                            }
                        }
                    }
                }
                // - Loop through each edge on both the main and external polyhedra
                for (int mainEdgeIt=0; mainEdgeIt<this->mainBody.polyhedron[mainShapeIt].edgeIndices.size(); mainEdgeIt += 2)
                {
                    for (int otherEdgeIt=0; otherEdgeIt<foriegnBody.polyhedron[otherShapeIt].edgeIndices.size(); otherEdgeIt += 2)
                    {
                        edgePair.clear();
                        edgePair.push_back( this->mainBody.polyhedron[mainShapeIt].edgeIndices[mainEdgeIt]);
                        edgePair.push_back( foriegnBody.polyhedron[otherShapeIt].edgeIndices[otherEdgeIt]);
                        // - Find the possible collision of an edge from each polyhedron
                        timeToContact = this->WhenEdgeContact(edgePair, this->mainBody.vertices, foriegnBody.vertices, this->mainBody.states, foriegnBody.states, this->simTimeStep, &mainEdgeContact_N, &otherEdgeContact_N, &colinear);
                        // - If there will be a collision, add it to the list of all collision points
                        if (timeToContact > -1)
                        {
                            newCollisionPoint.timeToContact = timeToContact;
                            newCollisionPoint.mainContactPoint = this->mainBody.states.dcm_BN  * (mainEdgeContact_N  - this->mainBody.states.r_BN_N);
                            newCollisionPoint.otherContactPoint = foriegnBody.states.dcm_BN  * (otherEdgeContact_N  - foriegnBody.states.r_BN_N);
                            // - If the edges are colinear, the contact normal must be calculated differently
                            if (colinear == 1)
                            {
                                newCollisionPoint.contactNormal = -((foriegnBody.states.dcm_NB * (foriegnBody.vertices[edgePair[1][1]] - foriegnBody.vertices[edgePair[1][0]])).cross(this->mainBody.states.dcm_NB * (this->mainBody.vertices[edgePair[0][1]] - this->mainBody.vertices[edgePair[0][0]])));
                                newCollisionPoint.contactNormal.normalize();
                                if ( newCollisionPoint.contactNormal.dot(this->mainBody.states.dcm_NB * (this->mainBody.vertices[edgePair[0][0]] - this->mainBody.polyhedron[mainShapeIt].centroid)) < 0){
                                    newCollisionPoint.contactNormal = -newCollisionPoint.contactNormal;
                                }
                            }else
                            {
                                newCollisionPoint.contactNormal = (foriegnBody.states.dcm_BN * foriegnBody.polyhedron[otherShapeIt].centroid + foriegnBody.states.r_BN_N) - (this->mainBody.states.dcm_NB * this->mainBody.polyhedron[mainShapeIt].centroid + this->mainBody.states.r_BN_N);
                                newCollisionPoint.contactNormal.normalize();
                                
                            }
                            //                            newCollisionPoint.contactNormal = (foriegnBody.states.dcm_BN * foriegnBody.polyhedron[otherShapeIt].centroid + foriegnBody.states.r_BN_N) - (this->mainBody.states.dcm_NB * this->mainBody.polyhedron[mainShapeIt].centroid + this->mainBody.states.r_BN_N);
                            //                            newCollisionPoint.contactNormal.normalize();
                            //                            currentCollisionList.push_back(newCollisionPoint);
                            if (newCollisionPoint.contactNormal.dot((this->mainBody.states.v_BN_N + eigenTilde(this->mainBody.states.dcm_NB * this->mainBody.states.omega_BN_B) * (mainEdgeContact_N  - this->mainBody.states.r_BN_N)).normalized()) > 0.0)
                            {
                                currentCollisionList.push_back(newCollisionPoint);
                            }
                        }
                    }
                }
                //            }
                //        }
                
                // - When there are more than one incoming collison point, they must be combined into a singluar point for each set of polyhedra
                timeToContact = 0;
                for (int timeIt = 0; timeIt < currentCollisionList.size(); ++timeIt)
                {
                    timeToContact += currentCollisionList[timeIt].timeToContact;
                }
                // - If there are three or more collision points, an average collision plane can be found
                if (currentCollisionList.size() >= 3)
                {
                    newCollisionPoint.timeToContact = timeToContact / currentCollisionList.size();
                    colPlaneVec1 = currentCollisionList[1].mainContactPoint - currentCollisionList[0].mainContactPoint;
                    colPlaneVec1.normalize();
                    colPlaneVecIter = 2;
                    
                    // - Find the normal vector for the collision plane
                    do
                    {
                        colPlaneVec2 = currentCollisionList[colPlaneVecIter].mainContactPoint - currentCollisionList[0].mainContactPoint;
                        colPlaneVec2.normalize();
                        colPlaneNormal = colPlaneVec1.cross(colPlaneVec2);
                        colPlaneVecIter++;
                    }
                    while ( (colPlaneNormal.norm() == 0) && (colPlaneVecIter < currentCollisionList.size()));
                    
                    colPlaneNormal.normalize();
                    if (colPlaneNormal.norm() != 0) // - A plane can actually be formed
                    {
                        // - Make sure the collision plane normal is in the same hemisphere as the rest of the collision normals
                        aggNormals << 0, 0, 0;
                        for (int pointIdx=0; pointIdx<currentCollisionList.size(); ++pointIdx)
                        {
                            aggNormals += currentCollisionList[pointIdx].mainContactPoint - this->mainBody.polyhedron[mainShapeIt].centroid;
                        }
                        
                        if ( colPlaneNormal.dot(aggNormals) < 0)
                        {
                            colPlaneNormal = -colPlaneNormal;
                        }
                        
                        // - Find the rotation to express all the collision points in-plane
                        sigma_ColPlaneToXY.setFromTwoVectors(colPlaneNormal, zDirection);
                        
                        // - Transform all collision points to be expressed in-plane
                        colPlanePoints.clear();
                        colZcoord = 0;
                        for (int buildInd = 0; buildInd < currentCollisionList.size(); buildInd++)
                        { colPlanePoints.push_back(sigma_ColPlaneToXY.toRotationMatrix().transpose() * currentCollisionList[buildInd].mainContactPoint);
                            colZcoord += colPlanePoints[buildInd][2];
                            colPlanePoints[buildInd][2] = 0.0;
                        }
                        colZcoord = colZcoord / currentCollisionList.size();
                        
                        // - Create convex hull within the collision plane
                        colConvexArea = findConvexHull(colPlanePoints);
                        // - If a convex hull cannot be found, these points are colinear
                        if (colConvexArea.empty())
                        {
                            goto JumpToColin;
                            continue;
                        }
                        // - Determine the area of the collision
                        newCollisionPoint.area = 0;
                        for (int areaInd = 0; areaInd < colConvexArea.size()-1; areaInd++)
                        {
                            newCollisionPoint.area += colConvexArea[areaInd][0] * colConvexArea[areaInd+1][1] - colConvexArea[areaInd+1][0] * colConvexArea[areaInd][1];
                        }
                        newCollisionPoint.area += colConvexArea[colConvexArea.size()-1][0] * colConvexArea[0][1] - colConvexArea[0][0] * colConvexArea[colConvexArea.size()-1][1];
                        newCollisionPoint.area = newCollisionPoint.area / 2.0;
                        
                        // - The centroid of the collision area becomes the new collision point
                        newCollisionPoint.mainContactPoint << 0, 0, colZcoord;
                        for (int coordInd = 0; coordInd < colConvexArea.size()-1; coordInd++)
                        {
                            newCollisionPoint.mainContactPoint[0] += (colConvexArea[coordInd][0] + colConvexArea[coordInd+1][0]) * (colConvexArea[coordInd][0] * colConvexArea[coordInd+1][1] - colConvexArea[coordInd+1][0] * colConvexArea[coordInd][1]);
                            newCollisionPoint.mainContactPoint[1] += (colConvexArea[coordInd][1] + colConvexArea[coordInd+1][1]) * (colConvexArea[coordInd][0] * colConvexArea[coordInd+1][1] - colConvexArea[coordInd+1][0] * colConvexArea[coordInd][1]);
                        }
                        newCollisionPoint.mainContactPoint[0] += (colConvexArea[colConvexArea.size()-1][0] + colConvexArea[0][0]) * (colConvexArea[colConvexArea.size()-1][0] * colConvexArea[0][1] - colConvexArea[0][0] * colConvexArea[colConvexArea.size()-1][1]);
                        newCollisionPoint.mainContactPoint[1] += (colConvexArea[colConvexArea.size()-1][1] + colConvexArea[0][1]) * (colConvexArea[colConvexArea.size()-1][0] * colConvexArea[0][1] - colConvexArea[0][0] * colConvexArea[colConvexArea.size()-1][1]);
                        newCollisionPoint.mainContactPoint[0] = newCollisionPoint.mainContactPoint[0] / (6 * newCollisionPoint.area);
                        newCollisionPoint.mainContactPoint[1] = newCollisionPoint.mainContactPoint[1] / (6 * newCollisionPoint.area);
                        
                        // - Rotate new collision point back into the polyheadra frame
                        newCollisionPoint.mainContactPoint = (sigma_ColPlaneToXY.toRotationMatrix()) * newCollisionPoint.mainContactPoint;
                        newCollisionPoint.contactNormal = this->mainBody.states.dcm_NB * colPlaneNormal;
                        newCollisionPoint.otherContactPoint = foriegnBody.states.dcm_BN * this->mainBody.states.dcm_NB * newCollisionPoint.mainContactPoint;
                        currentCollisionList.push_back(newCollisionPoint);
                    }else
                    {
                        // - If the collision points are colinear, then find their midpoint
                    JumpToColin:
                        colinCenter << 0, 0, 0;
                        newCollisionPoint.contactNormal << 0, 0, 0;
                        for (int sumInx=0; sumInx < currentCollisionList.size(); ++sumInx)
                        {
                            colinCenter += currentCollisionList[sumInx].mainContactPoint;
                            newCollisionPoint.contactNormal += currentCollisionList[sumInx].contactNormal;
                        }
                        colinCenter = colinCenter / currentCollisionList.size();
                        colinExtreme1 = currentCollisionList[0].mainContactPoint;
                        for (int maxInx=1; maxInx < currentCollisionList.size(); ++maxInx)
                        {
                            colinTestVec = currentCollisionList[maxInx].mainContactPoint - colinCenter;
                            if (colinTestVec.norm() > (colinExtreme1 - colinCenter).norm())
                            {
                                colinExtreme1 = currentCollisionList[maxInx].mainContactPoint;
                            }
                        }
                        
                        for (int maxInx=0; maxInx < currentCollisionList.size(); ++maxInx){
                            colinTestVec = currentCollisionList[maxInx].mainContactPoint - colinCenter;
                            if (colinTestVec.dot(colinExtreme1 - colinCenter) < 0) {
                                colinExtreme2 = currentCollisionList[maxInx].mainContactPoint;
                                break;
                            }
                        }
                        for (int maxInx=1; maxInx < currentCollisionList.size(); ++maxInx){
                            colinTestVec = currentCollisionList[maxInx].mainContactPoint - colinCenter;
                            if ((colinTestVec.norm() > (colinExtreme2 - colinCenter).norm()) && (colinTestVec.dot(colinExtreme1 - colinCenter) < 0)) {
                                colinExtreme2 = currentCollisionList[maxInx].mainContactPoint;
                            }
                        }
                        colinCenter = (colinExtreme1 + colinExtreme2) / 2;
                        newCollisionPoint.mainContactPoint =  colinCenter;
                        newCollisionPoint.otherContactPoint = foriegnBody.states.dcm_BN * this->mainBody.states.dcm_NB * newCollisionPoint.mainContactPoint;
                        newCollisionPoint.contactNormal.normalize();
                        currentCollisionList.push_back(newCollisionPoint);
                    }
                    // - If there are only two collision points, their midpoint collision is MUCH easier to determine
                }else if (currentCollisionList.size() == 2)
                {
                    newCollisionPoint.timeToContact = timeToContact / currentCollisionList.size();
                    newCollisionPoint.mainContactPoint = (currentCollisionList[0].mainContactPoint + currentCollisionList[1].mainContactPoint) / 2;
                    newCollisionPoint.otherContactPoint = (currentCollisionList[0].otherContactPoint + currentCollisionList[1].otherContactPoint) / 2;
                    newCollisionPoint.contactNormal = (currentCollisionList[0].contactNormal + currentCollisionList[1].contactNormal).normalized();
                    currentCollisionList.push_back(newCollisionPoint);
                }
                if (currentCollisionList.empty() == false)
                {
                    contactPointTaken = false;
                    if (this->mainBody.collisionPoints.empty() == false)
                    {
                        for ( int bodyIt = 0; bodyIt < this->mainBody.collisionPoints.size(); ++bodyIt)
                        {
                            if ((int(ceil(this->mainBody.collisionPoints[bodyIt].back().mainContactPoint[0] * pow(10.0, 9))) == int(ceil(currentCollisionList.back().mainContactPoint[0] * pow(10.0, 9)))) && (int(ceil(this->mainBody.collisionPoints[bodyIt].back().mainContactPoint[1] * pow(10.0, 9))) == int(ceil(currentCollisionList.back().mainContactPoint[1] * pow(10.0, 9)))) && (int(ceil(this->mainBody.collisionPoints[bodyIt].back().mainContactPoint[2] * pow(10.0, 9))) == int(ceil(currentCollisionList.back().mainContactPoint[3] * pow(10.0, 9)))))
                            {
                                contactPointTaken = true;
                            }
                        }
                    }
                    if (contactPointTaken == false)
                    {
                        this->mainBody.collisionPoints.push_back(currentCollisionList);
                    }
                }
                
                // - If the polyhedra are already intersecting, find the points of deepest penetration
            }else
            {
                
                inTriangleA = -1;
                inTriangleB = -1;
                colinear = -1;
                
                // - If penetration was found of a vertex on the external polyhedron with a face on the main polyhedron, then find where in the face the deepest penetration is projected to
                if  (faceA.supportIndex != -1)
                {
                    inTriangleA = this->PointInTriangle( (foriegnBody.states.dcm_NB * foriegnBody.vertices[faceA.supportIndex]) + foriegnBody.states.r_BN_N, (this->mainBody.states.dcm_NB *  this->mainBody.vertices[this->mainBody.polyhedron[mainShapeIt].faceTriangles[faceA.faceIndex][0]]) + this->mainBody.states.r_BN_N, (this->mainBody.states.dcm_NB *  this->mainBody.vertices[this->mainBody.polyhedron[mainShapeIt].faceTriangles[faceA.faceIndex][1]]) + this->mainBody.states.r_BN_N, (this->mainBody.states.dcm_NB *  this->mainBody.vertices[this->mainBody.polyhedron[mainShapeIt].faceTriangles[faceA.faceIndex][2]]) + this->mainBody.states.r_BN_N, &faceAContactPoint_N);
                    
                }
                // - If penetration was found of a vertex on the main polyhedron with a face on the external polyhedron, then find where in the face the deepest penetration is projected to
                if  (faceB.supportIndex != -1)
                {
                    inTriangleB = this->PointInTriangle( (this->mainBody.states.dcm_NB * this->mainBody.vertices[faceB.supportIndex]) + this->mainBody.states.r_BN_N, (foriegnBody.states.dcm_NB *  foriegnBody.vertices[foriegnBody.polyhedron[otherShapeIt].faceTriangles[faceB.faceIndex][0]]) + foriegnBody.states.r_BN_N, (foriegnBody.states.dcm_NB *  foriegnBody.vertices[foriegnBody.polyhedron[otherShapeIt].faceTriangles[faceB.faceIndex][1]]) + foriegnBody.states.r_BN_N, (foriegnBody.states.dcm_NB *  foriegnBody.vertices[foriegnBody.polyhedron[otherShapeIt].faceTriangles[faceB.faceIndex][2]]) + foriegnBody.states.r_BN_N, &faceBContactPoint_N);
                    
                }
                // - If penetration was found between two edges, find where on each edge the deepest penetration is projected to
                if (edge.edgePair.empty() == false)
                {
                    colinear = this->LineLineDistance( (this->mainBody.states.dcm_NB * this->mainBody.vertices[edge.edgePair[0][0]]) + this->mainBody.states.r_BN_N, (this->mainBody.states.dcm_NB * this->mainBody.vertices[edge.edgePair[0][1]]) + this->mainBody.states.r_BN_N, (foriegnBody.states.dcm_NB * foriegnBody.vertices[edge.edgePair[1][0]]) + foriegnBody.states.r_BN_N, (foriegnBody.states.dcm_NB * foriegnBody.vertices[edge.edgePair[1][1]]) + foriegnBody.states.r_BN_N, &mainEdgeContact_N, &otherEdgeContact_N);
                    
                }
                
                // - The next three if blocks determine where the deepest, real penetration is, and assigns the relevant information accordingly
                if (((faceA.separation > faceB.separation) || (inTriangleB == -1)) && ((faceA.separation > edge.separation) || (colinear < 0)) && (inTriangleA > -1))
                {
                    newPenetration.mainContactPoint = this->mainBody.states.dcm_BN * (faceAContactPoint_N - this->mainBody.states.r_BN_N);
                    newPenetration.springLine_N = this->mainBody.states.dcm_NB * this->mainBody.polyhedron[mainShapeIt].faceNormals[faceA.faceIndex];
                    newPenetration.contactCase = 0;
                    newPenetration.faceData = faceA;
                    
                    newCollisionPoint.mainContactPoint = this->mainBody.states.dcm_BN * (faceAContactPoint_N - this->mainBody.states.r_BN_N);
                    newCollisionPoint.otherContactPoint = foriegnBody.vertices[faceA.supportIndex];
                    newCollisionPoint.contactNormal = this->mainBody.states.dcm_NB * this->mainBody.polyhedron[mainShapeIt].faceNormals[faceA.faceIndex];
                    newCollisionPoint.contactNormal = newCollisionPoint.contactNormal.normalized();
                    newCollisionPoint.timeToContact = 0.0;
//                    newCollisionPoint.penetrationEnergy = -(faceAContactPoint_N - ((foriegnBody.states.dcm_NB * foriegnBody.vertices[faceA.supportIndex]) + foriegnBody.states.r_BN_N)).norm();
                    
                    if (newCollisionPoint.contactNormal.dot((this->mainBody.states.v_BN_N + eigenTilde(this->mainBody.states.dcm_NB * this->mainBody.states.omega_BN_B) * (faceAContactPoint_N - this->mainBody.states.r_BN_N)).normalized()) > 0.0)
                    {
                        currentCollisionList.push_back(newCollisionPoint);
                        this->mainBody.collisionPoints.push_back(currentCollisionList);
                    }
                    
                    //                    currentCollisionList.push_back(newCollisionPoint);
                    
                    this->mainBody.penetrationData.push_back(newPenetration);
                    continue;
                }
                
                if (((faceB.separation > faceA.separation) || (inTriangleA == -1)) && ((faceB.separation > edge.separation) || (colinear < 0)) && (inTriangleB > -1))
                {
                    newPenetration.otherContactPoint = foriegnBody.states.dcm_BN * (faceBContactPoint_N - foriegnBody.states.r_BN_N);
                    newPenetration.springLine_N = foriegnBody.states.dcm_NB * -foriegnBody.polyhedron[otherShapeIt].faceNormals[faceB.faceIndex];
                    newPenetration.contactCase = 1;
                    newPenetration.faceData = faceB;
                    
                    newCollisionPoint.mainContactPoint = this->mainBody.vertices[faceB.supportIndex];
                    newCollisionPoint.otherContactPoint = foriegnBody.states.dcm_BN * (faceBContactPoint_N - foriegnBody.states.r_BN_N);
                    newCollisionPoint.contactNormal = foriegnBody.states.dcm_NB * -foriegnBody.polyhedron[otherShapeIt].faceNormals[faceB.faceIndex];
                    newCollisionPoint.contactNormal = newCollisionPoint.contactNormal.normalized();
                    newCollisionPoint.timeToContact = 0.0;
//                    newCollisionPoint.penetrationEnergy = -(((this->mainBody.states.dcm_NB * this->mainBody.vertices[faceB.supportIndex]) + this->mainBody.states.r_BN_N) - faceBContactPoint_N).norm();
                    
                    if (newCollisionPoint.contactNormal.dot((this->mainBody.states.v_BN_N + eigenTilde(this->mainBody.states.dcm_NB * this->mainBody.states.omega_BN_B) * (this->mainBody.states.dcm_NB * this->mainBody.vertices[faceB.supportIndex])).normalized()) > 0.0)
                    {
                        currentCollisionList.push_back(newCollisionPoint);
                        this->mainBody.collisionPoints.push_back(currentCollisionList);
                    }
                    //                    currentCollisionList.push_back(newCollisionPoint);
                    
                    this->mainBody.penetrationData.push_back(newPenetration);
                    continue;
                }
                
                if (((edge.separation > faceB.separation) || (inTriangleB == -1)) && ((edge.separation > faceA.separation) || (inTriangleA == -1)) && (colinear > 0))
                {
                    newPenetration.mainContactPoint = this->mainBody.states.dcm_BN * (mainEdgeContact_N - this->mainBody.states.r_BN_N);
                    newPenetration.otherContactPoint = foriegnBody.states.dcm_BN * (otherEdgeContact_N - foriegnBody.states.r_BN_N);
                    newPenetration.springLine_N =  (mainEdgeContact_N - otherEdgeContact_N).normalized();
                    newPenetration.contactCase = 2;
                    newPenetration.edgeData = edge;
                    
                    newCollisionPoint.mainContactPoint = this->mainBody.states.dcm_BN * (mainEdgeContact_N  - this->mainBody.states.r_BN_N);
                    newCollisionPoint.otherContactPoint = foriegnBody.states.dcm_BN * (otherEdgeContact_N - foriegnBody.states.r_BN_N);
                    newCollisionPoint.contactNormal = (mainEdgeContact_N - otherEdgeContact_N).normalized();
                    newCollisionPoint.timeToContact = 0.0;
//                    newCollisionPoint.penetrationEnergy = -(mainEdgeContact_N  - otherEdgeContact_N).norm();
                    if (newCollisionPoint.contactNormal.dot((this->mainBody.states.v_BN_N + eigenTilde(this->mainBody.states.dcm_NB * this->mainBody.states.omega_BN_B) * (mainEdgeContact_N - this->mainBody.states.r_BN_N)).normalized()) > 0.0)
                    {
                        currentCollisionList.push_back(newCollisionPoint);
                        this->mainBody.collisionPoints.push_back(currentCollisionList);
                    }
                    
                    //                    currentCollisionList.push_back(newCollisionPoint);
                    
                    
                    this->mainBody.penetrationData.push_back(newPenetration);
                    continue;
                }
            }
        }
    }
    return true;
}


/*! This method determines if the intersection of two arcs on a Guass sphere forms a Minkowski face.
@return Bool if the arcs form a Minkowski face
@param edgeA Vector defining the line segment of edge A
@param edgeB Vector defining the line segment of edge B
@param a Normal vector of the first face attached to edge A
@param b Normal vector of the second face attached to edge A
@param c Negative of normal vector of the first face attached to edge B
@param d Negative of normal vector of the second face attached to edge B
*/
bool RigidBodyContactEffector::IsMinkowskiFace(Eigen::Vector3d edgeA, Eigen::Vector3d edgeB, Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Eigen::Vector3d d)
{
    // - Test if arcs ab and cd intersect on the unit sphere
    float cba = c.dot(edgeA);
    float dba = d.dot(edgeA);
    float adc = a.dot(edgeB);
    float bdc = b.dot(edgeB);
    
    return cba * dba < 0 && adc * bdc < 0 && cba * bdc > 0;
}

/*! This method finds the deepest penetration of two edges between two polyhedra.
@return Struct containing information about the deepest penetration
@param verticesA The vertices of body A
@param polyA The primitive information for polyhedron A (which is part of body A)
@param stateA The state information for body A
@param verticesB The vertices of body B
@param polyB The primitive information for polyhedron B (which is part of body B)
@param stateB The state information for body B
*/
edgeQuery RigidBodyContactEffector::QueryEdgeDirection(std::vector<Eigen::Vector3d> verticesA, halfEdge polyA, dynamicData stateA, std::vector<Eigen::Vector3d> verticesB, halfEdge polyB, dynamicData stateB)
{
    Eigen::Vector3d normalA;
    Eigen::Vector3d normalB;
    Eigen::Vector3d normalC;
    Eigen::Vector3d normalD;
    Eigen::Vector3d edgeDirectionA;
    Eigen::Vector3d edgeDirectionB;
    Eigen::Vector3d edgeHeadA;
    Eigen::Vector3d edgeHeadB;
    Eigen::Vector3d edgeCross;
    Eigen::Vector3d edgeCrossNorm;
    Eigen::Vector3d zeroVec(0.0f, 0.0f, 0.0f);
    double newSeparation;
    edgeQuery result;
    result.separation = -1000.0;
    
    for (int indexA=0; indexA<polyA.edgeIndices.size(); indexA += 2)
    {
        normalA = stateA.dcm_NB * polyA.faceNormals[polyA.faceIndices[indexA]];
        normalB = stateA.dcm_NB * polyA.faceNormals[polyA.faceIndices[indexA+1]];
        edgeDirectionA = stateA.dcm_NB * (verticesA[polyA.edgeIndices[indexA][1]] - verticesA[polyA.edgeIndices[indexA][0]]);
        for (int indexB=0; indexB<polyB.edgeIndices.size(); indexB += 2)
        {
            normalC = stateB.dcm_NB * polyB.faceNormals[polyB.faceIndices[indexB]];
            normalD = stateB.dcm_NB * polyB.faceNormals[polyB.faceIndices[indexB+1]];
            edgeDirectionB = stateB.dcm_NB * (verticesB[polyB.edgeIndices[indexB][1]] - verticesB[polyB.edgeIndices[indexB][0]]);
            
            if(this->IsMinkowskiFace(edgeDirectionA, edgeDirectionB, normalA, normalB, -normalC, -normalD))
            {
                edgeHeadA = stateA.dcm_NB * verticesA[polyA.edgeIndices[indexA][1]];
                edgeHeadB = (stateB.dcm_NB * verticesB[polyB.edgeIndices[indexB][1]]) + (stateB.r_BN_N - stateA.r_BN_N);
                edgeCross = edgeDirectionA.cross(edgeDirectionB);
                if (edgeCross == zeroVec) continue;

                edgeCrossNorm = edgeCross.normalized();
                if (edgeCrossNorm.dot(edgeHeadA)<0.0f)
                    edgeCrossNorm = -edgeCrossNorm;

                newSeparation = edgeCrossNorm.dot(edgeHeadB - edgeHeadA);
                if ( newSeparation > result.separation)
                {
                    result.separation = newSeparation;
                    result.edgePair.clear();
                    result.edgePair.push_back(polyA.edgeIndices[indexA]);
                    result.edgePair.push_back(polyB.edgeIndices[indexB]);
                }
            }
        }
    }
    return result;
}

/*! This method finds the deepest penetration of vertices in polyhedron B and faces in polyhedron A.
@return Struct containing information about the deepest penetration
@param verticesA The vertices of body A
@param polyA The primitive information for polyhedron A (which is part of body A)
@param stateA The state information for body A
@param verticesB The vertices of body B
@param polyB The primitive information for polyhedron B (which is part of body B)
@param stateB The state information for body B
*/
faceQuery RigidBodyContactEffector::QueryFaceDirection(std::vector<Eigen::Vector3d> verticesA, halfEdge polyA, dynamicData stateA, std::vector<Eigen::Vector3d> verticesB, halfEdge polyB, dynamicData stateB)
{
    Eigen::Vector3d edgeHead;
    Eigen::Vector3d supportPoint;
    Eigen::Vector3d contactPoint;
    double projection;
    double bestProjection;
    double supportDistance;
    int supportIdx;
    int onTriangle;
    faceQuery result;
    result.separation = -1000.0;
    result.supportIndex = -1;
    
    for (int indexA=0; indexA<polyA.faceNormals.size(); ++indexA)
    {
        bestProjection = -1000.0;
        for (int indexB=0; indexB<polyB.edgeIndices.size(); ++indexB)
        {
            edgeHead =  (stateB.dcm_NB * verticesB[polyB.edgeIndices[indexB][0]]);
            projection = (stateA.dcm_NB * -polyA.faceNormals[indexA]).dot(edgeHead);
            
            if ( projection > bestProjection)
            {
                bestProjection = projection;
                supportPoint = edgeHead + stateB.r_BN_N;
                supportIdx = polyB.edgeIndices[indexB][0];
            }
        }
        
        onTriangle = this->PointInTriangle(supportPoint, (stateA.dcm_NB * verticesA[polyA.faceTriangles[indexA][0]]) + stateA.r_BN_N, (stateA.dcm_NB * verticesA[polyA.faceTriangles[indexA][1]]) + stateA.r_BN_N, (stateA.dcm_NB * verticesA[polyA.faceTriangles[indexA][2]]) + stateA.r_BN_N, &contactPoint);

        supportDistance = ( stateA.dcm_NB * polyA.faceNormals[indexA]).dot(supportPoint - contactPoint);

        if ( ((supportDistance > result.separation) && (onTriangle > -1)) || ((supportDistance > result.separation) && (supportDistance > 0)))
        {
            result.separation = supportDistance;
            result.faceIndex = indexA;
            result.supportIndex = supportIdx;
        }
//        if (supportDistance > result.separation)
//        {
//            result.separation = supportDistance;
//            result.faceIndex = indexA;
//            result.supportIndex = supportIdx;
//        }
    }
    return result;
}

Eigen::Vector3d RigidBodyContactEffector::MakeIntervalValue(Eigen::Vector3d vertex_B, dynamicData dynamics, double time)
{
    
    Eigen::Vector3d vertex_N = ((dynamics.dcm_NB * vertex_B + dynamics.dcm_NB * (dynamics.omegaTilde_BN_B * time) * vertex_B) + dynamics.r_BN_N) + dynamics.v_BN_N * time;
    
    return vertex_N;
}

vectorInterval RigidBodyContactEffector::MakeIntervalValues(Eigen::Vector3d vertex_B, dynamicData dynamics, double screwAngle, Eigen::Matrix3d screwRot, Eigen::Vector3d screwOffset, double screwDistance, std::vector<double> timeInterval)
{
    vectorInterval result;

    if (screwAngle == 0)
    {
        Eigen::Vector3d displacement = dynamics.v_BN_N * this->simTimeStep;
        result.lower = dynamics.dcm_NB * vertex_B + dynamics.r_BN_N + displacement * timeInterval[0];
        result.upper = dynamics.dcm_NB * vertex_B + dynamics.r_BN_N + displacement * timeInterval[1];
        return result;
    }
    
    Eigen::Vector3d Px = (screwRot *  vertex_B) + screwOffset;
    std::vector<double> cosInt = this->IntervalCosine(timeInterval[0] * screwAngle, timeInterval[1] * screwAngle);
    std::vector<double> sinInt = this->IntervalSine(timeInterval[0] * screwAngle, timeInterval[1] * screwAngle);
    
    result.lower[0] = cosInt[0] * Px[0] + sinInt[0] * Px[1];
    result.lower[1] = -sinInt[0] * Px[0] + cosInt[0] * Px[1];
    result.lower[2] = Px[2] + screwDistance * timeInterval[0];
    
    result.upper[0] = cosInt[1] * Px[0] + sinInt[1] * Px[1];
    result.upper[1] = -sinInt[1] * Px[0] + cosInt[1] * Px[1];
    result.upper[2] = Px[2] + screwDistance * timeInterval[1];
    
    result.lower = (screwRot.transpose() * result.lower) - (screwRot.transpose() * screwOffset);
    result.upper = (screwRot.transpose() * result.upper) - (screwRot.transpose() * screwOffset);
    
    result.lower = dynamics.dcm_NB * result.lower + dynamics.r_BN_N;
    result.upper = dynamics.dcm_NB * result.upper + dynamics.r_BN_N;
    
    return result;
}

double RigidBodyContactEffector::FindEdgeIntervalThreshold(Eigen::Vector3d vertexA0_B, Eigen::Vector3d vertexA1_B, dynamicData dynamicsA, Eigen::Vector3d vertexB0_B, Eigen::Vector3d vertexB1_B, dynamicData dynamicsB, double maxError)
{
    std::vector<double> thresholdList;
    Eigen::Vector3d vertexVelocity_N;
    
    
    vertexVelocity_N = (dynamicsA.dcm_NB * dynamicsA.omegaTilde_BN_B * vertexA0_B) + dynamicsA.v_BN_N;
    thresholdList.push_back(maxError / vertexVelocity_N.norm());
    vertexVelocity_N = (dynamicsA.dcm_NB * dynamicsA.omegaTilde_BN_B * vertexA1_B) + dynamicsA.v_BN_N;
    thresholdList.push_back(maxError / vertexVelocity_N.norm());
    vertexVelocity_N = (dynamicsB.dcm_NB * dynamicsB.omegaTilde_BN_B * vertexB0_B) + dynamicsB.v_BN_N;
    thresholdList.push_back(maxError / vertexVelocity_N.norm());
    vertexVelocity_N = (dynamicsB.dcm_NB * dynamicsB.omegaTilde_BN_B * vertexB1_B) + dynamicsB.v_BN_N;
    thresholdList.push_back(maxError / vertexVelocity_N.norm());
    
    return *std::min_element(thresholdList.begin(), thresholdList.end());
}

double RigidBodyContactEffector::FindEdgeIntervalThresholds(Eigen::Vector3d vertexA0_B, Eigen::Vector3d vertexA1_B, dynamicData dynamicsA, double screwAngleA, Eigen::Matrix3d screwRotA, Eigen::Vector3d screwOffsetA, double screwDistanceA, Eigen::Vector3d vertexB0_B, Eigen::Vector3d vertexB1_B, dynamicData dynamicsB, double screwAngleB, Eigen::Matrix3d screwRotB, Eigen::Vector3d screwOffsetB, double screwDistanceB, double maxError)
{
    std::vector<double> thresholdList;
    Eigen::Vector3d vertex_S;
    
    if (screwAngleA > 0)
    {
        vertex_S = screwRotA * vertexA0_B + screwOffsetA;
        thresholdList.push_back(maxError / sqrt( pow(screwDistanceA, 2) + pow(screwAngleA, 2) * (pow(vertex_S[0], 2) + pow(vertex_S[1], 2))));
        vertex_S = screwRotA * vertexA1_B + screwOffsetA;
        thresholdList.push_back(maxError / sqrt( pow(screwDistanceA, 2) + pow(screwAngleA, 2) * (pow(vertex_S[0], 2) + pow(vertex_S[1], 2))));
    }else if (dynamicsA.v_BN_N.norm() > 0)
    {
        thresholdList.push_back(maxError / (dynamicsA.v_BN_N.norm() * this->simTimeStep));
    }
    
    if (screwAngleB > 0)
    {
        vertex_S = screwRotB * vertexB0_B + screwOffsetB;
        thresholdList.push_back(maxError / sqrt( pow(screwDistanceB, 2) + pow(screwAngleB, 2) * (pow(vertex_S[0], 2) + pow(vertex_S[1], 2))));
        vertex_S = screwRotB * vertexB1_B + screwOffsetB;
        thresholdList.push_back(maxError / sqrt( pow(screwDistanceB, 2) + pow(screwAngleB, 2) * (pow(vertex_S[0], 2) + pow(vertex_S[1], 2))));
    }else if (dynamicsB.v_BN_N.norm() > 0)
    {
        thresholdList.push_back(maxError / (dynamicsB.v_BN_N.norm() * this->simTimeStep));
    }

    return *std::min_element(thresholdList.begin(), thresholdList.end());
}

double RigidBodyContactEffector::FindFaceIntervalThreshold(Eigen::Vector3d faceVertex0_B, Eigen::Vector3d faceVertex1_B, Eigen::Vector3d faceVertex2_B, dynamicData dynamicsA, Eigen::Vector3d supportVertex_B, dynamicData dynamicsB, double maxError)
{
    std::vector<double> thresholdList;
    Eigen::Vector3d vertexVelocity_N;
    
    
    vertexVelocity_N = (dynamicsA.dcm_NB * dynamicsA.omegaTilde_BN_B * faceVertex0_B) + dynamicsA.v_BN_N;
    thresholdList.push_back(maxError / vertexVelocity_N.norm());
    vertexVelocity_N = (dynamicsA.dcm_NB * dynamicsA.omegaTilde_BN_B * faceVertex1_B) + dynamicsA.v_BN_N;
    thresholdList.push_back(maxError / vertexVelocity_N.norm());
    vertexVelocity_N = (dynamicsA.dcm_NB * dynamicsA.omegaTilde_BN_B * faceVertex2_B) + dynamicsA.v_BN_N;
    thresholdList.push_back(maxError / vertexVelocity_N.norm());
    vertexVelocity_N = (dynamicsB.dcm_NB * dynamicsB.omegaTilde_BN_B * supportVertex_B) + dynamicsB.v_BN_N;
    thresholdList.push_back(maxError / vertexVelocity_N.norm());
    
    return *std::min_element(thresholdList.begin(), thresholdList.end());
}

double RigidBodyContactEffector::FindFaceIntervalThresholds(Eigen::Vector3d faceVertex0_B, Eigen::Vector3d faceVertex1_B, Eigen::Vector3d faceVertex2_B, dynamicData dynamicsA,  double screwAngleA, Eigen::Matrix3d screwRotA, Eigen::Vector3d screwOffsetA, double screwDistanceA, Eigen::Vector3d supportVertex_B, dynamicData dynamicsB,  double screwAngleB, Eigen::Matrix3d screwRotB, Eigen::Vector3d screwOffsetB, double screwDistanceB, double maxError)
{
    std::vector<double> thresholdList;
    Eigen::Vector3d vertex_S;

    if (screwAngleA > 0)
    {
        vertex_S = screwRotA * faceVertex0_B + screwOffsetA;
        thresholdList.push_back(maxError / sqrt( pow(screwDistanceA, 2) + pow(screwAngleA, 2) * (pow(vertex_S[0], 2) + pow(vertex_S[1], 2))));
        vertex_S = screwRotA * faceVertex1_B + screwOffsetA;
        thresholdList.push_back(maxError / sqrt( pow(screwDistanceA, 2) + pow(screwAngleA, 2) * (pow(vertex_S[0], 2) + pow(vertex_S[1], 2))));
        vertex_S = screwRotA * faceVertex2_B + screwOffsetA;
        thresholdList.push_back(maxError / sqrt( pow(screwDistanceA, 2) + pow(screwAngleA, 2) * (pow(vertex_S[0], 2) + pow(vertex_S[1], 2))));
    }else if (dynamicsA.v_BN_N.norm() > 0)
    {
        thresholdList.push_back(maxError / (dynamicsA.v_BN_N.norm() * this->simTimeStep));
    }
    
    if (screwAngleB > 0)
    {
        vertex_S = screwRotB * supportVertex_B + screwOffsetB;
        thresholdList.push_back(maxError / sqrt( pow(screwDistanceB, 2) + pow(screwAngleB, 2) * (pow(vertex_S[0], 2) + pow(vertex_S[1], 2))));
    }else if (dynamicsB.v_BN_N.norm() > 0)
    {
        thresholdList.push_back(maxError / (dynamicsB.v_BN_N.norm() * this->simTimeStep));
    }
    
    return *std::min_element(thresholdList.begin(), thresholdList.end());
}

std::vector<double> RigidBodyContactEffector::IntervalSine(double a, double b)
{
    std::vector<double> result;
    
    if  ((((3 * M_PI) / 2) >= std::min(a, b)) && (((3 * M_PI) / 2) <= std::max(a, b)))
    {
        result.push_back(-1.0);
    }else
    {
        result.push_back(std::min(sin(a),sin(b)));
    }
    
    if  (((( M_PI) / 2) >= std::min(a, b)) && (((M_PI) / 2) <= std::max(a, b)))
    {
        result.push_back(1.0);
    }else
    {
        result.push_back(std::max(sin(a),sin(b)));
    }
    
    return result;
}

std::vector<double> RigidBodyContactEffector::IntervalCosine(double a, double b)
{
    std::vector<double> result;
    
    if ((1 + (a / M_PI)) <= (b / M_PI))
    {
        result.push_back(-1.0);
        result.push_back(1.0);
    }else if (((a / M_PI) <= (b / M_PI)) && ((fmod((a / M_PI), 2)) == 1))
    {
        result.push_back(-1.0);
        result.push_back(std::max(cos(a), cos(b)));
    }else if (((a / M_PI) <= (b / M_PI)) && ((fmod((a / M_PI), 2)) == 0))
    {
        result.push_back(std::min(cos(a), cos(b)));
        result.push_back(1.0);
    }else
    {
        result.push_back(std::min(cos(a), cos(b)));
        result.push_back(std::max(cos(a), cos(b)));
    }
    
    return result;
}

std::vector<double> RigidBodyContactEffector::IntervalDotProduct(vectorInterval vectorA, vectorInterval vectorB)
{
    std::vector<double> component1;
    std::vector<double> component2;
    std::vector<double> component3;
    std::vector<double> result;
    
    component1.push_back(vectorA.lower[0] * vectorB.lower[0]);
    component1.push_back(vectorA.lower[0] * vectorB.upper[0]);
    component1.push_back(vectorA.upper[0] * vectorB.lower[0]);
    component1.push_back(vectorA.upper[0] * vectorB.upper[0]);
    
    component2.push_back(vectorA.lower[1] * vectorB.lower[1]);
    component2.push_back(vectorA.lower[1] * vectorB.upper[1]);
    component2.push_back(vectorA.upper[1] * vectorB.lower[1]);
    component2.push_back(vectorA.upper[1] * vectorB.upper[1]);
    
    component3.push_back(vectorA.lower[2] * vectorB.lower[2]);
    component3.push_back(vectorA.lower[2] * vectorB.upper[2]);
    component3.push_back(vectorA.upper[2] * vectorB.lower[2]);
    component3.push_back(vectorA.upper[2] * vectorB.upper[2]);
    
    result.push_back(*std::min_element(component1.begin(), component1.end()) + *std::min_element(component2.begin(), component2.end()) + *std::min_element(component3.begin(), component3.end()));
    
    result.push_back(*std::max_element(component1.begin(), component1.end()) + *std::max_element(component2.begin(), component2.end()) + *std::max_element(component3.begin(), component3.end()));
    
    return result;
}

vectorInterval RigidBodyContactEffector::IntervalCrossProduct(vectorInterval vectorA, vectorInterval vectorB)
{
    std::vector<double> a2b3;
    std::vector<double> a3b2;
    std::vector<double> a3b1;
    std::vector<double> a1b3;
    std::vector<double> a1b2;
    std::vector<double> a2b1;
    vectorInterval result;
    
    a2b3.push_back(vectorA.lower[1] * vectorB.lower[2]);
    a2b3.push_back(vectorA.lower[1] * vectorB.upper[2]);
    a2b3.push_back(vectorA.upper[1] * vectorB.lower[2]);
    a2b3.push_back(vectorA.upper[1] * vectorB.upper[2]);
    
    a3b2.push_back(vectorA.lower[2] * vectorB.lower[1]);
    a3b2.push_back(vectorA.lower[2] * vectorB.upper[1]);
    a3b2.push_back(vectorA.upper[2] * vectorB.lower[1]);
    a3b2.push_back(vectorA.upper[2] * vectorB.upper[1]);
    
    a3b1.push_back(vectorA.lower[2] * vectorB.lower[0]);
    a3b1.push_back(vectorA.lower[2] * vectorB.upper[0]);
    a3b1.push_back(vectorA.upper[2] * vectorB.lower[0]);
    a3b1.push_back(vectorA.upper[2] * vectorB.upper[0]);
    
    a1b3.push_back(vectorA.lower[0] * vectorB.lower[2]);
    a1b3.push_back(vectorA.lower[0] * vectorB.upper[2]);
    a1b3.push_back(vectorA.upper[0] * vectorB.lower[2]);
    a1b3.push_back(vectorA.upper[0] * vectorB.upper[2]);
    
    a1b2.push_back(vectorA.lower[0] * vectorB.lower[1]);
    a1b2.push_back(vectorA.lower[0] * vectorB.upper[1]);
    a1b2.push_back(vectorA.upper[0] * vectorB.lower[1]);
    a1b2.push_back(vectorA.upper[0] * vectorB.upper[1]);
    
    a2b1.push_back(vectorA.lower[1] * vectorB.lower[0]);
    a2b1.push_back(vectorA.lower[1] * vectorB.upper[0]);
    a2b1.push_back(vectorA.upper[1] * vectorB.lower[0]);
    a2b1.push_back(vectorA.upper[1] * vectorB.upper[0]);
    
    Eigen::Vector3d low((*std::min_element(a2b3.begin(), a2b3.end()) - *std::min_element(a3b2.begin(), a3b2.end())), (*std::min_element(a3b1.begin(), a3b1.end()) - *std::min_element(a1b3.begin(), a1b3.end())), (*std::min_element(a1b2.begin(), a1b2.end()) - *std::min_element(a2b1.begin(), a2b1.end())));
    result.lower = low;
    
    Eigen::Vector3d up((*std::max_element(a2b3.begin(), a2b3.end()) - *std::max_element(a3b2.begin(), a3b2.end())), (*std::max_element(a3b1.begin(), a3b1.end()) - *std::max_element(a1b3.begin(), a1b3.end())), (*std::max_element(a1b2.begin(), a1b2.end()) - *std::max_element(a2b1.begin(), a2b1.end())));
    result.upper = up;
    
    return result;
}

int RigidBodyContactEffector::LineLineDistance(Eigen::Vector3d vertex1, Eigen::Vector3d vertex2, Eigen::Vector3d vertex3, Eigen::Vector3d vertex4, Eigen::Vector3d *pointA, Eigen::Vector3d *pointB)
{
    Eigen::Vector3d line13, line23, line43, line21, line24, line14;
    double d1343, d4321, d1321, d4343, d2121;
    double numer, denom;
    double mua, mub;
    double dotRes;
    int retValue = 1;
    
    line13 = vertex1 - vertex3;
    line43 = vertex4 - vertex3;
    line21 = vertex2 - vertex1;
    
    d1343 = line13[0] * line43[0] + line13[1] * line43[1] + line13[2] * line43[2];
    d4321 = line43[0] * line21[0] + line43[1] * line21[1] + line43[2] * line21[2];
    d1321 = line13[0] * line21[0] + line13[1] * line21[1] + line13[2] * line21[2];
    d4343 = line43[0] * line43[0] + line43[1] * line43[1] + line43[2] * line43[2];
    d2121 = line21[0] * line21[0] + line21[1] * line21[1] + line21[2] * line21[2];
    
    denom = d2121 * d4343 - d4321 * d4321;
    if (denom < 0.001)
    {
        line23 = vertex2 - vertex3;
        dotRes = (line13.normalized()).dot(line23.normalized());
        if (dotRes < 0)
        {
            *pointA = vertex1 + ((vertex3 - vertex1).dot(line21) / line21.dot(line21)) * line21;
            *pointA = (*pointA + vertex2) / 2.0;
            *pointB = vertex3 + ((vertex2 - vertex3).dot(line43) / line43.dot(line43)) * line43;
            *pointB = (*pointB + vertex3) / 2.0;
            return 0;
        }
        line24 = vertex2 - vertex4;
        line14 = vertex1 - vertex4;
        dotRes = (line14.normalized()).dot(line24.normalized());
        if (dotRes < 0)
        {
            *pointA = vertex1 + ((vertex4 - vertex1).dot(line21) / line21.dot(line21)) * line21;
            *pointA = (*pointA + vertex1) / 2.0;
            *pointB = vertex3 + ((vertex1 - vertex3).dot(line43) / line43.dot(line43)) * line43;
            *pointB = (*pointB + vertex4) / 2.0;
            return 0;
        }
        dotRes = (line21.normalized()).dot(-line23.normalized());
        if (dotRes >= 0)
        {
            *pointA = vertex2;
            *pointB = vertex3;
            return 0;
        }
        dotRes = (line14.normalized()).dot(line21.normalized());
        if (dotRes >= 0)
        {
            *pointA = vertex1;
            *pointB = vertex4;
            return 0;
        }
        return -1;
    }
    numer = d1343 * d4321 - d1321 * d4343;
    mua = numer / denom;
    mub = (d1343 + d4321 * mua) / d4343;
    
    if (mua < 0.0)
    {
        *pointA = vertex1;
        retValue = -1;
    }else if (mua > 1.0)
    {
        *pointA = vertex2;
        retValue = -1;
    }else
    {
        *pointA = vertex1 + mua * line21;
    }
    
    if (mub < 0.0)
    {
        *pointB = vertex3;
        retValue = -1;
    }else if (mub > 1.0)
    {
        *pointB = vertex4;
        retValue = -1;
    }else
    {
        *pointB = vertex3 + mub * line43;
    }
    
    return retValue;
}

int RigidBodyContactEffector::PointInTriangle(Eigen::Vector3d supportPoint, Eigen::Vector3d triVertex0, Eigen::Vector3d triVertex1, Eigen::Vector3d triVertex2, Eigen::Vector3d *contactPoint)
{
    Eigen::Vector3d u01 = triVertex1 - triVertex0;
    Eigen::Vector3d u02 = triVertex2 - triVertex0;
    Eigen::Vector3d u12 = triVertex2 - triVertex1;
    Eigen::Vector3d n = u01.cross(u02).normalized();
    Eigen::Vector3d w = supportPoint - triVertex0;
    double alpha = w.dot(n);
    Eigen::Vector3d vecToPlane = -alpha * n;
    
    *contactPoint = supportPoint + vecToPlane;
    
    Eigen::Vector3d V1 = -u01.normalized() - u02.normalized();
    Eigen::Vector3d V2 = -u12.normalized() + u01.normalized();
    Eigen::Vector3d V3 = u02.normalized() + u12.normalized();
    double f1 = (V1.cross(*contactPoint - triVertex0)).dot(n);
    double f2 = (V2.cross(*contactPoint - triVertex1)).dot(n);
    double f3 = (V3.cross(*contactPoint - triVertex2)).dot(n);
    
    if ( (f2 <= 0) && (f1 > 0))
    {
        if ( ((triVertex0 - *contactPoint).cross(triVertex1 - *contactPoint)).dot(n) >= -1e-9 )
        {
            return 1;
        }else{
            return -1;
        }
    }
    if ( (f3 <= 0) && (f2 > 0))
    {
        if ( ((triVertex1 - *contactPoint).cross(triVertex2 - *contactPoint)).dot(n) >= -1e-9 )
        {
            return 1;
        }else
        {
            return -1;
        }
    }
    if ( (f1 <= 0) && (f3 > 0))
    {
        if ( ((triVertex2 - *contactPoint).cross(triVertex0 - *contactPoint)).dot(n) >= -1e-9 )
        {
            return 1;
        }else
        {
            return -1;
        }
    }
    return -1;
}

double RigidBodyContactEffector::WhenEdgeContact(std::vector<std::vector<int>> edgePair, std::vector<Eigen::Vector3d> verticesA, std::vector<Eigen::Vector3d> verticesB, dynamicData dynamicsA, dynamicData dynamicsB, double dt, Eigen::Vector3d *edgeAContact_N, Eigen::Vector3d *edgeBContact_N, int *coLin)
{
    vectorInterval edgeIntervalA;
    vectorInterval edgeIntervalB;
    vectorInterval edgeIntervalMixed;
    vectorInterval vertexIntervalA0;
    vectorInterval vertexIntervalA1;
    vectorInterval vertexIntervalB0;
    vectorInterval vertexIntervalB1;
    Eigen::Vector3d edgeVertexA0 = verticesA[edgePair[0][0]];
    Eigen::Vector3d edgeVertexA1 = verticesA[edgePair[0][1]];
    Eigen::Vector3d edgeVertexB0 = verticesB[edgePair[1][0]];
    Eigen::Vector3d edgeVertexB1 = verticesB[edgePair[1][1]];
    std::vector<double> contactInterval;
    double intervalThreshold;
    std::vector<double> timeInterval;
    std::vector<double> timeIntervalChain;
    std::vector<Eigen::Vector3d> upperIntervalChain;
    vectorInterval tempIntervalA;
    vectorInterval tempIntervalB;
    vectorInterval tempIntervalMixed;
    int timeSearch = 2;
    Eigen::Vector3d potentialContactA0;
    Eigen::Vector3d potentialContactA1;
    Eigen::Vector3d potentialContactB0;
    Eigen::Vector3d potentialContactB1;
    Eigen::Vector3d edgeConnect;
    Eigen::Vector3d colinCheck1;
    Eigen::Vector3d colinCheck2;
    double screwAngleA;
    Eigen::Matrix3d screwRotA;
    Eigen::Vector3d screwOffsetA;
    double screwDistanceA;
    double screwAngleB;
    Eigen::Matrix3d screwRotB;
    Eigen::Vector3d screwOffsetB;
    double screwDistanceB;
    
    this->C2Screw(dynamicsA.dcm_BprimeB, dynamicsA.dcm_BN * (dynamicsA.v_BN_N * this->simTimeStep), &screwAngleA, &screwRotA, &screwOffsetA, &screwDistanceA);
    this->C2Screw(dynamicsB.dcm_BprimeB, dynamicsB.dcm_BN * (dynamicsB.v_BN_N * this->simTimeStep), &screwAngleB, &screwRotB, &screwOffsetB, &screwDistanceB);
    
    timeInterval.push_back(0.0);
    timeInterval.push_back(1.0);
    
    vertexIntervalA0 = MakeIntervalValues(edgeVertexA0, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, timeInterval);
    vertexIntervalA1 = MakeIntervalValues(edgeVertexA1, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, timeInterval);
    vertexIntervalB0 = MakeIntervalValues(edgeVertexB0, dynamicsB, screwAngleB, screwRotB, screwOffsetB, screwDistanceB, timeInterval);
    vertexIntervalB1 = MakeIntervalValues(edgeVertexB1, dynamicsB, screwAngleB, screwRotB, screwOffsetB, screwDistanceB, timeInterval);
    
    edgeIntervalA.lower = vertexIntervalA1.lower - vertexIntervalA0.lower;
    edgeIntervalA.upper = vertexIntervalA1.upper - vertexIntervalA0.upper;
    edgeIntervalB.lower = vertexIntervalB1.lower - vertexIntervalB0.lower;
    edgeIntervalB.upper = vertexIntervalB1.upper - vertexIntervalB0.upper;
    edgeIntervalMixed.lower = vertexIntervalB0.lower - vertexIntervalA0.lower;
    edgeIntervalMixed.upper = vertexIntervalB0.upper - vertexIntervalA0.upper;
    
    contactInterval = this->IntervalDotProduct(edgeIntervalMixed,this->IntervalCrossProduct(edgeIntervalA, edgeIntervalB));
    
    if (contactInterval[0] > 0.0 || contactInterval[1] < 0.0 )
    {
        return -1;
    } else if ((contactInterval[0] > -0.0001f) && (contactInterval[1] < 0.0001f))
    {
        *coLin = this->LineLineDistance(vertexIntervalA0.lower, vertexIntervalA1.lower, vertexIntervalB0.lower, vertexIntervalB1.lower, edgeAContact_N, edgeBContact_N);
        if (*coLin == -1)
        {
            return -1;
        }
        colinCheck1 = *edgeBContact_N - *edgeAContact_N;
        *coLin = this->LineLineDistance(vertexIntervalA0.upper, vertexIntervalA1.upper, vertexIntervalB0.upper, vertexIntervalB1.upper, edgeAContact_N, edgeBContact_N);
        if (*coLin == -1)
        {
            return -1;
        }
        colinCheck2 = *edgeBContact_N - *edgeAContact_N;
        if (colinCheck1.dot(colinCheck2) > 0){
            return -1;
        }
    }

        intervalThreshold = this->FindEdgeIntervalThresholds(edgeVertexA0, edgeVertexA1, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, edgeVertexB0, edgeVertexB1, dynamicsB, screwAngleB, screwRotB, screwOffsetB, screwDistanceB, this->maxPosError / 100.0f);

        while (timeSearch > 0)
        {
            timeIntervalChain.push_back(timeInterval[1]);
            timeInterval[1] = 0.5f * (timeInterval[0] + timeInterval[1]);
            vertexIntervalA0 = MakeIntervalValues(edgeVertexA0, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, timeInterval);
            vertexIntervalA1 = MakeIntervalValues(edgeVertexA1, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, timeInterval);
            vertexIntervalB0 = MakeIntervalValues(edgeVertexB0, dynamicsB, screwAngleB, screwRotB, screwOffsetB, screwDistanceB, timeInterval);
            vertexIntervalB1 = MakeIntervalValues(edgeVertexB1, dynamicsB, screwAngleB, screwRotB, screwOffsetB, screwDistanceB, timeInterval);
            
            edgeIntervalA.lower = vertexIntervalA1.lower - vertexIntervalA0.lower;
            edgeIntervalA.upper = vertexIntervalA1.upper - vertexIntervalA0.upper;
            edgeIntervalB.lower = vertexIntervalB1.lower - vertexIntervalB0.lower;
            edgeIntervalB.upper = vertexIntervalB1.upper - vertexIntervalB0.upper;
            edgeIntervalMixed.lower = vertexIntervalB0.lower - vertexIntervalA0.lower;
            edgeIntervalMixed.upper = vertexIntervalB0.upper - vertexIntervalA0.upper;
            
            contactInterval = this->IntervalDotProduct(edgeIntervalMixed,this->IntervalCrossProduct(edgeIntervalA, edgeIntervalB));
            
            if (contactInterval[0] > 0.0f || contactInterval[1] < 0.0f)
            {
                timeInterval[0] = timeInterval[1];
                timeInterval[1] = timeIntervalChain.back();
                timeIntervalChain.pop_back();
                if ((timeInterval[1] - timeInterval[0]) * dt <= intervalThreshold)
                {
                    if (timeIntervalChain.size() == 0)
                    {
                        timeSearch -= 1;
                    }else
                    {
                        timeInterval[1] = timeIntervalChain.back();
                        timeIntervalChain.pop_back();
                    }
                    
                }
                continue;
            }
            
            if ((timeInterval[1] - timeInterval[0]) * dt <= intervalThreshold)
            {
                *coLin = this->LineLineDistance(vertexIntervalA0.lower, vertexIntervalA1.lower, vertexIntervalB0.lower, vertexIntervalB1.lower, edgeAContact_N, edgeBContact_N);
                
                if (*coLin >= 0)
                {
                    edgeConnect = *edgeBContact_N - *edgeAContact_N;
                    if ( edgeConnect.norm() <= this->maxPosError)
                    {
                        return dt * (timeInterval[1] + timeInterval[0]) / 2.0f;
                    }
                }
                timeInterval[0] = timeInterval[1];
                timeInterval[1] = timeIntervalChain.back();
                timeIntervalChain.pop_back();
                if (timeIntervalChain.size() == 0)
                {
                    timeSearch -= 1;
                }else
                {
                    timeInterval[1] = timeIntervalChain.back();
                    timeIntervalChain.pop_back();
                }
            }
        }
    
    return -1;
}


double RigidBodyContactEffector::WhenFaceContact(std::vector<int> trianglePoints, std::vector<Eigen::Vector3d> verticesA, int supportPoint, std::vector<Eigen::Vector3d> verticesB, dynamicData dynamicsA, dynamicData dynamicsB, double dt, Eigen::Vector3d *faceContactPoint_N)
{
    vectorInterval supportInterval;
    vectorInterval faceLegInterval1;
    vectorInterval faceLegInterval2;
    vectorInterval vertexIntervalA0;
    vectorInterval vertexIntervalA1;
    vectorInterval vertexIntervalA2;
    vectorInterval vertexIntervalB0;
    Eigen::Vector3d faceVertex0 = verticesA[trianglePoints[0]];
    Eigen::Vector3d faceVertex1 = verticesA[trianglePoints[1]];
    Eigen::Vector3d faceVertex2 = verticesA[trianglePoints[2]];
    Eigen::Vector3d supportVertex = verticesB[supportPoint];
    std::vector<double> contactInterval;
    double intervalThreshold;
    std::vector<double> timeInterval;
    std::vector<double> timeIntervalChain;
    std::vector<Eigen::Vector3d> upperIntervalChain;
    vectorInterval tempLegInterval1;
    vectorInterval tempLegInterval2;
    vectorInterval tempSupportInterval;
    int timeSearch = 2;
    Eigen::Vector3d potentialSupport;
    Eigen::Vector3d potentialVertex0;
    Eigen::Vector3d potentialVertex1;
    Eigen::Vector3d potentialVertex2;
    double screwAngleA;
    Eigen::Matrix3d screwRotA;
    Eigen::Vector3d screwOffsetA;
    double screwDistanceA;
    double screwAngleB;
    Eigen::Matrix3d screwRotB;
    Eigen::Vector3d screwOffsetB;
    double screwDistanceB;
    
    this->C2Screw(dynamicsA.dcm_BprimeB, dynamicsA.dcm_BN * (dynamicsA.v_BN_N * this->simTimeStep), &screwAngleA, &screwRotA, &screwOffsetA, &screwDistanceA);
    this->C2Screw(dynamicsB.dcm_BprimeB, dynamicsB.dcm_BN * (dynamicsB.v_BN_N * this->simTimeStep), &screwAngleB, &screwRotB, &screwOffsetB, &screwDistanceB);
    
    timeInterval.push_back(0.0);
    timeInterval.push_back(1.0);
    
    vertexIntervalA0 = MakeIntervalValues(faceVertex0, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, timeInterval);
    vertexIntervalA1 = MakeIntervalValues(faceVertex1, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, timeInterval);
    vertexIntervalA2 = MakeIntervalValues(faceVertex2, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, timeInterval);
    vertexIntervalB0 = MakeIntervalValues(supportVertex, dynamicsB, screwAngleB, screwRotB, screwOffsetB, screwDistanceB, timeInterval);
    
    faceLegInterval1.lower = vertexIntervalA0.lower - vertexIntervalA1.lower;
    faceLegInterval1.upper = vertexIntervalA0.upper - vertexIntervalA1.upper;
    faceLegInterval2.lower = vertexIntervalA0.lower - vertexIntervalA2.lower;
    faceLegInterval2.upper = vertexIntervalA0.upper - vertexIntervalA2.upper;
    supportInterval.lower = vertexIntervalB0.lower - vertexIntervalA0.lower;
    supportInterval.upper = vertexIntervalB0.upper - vertexIntervalA0.upper;
    
    contactInterval = this->IntervalDotProduct(supportInterval,this->IntervalCrossProduct(faceLegInterval1, faceLegInterval2));
    
    if (contactInterval[0] > 0.0000001f || contactInterval[1] < 0.0000001f)
    {
        return -1;
    } else
    {
        intervalThreshold = this->FindFaceIntervalThresholds( faceVertex0, faceVertex1, faceVertex2, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, supportVertex, dynamicsB,  screwAngleB, screwRotB, screwOffsetB, screwDistanceB, this->maxPosError / 100.0f);
        while (timeSearch > 0)
        {
            timeIntervalChain.push_back(timeInterval[1]);
            timeInterval[1] = 0.5f * (timeInterval[0] + timeInterval[1]);
            vertexIntervalA0 = MakeIntervalValues(faceVertex0, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, timeInterval);
            vertexIntervalA1 = MakeIntervalValues(faceVertex1, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, timeInterval);
            vertexIntervalA2 = MakeIntervalValues(faceVertex2, dynamicsA, screwAngleA, screwRotA, screwOffsetA, screwDistanceA, timeInterval);
            vertexIntervalB0 = MakeIntervalValues(supportVertex, dynamicsB, screwAngleB, screwRotB, screwOffsetB, screwDistanceB, timeInterval);
            
            faceLegInterval1.lower = vertexIntervalA0.lower - vertexIntervalA1.lower;
            faceLegInterval1.upper = vertexIntervalA0.upper - vertexIntervalA1.upper;
            faceLegInterval2.lower = vertexIntervalA0.lower - vertexIntervalA2.lower;
            faceLegInterval2.upper = vertexIntervalA0.upper - vertexIntervalA2.upper;
            supportInterval.lower = vertexIntervalB0.lower - vertexIntervalA0.lower;
            supportInterval.upper = vertexIntervalB0.upper - vertexIntervalA0.upper;
            
            contactInterval = this->IntervalDotProduct(supportInterval,this->IntervalCrossProduct(faceLegInterval1, faceLegInterval2));
            
            if (contactInterval[0] > 0.0f || contactInterval[1] < 0.0f)
            {
                timeInterval[0] = timeInterval[1];
                timeInterval[1] = timeIntervalChain.back();
                timeIntervalChain.pop_back();
                if ((timeInterval[1] - timeInterval[0])  <= intervalThreshold)
                {
                    if (timeIntervalChain.size() == 0)
                    {
                        timeSearch -= 1;
                    }else
                    {
                        timeInterval[1] = timeIntervalChain.back();
                        timeIntervalChain.pop_back();
                    }
                    
                }
                continue;
            }
            
            if ((timeInterval[1] - timeInterval[0])  <= intervalThreshold)
            {
                if (this->PointInTriangle(vertexIntervalB0.lower, vertexIntervalA0.lower, vertexIntervalA1.lower, vertexIntervalA2.lower, faceContactPoint_N) == 1)
                {
                    if ( (*faceContactPoint_N - vertexIntervalB0.lower).norm() <= this->maxPosError)
                    {
                        return dt * (timeInterval[1] + timeInterval[0]) / 2.0f;
                    }
                }
                
                timeInterval[0] = timeInterval[1];
                timeInterval[1] = timeIntervalChain.back();
                timeIntervalChain.pop_back();
                if (timeIntervalChain.size() == 0)
                {
                    timeSearch -= 1;
                }else
                {
                    timeInterval[1] = timeIntervalChain.back();
                    timeIntervalChain.pop_back();
                }
            }
        }
    }
    return -1;
}
    
Eigen::Vector3d RigidBodyContactEffector::CalcImpluse(contactDetail collisionData, dynamicData otherDynamics, double coefRestitution)
{
    Eigen::MatrixXd invIMainPntB_N = this->mainBody.states.dcm_NB *  this->mainBody.states.ISCPntB_B.inverse() * this->mainBody.states.dcm_BN;
////    Eigen::MatrixXd invIOtherPntB_N = otherDynamics.dcm_NB *  otherDynamics.ISCPntB_B.inverse() * otherDynamics.dcm_BN;
//
    Eigen::Vector3d contactPosMain_N = this->mainBody.states.dcm_NB * (collisionData.mainContactPoint - this->mainBody.states.c_B);
    Eigen::Vector3d contactPosOther_N = otherDynamics.dcm_NB * (collisionData.otherContactPoint);// - otherDynamics.c_B);
//
    Eigen::Vector3d contactVelMain_N = this->mainBody.states.v_BN_N + eigenTilde(this->mainBody.states.dcm_NB * this->mainBody.states.omega_BN_B) * contactPosMain_N;
    Eigen::Vector3d contactVelOther_N = otherDynamics.v_BN_N + (otherDynamics.dcm_NB * otherDynamics.omegaTilde_BN_B * otherDynamics.dcm_BN) * contactPosOther_N;
//
    double v_rel_N = collisionData.contactNormal.dot(contactVelMain_N - contactVelOther_N);
    if (v_rel_N < 0.0)
    {
        return 0.0 * collisionData.contactNormal;
    }
//
//    Eigen::Vector3d interDenom1 = eigenTilde(contactPosMain_N) * collisionData.contactNormal;
//
////    Eigen::Vector3d interDenom2 = invIOtherPntB_N * eigenTilde(contactPosOther_N) * collisionData.contactNormal;
//
////    double j = (-(1 + coefRestitution) * v_rel_N) / ( 1/this->mainBody.states.m_SC + 1/otherDynamics.m_SC + collisionData.contactNormal.dot(eigenTilde(interDenom1) * contactPosMain_N) + collisionData.contactNormal.dot(eigenTilde(interDenom2) * contactPosOther_N));
//
//    double j = (-(1.0 + coefRestitution) * v_rel_N) / ( (1.0 / this->mainBody.states.m_SC) + collisionData.contactNormal.dot(invIMainPntB_N * eigenTilde(interDenom1) * contactPosMain_N));
    
    double j = (((1.0 / this->mainBody.states.m_SC ) * Eigen::Matrix3d::Identity() - eigenTilde(this->mainBody.states.dcm_NB * collisionData.mainContactPoint) * invIMainPntB_N * eigenTilde(this->mainBody.states.dcm_NB * collisionData.mainContactPoint)).inverse() * (eigenTilde(this->mainBody.states.dcm_NB * collisionData.mainContactPoint) * (this->mainBody.states.dcm_NB * this->mainBody.states.omega_BN_B) - this->mainBody.states.v_BN_N + contactVelOther_N)).norm();
    
    return (j + coefRestitution * j) * -collisionData.contactNormal;
}
    
void RigidBodyContactEffector::C2Screw(Eigen::Matrix3d DCM, Eigen::Vector3d displacement, double *screwAngle, Eigen::Matrix3d *screwRot, Eigen::Vector3d *screwOffset, double *screwDistance)
{
    Eigen::Matrix3d screwFrame;
    double C[3][3];
    double Q[4];
    Eigen::Vector3d PRV;
    eigenMatrix3d2CArray(DCM, *C);
    Q[0] = acos(0.5 * ( C[0][0] + C[1][1] + C[2][2] - 1));
    Q[1] = (1 / (2 * sin(Q[0]))) * (C[1][2] - C[2][1]);
    Q[2] = (1 / (2 * sin(Q[0]))) * (C[2][0] - C[0][2]);
    Q[3] = (1 / (2 * sin(Q[0]))) * (C[0][1] - C[1][0]);
    *screwAngle = Q[0];
    PRV[0] = Q[1];
    PRV[1] = Q[2];
    PRV[2] = Q[3];
    PRV.normalize();
    
    Eigen::Vector3d perpD = displacement - displacement.dot(PRV) * PRV;
    
    if ( (round(perpD[0] * 1000.0) == 0.0) && (round(perpD[1] * 1000.0) == 0.0) && (round(perpD[2] * 1000.0) == 0.0))
    {
        Eigen::Vector3d newPerp;
        if (abs(PRV[0]) > abs(PRV[1]))
        {
            newPerp[0] = PRV[2];
            newPerp[1] = 0.0;
            newPerp[2] = -PRV[0];
        }else
        {
            newPerp[0] = 0.0;
            newPerp[1] = PRV[2];
            newPerp[2] = -PRV[1];
        }
        Eigen::Vector3d w = PRV.cross(newPerp);
        screwFrame << newPerp, w, PRV;
        *screwRot = screwFrame.transpose();
        *screwOffset << 0.0, 0.0, 0.0;
        *screwDistance = PRV.dot(displacement);
        return;
    }
    
    Eigen::Vector3d v = perpD.normalized();
    Eigen::Vector3d w = PRV.cross(v);
    screwFrame << v, w, PRV;
    *screwRot = screwFrame.transpose();
    
    *screwOffset = (perpD.norm() / 2) * ( v + (sin(*screwAngle) / (1 - cos(*screwAngle))) * w);
    *screwOffset = - *screwRot * *screwOffset;
    *screwDistance = PRV.dot(displacement);
    return;
}

Eigen::Vector3d RigidBodyContactEffector::SecondTop(std::stack<Eigen::Vector3d> &stk)
{
   Eigen::Vector3d tempPoint = stk.top();
   stk.pop();
   Eigen::Vector3d result = stk.top();    //get the second top element
   stk.push(tempPoint);      //push previous top again
   return result;
}


bool RigidBodyContactEffector::ComparePoints(const std::vector<Eigen::Vector3d> &point1, const std::vector<Eigen::Vector3d> &point2)
{
    Eigen::Vector3d p1 = point1[0];
    Eigen::Vector3d p2 = point2[0];
     Eigen::Vector3d p0 = point1[1];
    double val = (p1[1] - p0[1]) * (p2[0] - p1[0]) - (p1[0] - p0[0]) * (p2[1] - p1[1]);
    int dir;
    if (abs(val) < 0.01)
    {
       dir = 0;    //colinear
    }else if(val < 0)
    {
       dir = 2;    //anti-clockwise direction
    }else
    {
       dir = 1;    //clockwise direction
    }
   if(dir == 0)
      return ((p0 - p2).squaredNorm() > (p0 - p1).squaredNorm()) ? true : false;
   return (dir==2) ? true : false;
}


std::vector<Eigen::Vector3d> RigidBodyContactEffector::findConvexHull(std::vector<Eigen::Vector3d> points)
{
   std::vector<Eigen::Vector3d> convexHullPoints;
    std::vector<std::vector<Eigen::Vector3d>> pointsWithMin;
    int n = points.size();
   int minY = points[0][1], min = 0;
   for(int i = 1; i<n; i++)
   {
      int y = points[i][1];
      //find bottom most or left most point
      if(((y < minY) || (minY == y)) && (points[i][0] < points[min][0]))
      {
         minY = points[i][1];
         min = i;
      }
   }
   std::swap(points[0], points[min]);    //swap min point to 0th location
   Eigen::Vector3d p0 = points[0];
    for (int minAsId = 1; minAsId < points.size(); ++minAsId)
    {
        convexHullPoints.push_back(points[minAsId]);
        convexHullPoints.push_back(p0);
        pointsWithMin.push_back(convexHullPoints);
        convexHullPoints.clear();
    }
    std::sort(pointsWithMin.begin(), pointsWithMin.end(), [this](const std::vector<Eigen::Vector3d> &point1, const std::vector<Eigen::Vector3d> &point2){
        return this->ComparePoints(point1, point2);
    });
   int arrSize = 1;    //used to locate items in modified array
   double dir;
    points.clear();
    points.push_back(p0);
   for(int i = 0; i<n-2; i++)
   {
      //when the angle of ith and (i+1)th elements are same, remove points
       dir = (pointsWithMin[i][0][1] - p0[1]) * (pointsWithMin[i+1][0][0] - pointsWithMin[i][0][0]) - (pointsWithMin[i][0][0] - p0[0]) * (pointsWithMin[i+1][0][1] - pointsWithMin[i][0][1]);
       
       while((i < n-3) && (abs(dir) < 0.01))
       {
         i++;
         dir = (pointsWithMin[i][0][1] - p0[1]) * (pointsWithMin[i+1][0][0] - pointsWithMin[i][0][0]) - (pointsWithMin[i][0][0] - p0[0]) * (pointsWithMin[i+1][0][1] - pointsWithMin[i][0][1]);
       }
         points[arrSize] = pointsWithMin[i][0];
         arrSize++;
   }
    if(arrSize < 3)
    {
      return convexHullPoints;    //there must be at least 3 points, return empty list.
    }
      //create a stack and add first three points in the stack
    std::stack<Eigen::Vector3d> stk;
    stk.push(points[0]); stk.push(points[1]); stk.push(points[2]);
    double val;
   for(int i = 3; i<arrSize; i++)
   {    //for remaining vertices
       val = (stk.top()[1] - this->SecondTop(stk)[1]) * (points[i][0] - stk.top()[0]) - (stk.top()[0] - this->SecondTop(stk)[0]) * (points[i][1] - stk.top()[1]);
       if (abs(val) < 0.01)
       {
          dir = 0;    //colinear
       }else if(val < 0)
       {
          dir = 2;    //anti-clockwise direction
       }else
       {
          dir = 1;    //clockwise direction
       }
       if(dir != 2)
       {
         stk.pop();    //when top, second top and ith point are not making left turn, remove point
       }
       stk.push(points[i]);
   }
   while(!stk.empty())
   {
      convexHullPoints.push_back(stk.top());    //add points from stack
      stk.pop();
   }
    return convexHullPoints;
}

