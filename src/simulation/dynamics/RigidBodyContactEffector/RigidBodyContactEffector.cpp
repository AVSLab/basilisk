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
#include <stdlib.h>




/*! This is the constructor, setting variables to default values */
RigidBodyContactEffector::RigidBodyContactEffector()
{
    this->isOverlap = false;
    this->mainBody.boundingRadius = 0.0;
    this->mainBody.collisionPoints.clear();
    this->currentBodyInCycle = 0;
    this->numBodies = 0;
    this->boundingBoxFF = 1.0;
    this->minBoundingBoxDim = 0.005;
    this->maxTimeStep = 0.001;
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
    this->currentBodyInCycle = 0;
    
}

/*! This method uses TinyOBJLoader to load the primary body primitives.
@return void
@param objFile The .obj file associated with the primary body
 */
void RigidBodyContactEffector::LoadSpacecraftBody(const char *objFile, std::string modelTag, Message<SCStatesMsgPayload> *scStateMsg, Message<SCMassPropsMsgPayload> *scMassStateMsg, double boundingRadius, double coefRestitution, double coefFriction)
{
    geometry body;
    body.boundingRadius = boundingRadius;
    body.coefRestitution = coefRestitution;
    body.coefFriction = coefFriction;
    body.modelTag = modelTag;
    body.scStateInMsg = scStateMsg->addSubscriber();
    body.scMassStateInMsg = scMassStateMsg->addSubscriber();
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
    body.planetInMsg.subscribeTo(planetSpiceMsg);
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
    double minX;
    double minY;
    double minZ;
    
    std::vector<std::vector<int>> edgeIndices;
    std::vector<int> faceIndices;
    std::vector<int> shapeIndices;
    std::set<int> totalSet;
    
    for (int shapeIt=0; shapeIt<shapes.size(); shapeIt++)
    {
        indexOffset = 0;
        for (int faceIt=0; faceIt<shapes[shapeIt].mesh.num_face_vertices.size(); faceIt++)
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
            for (int ii=0; ii < 3; ii++)
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
            v2 << maxX, maxY, maxZ;
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
    
    for ( int ii=0; ii<allConnections.size(); ii++)
    {
        if (std::find(allConnections[ii].begin(), allConnections[ii].end(), -1) != allConnections[ii].end())
        {
            for ( int jj=0; jj<unconnectedFaces.size(); jj++)
            {
                if (unconnectedFaces[jj] == ii)
                {
                    continue;
                }
                for ( int kk=0; kk<3; kk++)
                {
                    for ( int gg=0; gg<3; gg++)
                    {
                        for ( int ww=2; ww>=0; ww--)
                        {
                            for ( int pp=2; pp>=0; pp--)
                            {
                                if ((ww != kk) && (gg != pp) && (allFaces[ii][kk] == allFaces[unconnectedFaces[jj]][gg]) && (allFaces[ii][ww] == allFaces[unconnectedFaces[jj]][pp]))
                                {
                                    *std::find(allConnections[ii].begin(), allConnections[ii].end(), -1) = unconnectedFaces[jj];
                                    *std::find(allConnections[unconnectedFaces[jj]].begin(), allConnections[unconnectedFaces[jj]].end(), -1) = ii;
                                    goto endloop;
                                }
                            }
                        }
                    }
                }
                endloop:
                if (std::find(allConnections[ii].begin(), allConnections[ii].end(), -1) == allConnections[ii].end())
                {
                    break;
                }
            }
            
            
            for ( int jj=0; jj<3; jj++)
            {
                if (std::find(allConnections.at(allConnections[ii][jj]).begin(), allConnections.at(allConnections[ii][jj]).end(), -1) == allConnections.at(allConnections[ii][jj]).end())
                {
                    unconnectedFaces.erase(std::remove(unconnectedFaces.begin(), unconnectedFaces.end(), allConnections[ii][jj]), unconnectedFaces.end());
                }
            }
            unconnectedFaces.erase(std::remove(unconnectedFaces.begin(), unconnectedFaces.end(), ii), unconnectedFaces.end());
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
        boundingGroup.faceCentroids.clear();
        boundingGroup.faceBoundingBoxes.clear();
        facesInGroup.clear();
        verticesInGroup.clear();
        
        boundingGroup.faceTriangles.push_back(allFaces[ungroupedFaces[0]]);
        boundingGroup.faceNormals.push_back(allNormals[ungroupedFaces[0]]);
        boundingGroup.faceCentroids.push_back(allCentroids[ungroupedFaces[0]]);
        boundingGroup.faceBoundingBoxes.push_back(allBoundingBoxes[ungroupedFaces[0]]);
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
                indivMaxDists.clear();
                for (int jj=0; jj < 3; ++jj)
                {
                    
                    for (int kk=0; kk < verticesInGroup.size(); ++kk)
                    {
                        indivMaxDists.push_back((verticesInGroup[kk] - vertices[allFaces[adjacentFacesToGroup[ii]][jj]]).norm());
                    }
                }
                if (*std::max_element(indivMaxDists.begin(), indivMaxDists.end()) > adjacentDistsToGroup[ii])
                {
                    adjacentDistsToGroup[ii] = *std::max_element(indivMaxDists.begin(), indivMaxDists.end());
                }
                
            }
            
            if (*std::min_element(adjacentDistsToGroup.begin(), adjacentDistsToGroup.end()) >= this->maxBoundingBoxDim)
            {
                searchingForCloseFace = false;
                continue;
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
            ungroupedFaces.erase(std::remove(ungroupedFaces.begin(), ungroupedFaces.end(), adjacentFacesToGroup[faceCounter]), ungroupedFaces.end());
            
            if (ungroupedFaces.empty())
            {
                searchingForCloseFace = false;
            }
//            indivMaxDists.clear();
//            for (int ii=0; ii < verticesInGroup.size()-1; ++ii)
//            {
//                for (int jj=ii+1; jj < verticesInGroup.size(); ++jj)
//                {
//                    indivMaxDists.push_back((verticesInGroup[ii] - verticesInGroup[jj]).norm());
//                }
//            }
//            if (*std::max_element(indivMaxDists.begin(), indivMaxDists.end()) >= this->maxBoundingBoxDim)
//            {
//                searchingForCloseFace = false;
//            }
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
        if (convHullPoints.size() == 0)
        {
            convHullPoints = verticesInGroup;
        }
        centroid << 0.0, 0.0, 0.0;
        
//        for (int ii=0; ii < convHullPoints.size(); ++ii)
//        {
//            centroid += convHullPoints[ii];
//            std::cout << convHullPoints[ii] << std::endl;
//        }
//        centroid = centroid / convHullPoints.size();
        
        maxX = -1E15;
        maxY = -1E15;
        maxZ = -1E15;
        minX = 1E15;
        minY = 1E15;
        minZ = 1E15;
        for (int ii=0; ii < convHullPoints.size(); ++ii)
        {
            std::cout << convHullPoints[ii][0] << " " << convHullPoints[ii][1] << " " << convHullPoints[ii][2] << std::endl;
            if (convHullPoints[ii][0] > maxX)
            {
                maxX = convHullPoints[ii][0];
            }
            if (convHullPoints[ii][1] > maxY)
            {
                maxY = convHullPoints[ii][1];
            }
            if (convHullPoints[ii][2] > maxZ)
            {
                maxZ = convHullPoints[ii][2];
            }
            if (convHullPoints[ii][0] < minX)
            {
                minX = convHullPoints[ii][0];
            }
            if (convHullPoints[ii][1] < minY)
            {
                minY = convHullPoints[ii][1];
            }
            if (convHullPoints[ii][2] < minZ)
            {
                minZ = convHullPoints[ii][2];
            }
        }
        centroid << (maxX + minX)/2.0, (maxY + minY)/2.0, (maxZ + minZ)/2.0;
        
        std::cout << std::endl << centroid << std::endl;
        maxX = this->minBoundingBoxDim;
        maxY = this->minBoundingBoxDim;
        maxZ = this->minBoundingBoxDim;
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
        std::cout << maxX << " " << maxY << " " << maxZ << std::endl << std::endl;
        
        
        for (int faceIt=0; faceIt<polyhedron[shapeIt].faceTriangles.size(); ++faceIt)
        {
            
            for ( int inx=0; inx<2; ++inx)
            {
                polyhedron[shapeIt].uniqueVertIndices.push_back(polyhedron[shapeIt].faceTriangles[faceIt][inx]);
                std::vector<int> edgeGroup = {polyhedron[shapeIt].faceTriangles[faceIt][inx], polyhedron[shapeIt].faceTriangles[faceIt][inx+1]};
                edgeIndices.push_back(edgeGroup);
                faceIndices.push_back(faceIt);
                shapeIndices.push_back(shapeIt);
            }
            polyhedron[shapeIt].uniqueVertIndices.push_back(polyhedron[shapeIt].faceTriangles[faceIt][2]);
            std::vector<int> edgeGroup = { polyhedron[shapeIt].faceTriangles[faceIt][2], polyhedron[shapeIt].faceTriangles[faceIt][0]};
            edgeIndices.push_back(edgeGroup);
            faceIndices.push_back(faceIt);
            shapeIndices.push_back(shapeIt);
        }
        
        std::set<int> uniqueSet(polyhedron[shapeIt].uniqueVertIndices.begin(), polyhedron[shapeIt].uniqueVertIndices.end());
        polyhedron[shapeIt].uniqueVertIndices.clear();
        std::set<int> newSet;
        std::set_difference(uniqueSet.begin(), uniqueSet.end(), totalSet.begin(), totalSet.end(),
                                std::inserter(newSet, newSet.begin()));
        polyhedron[shapeIt].uniqueVertIndices.assign(newSet.begin(), newSet.end());
        totalSet.insert(uniqueSet.begin(), uniqueSet.end());
        
    }
    
    for ( int edgeIt=0; edgeIt<edgeIndices.size(); ++edgeIt)
    {
        searchEdge[0] = edgeIndices[edgeIt][1];
        searchEdge[1] = edgeIndices[edgeIt][0];
        for ( int searchIt=edgeIt+1; searchIt<edgeIndices.size(); ++searchIt)
        {
            if (edgeIndices[searchIt]==searchEdge)
            {
                std::vector<std::vector<int>>::iterator itEdge1 = edgeIndices.begin() + edgeIt+1;
                std::vector<std::vector<int>>::iterator itSearch1 = edgeIndices.begin() + searchIt+1;
                std::vector<int>::iterator itEdge2 = faceIndices.begin() + edgeIt+1;
                std::vector<int>::iterator itSearch2 = faceIndices.begin() + searchIt+1;
                std::vector<int>::iterator itEdge3 = shapeIndices.begin() + edgeIt+1;
                std::vector<int>::iterator itSearch3 = shapeIndices.begin() + searchIt+1;
                edgeIndices.emplace(itEdge1, searchEdge);
                edgeIndices.erase(itSearch1);
                faceIndices.emplace(itEdge2, faceIndices[searchIt]);
                faceIndices.erase(itSearch2);
                shapeIndices.emplace(itEdge3, shapeIndices[searchIt]);
                shapeIndices.erase(itSearch3);
                edgeIt++;
                continue;
            }
        }
    }
    
    for ( int edgeIt=0; edgeIt<edgeIndices.size(); edgeIt=edgeIt+2)
    {
        polyhedron[shapeIndices[edgeIt]].edgeIndices.push_back(edgeIndices[edgeIt]);
        polyhedron[shapeIndices[edgeIt]].faceIndices.push_back(std::make_tuple(faceIndices[edgeIt], shapeIndices[edgeIt+1], faceIndices[edgeIt+1]));
    }
    
    
    return polyhedron;
}

/*! This method allows the RB Contact state effector to have access to the hub states and gravity*/
void RigidBodyContactEffector::linkInStates(DynParamManager& statesIn)
{
//    while (this->Bodies[this->currentBodyInCycle].isSpice)
//    {
//        this->currentBodyInCycle++;
//    }
//
//    this->Bodies[this->currentBodyInCycle].hubState.hubPosition = statesIn.getStateObject("hubPosition");
//    this->Bodies[this->currentBodyInCycle].hubState.hubSigma = statesIn.getStateObject("hubSigma");
//    this->Bodies[this->currentBodyInCycle].hubState.hubOmega_BN_N = statesIn.getStateObject("hubOmega");
//    this->Bodies[this->currentBodyInCycle].hubState.hubVelocity = statesIn.getStateObject("hubVelocity");
//    this->Bodies[this->currentBodyInCycle].hubState.r_BN_N = statesIn.getPropertyReference( "r_BN_N");
//    this->Bodies[this->currentBodyInCycle].hubState.v_BN_N = statesIn.getPropertyReference( "v_BN_N");
//    this->Bodies[this->currentBodyInCycle].hubState.m_SC = statesIn.getPropertyReference( "m_SC");
//    this->Bodies[this->currentBodyInCycle].hubState.ISCPntB_B = statesIn.getPropertyReference( "inertiaSC");
//    this->Bodies[this->currentBodyInCycle].hubState.c_B = statesIn.getPropertyReference( "centerOfMassSC");
//
//    this->currentBodyInCycle++;
}

/*! This method computes the Forces on Torque on the Spacecraft Body.
@return void
@param integTime Integration time
@ToDo Distribute the mass at each contact point
*/
void RigidBodyContactEffector::computeForceTorque(double currentTime, double timeStep)
{
    this->forceExternal_N.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    //td::cout << currentTime << std::endl;
    
    Eigen::Vector3d forceExternalOther_N;
    Eigen::Vector3d torqueExternalOtherPntB_B;
    forceExternalOther_N.setZero();
    torqueExternalOtherPntB_B.setZero();
    
    dynamicData body1Current;
    dynamicData body2Current;
    dynamicData body1Future;
    dynamicData body2Future;
    
    vectorInterval tempVecInter;
    std::tuple<vectorInterval, vectorInterval, Eigen::Vector3d, Eigen::Vector3d> tempEdgeInter;
    
    std::vector<vectorInterval> body1UniqueVertInter;
    std::vector<vectorInterval> body2UniqueVertInter;
    std::vector<std::tuple<vectorInterval, vectorInterval, Eigen::Vector3d, Eigen::Vector3d>> body1EdgeInter;
    std::vector<std::tuple<vectorInterval, vectorInterval, Eigen::Vector3d, Eigen::Vector3d>> body2EdgeInter;
    
    std::tuple<vectorInterval, vectorInterval, vectorInterval> faceVertInter;
    
    vectorInterval faceLegInterval1;
    vectorInterval faceLegInterval2;
    vectorInterval supportInterval;
    
    std::vector<int> usedVerts;

    Eigen::Vector3d contactNormal_N;
    Eigen::Vector3d contactVelocity_N;
    
    std::vector<double> elemTest;
    int intersectFlag;
    Eigen::Vector3d contactPoint;
    Eigen::Vector3d contactPoint2;
    double contactError;
    double currentMaxError = 0.0;
    this->currentMinError = 100.0;
    int maxErrorInd=-1;
    int numImpacts;
    bool alreadyFound;
    
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
    Eigen::VectorXd dv3dj;
    Eigen::VectorXd tempVec;
    
    Eigen::VectorXd X_c;
    bool energyMet;
    int currLoop;
    int loopMax = 1e9;
    double colIntStep;
    Eigen::VectorXd k1;
    Eigen::VectorXd k2;
    Eigen::VectorXd k3;
    Eigen::VectorXd k4;
    Eigen::Vector3d impulse_Body1_N;
    
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>> impacts;
    
    if (this->justOneTime){
        return;
    }
    
    for (int groupIt1=0; groupIt1 < this->closeBodies.size(); ++groupIt1)
    {
        if (this->responseFound)
        {
            if ((this->timeFound >= currentTime) && (abs(timeStep - this->integrateTimeStep) < 1e-15))
            {
                this->forceExternal_N = this->Bodies[this->closeBodies[groupIt1][0]].forceExternal_N;// / timeStep;
                this->torqueExternalPntB_B = this->Bodies[this->closeBodies[groupIt1][0]].torqueExternalPntB_B;// / timeStep;
                //this->integrateTimeStep = timeStep;
                std::cout << "again" << std::endl;
                return;
            }
            this->responseFound = false;
            this->justOneTime = true;
            return;
        }
        
        // Fine collision detection begin
        if (this->Bodies[this->closeBodies[groupIt1][0]].isSpice == true)
        {
            body1Current.r_BN_N = this->Bodies[this->closeBodies[groupIt1][0]].states.r_BN_N + this->Bodies[this->closeBodies[groupIt1][0]].states.v_BN_N * (currentTime - this->currentSimSeconds);
            body1Current.v_BN_N = this->Bodies[this->closeBodies[groupIt1][0]].states.v_BN_N ;
            body1Current.dcm_BN = ((-this->Bodies[this->closeBodies[groupIt1][0]].states.omegaTilde_BN_B * this->Bodies[this->closeBodies[groupIt1][0]].states.dcm_BN) * (currentTime - this->currentSimSeconds)) + this->Bodies[this->closeBodies[groupIt1][0]].states.dcm_BN;
            body1Current.dcm_NB = body1Current.dcm_BN.transpose();
            body1Current.omegaTilde_BN_B = this->Bodies[this->closeBodies[groupIt1][0]].states.omegaTilde_BN_B;
            
            body1Future.r_BN_N = body1Current.r_BN_N + this->Bodies[this->closeBodies[groupIt1][0]].states.v_BN_N * timeStep ;
            body1Future.v_BN_N = this->Bodies[this->closeBodies[groupIt1][0]].states.v_BN_N ;
            body1Future.dcm_BN = ((-this->Bodies[this->closeBodies[groupIt1][0]].states.omegaTilde_BN_B * body1Current.dcm_BN) * timeStep) + body1Current.dcm_BN;
            body1Future.dcm_NB = body1Future.dcm_BN.transpose();
        }else
        {
            body1Current.r_BN_N = this->Bodies[this->closeBodies[groupIt1][0]].states.r_BN_N + this->Bodies[this->closeBodies[groupIt1][0]].states.v_BN_N * (currentTime - this->currentSimSeconds) + this->Bodies[this->closeBodies[groupIt1][0]].states.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][0]].states.nonConservativeAccelpntB_B * (currentTime - this->currentSimSeconds) * (currentTime - this->currentSimSeconds));
            body1Current.v_BN_N = this->Bodies[this->closeBodies[groupIt1][0]].states.v_BN_N + this->Bodies[this->closeBodies[groupIt1][0]].states.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][0]].states.nonConservativeAccelpntB_B * (currentTime - this->currentSimSeconds));
            body1Current.omega_BN_B = this->Bodies[this->closeBodies[groupIt1][0]].states.omega_BN_B + this->Bodies[this->closeBodies[groupIt1][0]].states.omegaDot_BN_B * (currentTime - this->currentSimSeconds);
            body1Current.sigma_BN = (0.25 * this->Bodies[this->closeBodies[groupIt1][0]].states.sigma_BN.Bmat() * body1Current.omega_BN_B * (currentTime - this->currentSimSeconds)) + ((Eigen::Vector3d) this->Bodies[this->closeBodies[groupIt1][0]].states.sigma_BN.coeffs());
            body1Current.dcm_NB = body1Current.sigma_BN.toRotationMatrix();
            body1Current.dcm_BN = body1Current.dcm_NB.transpose();
            body1Current.omegaTilde_BN_B = eigenTilde(body1Current.omega_BN_B);
            
            body1Future.r_BN_N = body1Current.r_BN_N + body1Current.v_BN_N * timeStep + body1Current.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][0]].states.nonConservativeAccelpntB_B * timeStep * timeStep);
            body1Future.omega_BN_B = body1Current.omega_BN_B + this->Bodies[this->closeBodies[groupIt1][0]].states.omegaDot_BN_B * timeStep;
            body1Future.sigma_BN = (0.25 * body1Current.sigma_BN.Bmat() * body1Future.omega_BN_B * timeStep) + ((Eigen::Vector3d) body1Current.sigma_BN.coeffs());
            body1Future.dcm_NB = body1Future.sigma_BN.toRotationMatrix();
            body1Future.dcm_BN = body1Future.dcm_NB.transpose();
        }
        
        if (this->Bodies[this->closeBodies[groupIt1][1]].isSpice == true)
        {
            body2Current.r_BN_N = this->Bodies[this->closeBodies[groupIt1][1]].states.r_BN_N + this->Bodies[this->closeBodies[groupIt1][1]].states.v_BN_N * (currentTime - this->currentSimSeconds);
            body2Current.v_BN_N = this->Bodies[this->closeBodies[groupIt1][1]].states.v_BN_N;
            body2Current.dcm_BN = ((-this->Bodies[this->closeBodies[groupIt1][1]].states.omegaTilde_BN_B * this->Bodies[this->closeBodies[groupIt1][1]].states.dcm_BN) * (currentTime - this->currentSimSeconds)) + this->Bodies[this->closeBodies[groupIt1][1]].states.dcm_BN;
            body2Current.dcm_NB = body2Current.dcm_BN.transpose();
            body2Current.omegaTilde_BN_B = this->Bodies[this->closeBodies[groupIt1][1]].states.omegaTilde_BN_B;
            
            body2Future.r_BN_N = body2Current.r_BN_N + this->Bodies[this->closeBodies[groupIt1][1]].states.v_BN_N * timeStep ;
            body2Future.v_BN_N = this->Bodies[this->closeBodies[groupIt1][1]].states.v_BN_N;
            body2Future.dcm_BN = ((-this->Bodies[this->closeBodies[groupIt1][1]].states.omegaTilde_BN_B * body2Current.dcm_BN) * timeStep) + body2Current.dcm_BN;
            body2Future.dcm_NB = body2Future.dcm_BN.transpose();
        }else
        {
            body2Current.r_BN_N = this->Bodies[this->closeBodies[groupIt1][1]].states.r_BN_N + this->Bodies[this->closeBodies[groupIt1][1]].states.v_BN_N * (currentTime - this->currentSimSeconds) + this->Bodies[this->closeBodies[groupIt1][1]].states.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][1]].states.nonConservativeAccelpntB_B * (currentTime - this->currentSimSeconds) * (currentTime - this->currentSimSeconds));
            body2Current.v_BN_N = this->Bodies[this->closeBodies[groupIt1][1]].states.v_BN_N + this->Bodies[this->closeBodies[groupIt1][1]].states.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][1]].states.nonConservativeAccelpntB_B * (currentTime - this->currentSimSeconds));
            body2Current.omega_BN_B = this->Bodies[this->closeBodies[groupIt1][1]].states.omega_BN_B + this->Bodies[this->closeBodies[groupIt1][1]].states.omegaDot_BN_B * (currentTime - this->currentSimSeconds);
            body2Current.sigma_BN = (0.25 * this->Bodies[this->closeBodies[groupIt1][1]].states.sigma_BN.Bmat() * body2Current.omega_BN_B * (currentTime - this->currentSimSeconds)) + ((Eigen::Vector3d) this->Bodies[this->closeBodies[groupIt1][1]].states.sigma_BN.coeffs());
            body2Current.dcm_NB = body2Current.sigma_BN.toRotationMatrix();
            body2Current.dcm_BN = body2Current.dcm_NB.transpose();
            body2Current.omegaTilde_BN_B = eigenTilde(body2Current.omega_BN_B);
            
            body2Future.r_BN_N = body2Current.r_BN_N + body2Current.v_BN_N * timeStep + body2Current.dcm_NB * (this->Bodies[this->closeBodies[groupIt1][1]].states.nonConservativeAccelpntB_B * timeStep * timeStep);
            body2Future.omega_BN_B = body2Current.omega_BN_B + this->Bodies[this->closeBodies[groupIt1][0]].states.omegaDot_BN_B * timeStep;
            body2Future.sigma_BN = (0.25 * body2Current.sigma_BN.Bmat() * body2Future.omega_BN_B * timeStep) + ((Eigen::Vector3d) body2Current.sigma_BN.coeffs());
            body2Future.dcm_NB = body2Future.sigma_BN.toRotationMatrix();
            body2Future.dcm_BN = body2Future.dcm_NB.transpose();
        }
        
        //std::cout << body1Current.r_BN_N(2) - 0.5 << " " << body1Future.r_BN_N(2) - 0.5 << std::endl;
        
        // Begin looping through contactable triangles
        for (int polyPairInd=0; polyPairInd < this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps.size(); polyPairInd++)
        {
            body1UniqueVertInter.clear();
            body2UniqueVertInter.clear();
            body1EdgeInter.clear();
            body2EdgeInter.clear();
            
            for (int uniqueVertInd=0; uniqueVertInd < this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].uniqueVertIndices.size(); uniqueVertInd++)
            {
                tempVecInter.lower = body1Current.r_BN_N + body1Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].uniqueVertIndices[uniqueVertInd]];
                tempVecInter.upper = body1Future.r_BN_N + body1Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].uniqueVertIndices[uniqueVertInd]];
                
                body1UniqueVertInter.push_back(tempVecInter);
            }
            
            for (int uniqueVertInd=0; uniqueVertInd < this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].uniqueVertIndices.size(); uniqueVertInd++)
            {
                tempVecInter.lower = body2Current.r_BN_N + body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].uniqueVertIndices[uniqueVertInd]];
                tempVecInter.upper = body2Future.r_BN_N + body2Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].uniqueVertIndices[uniqueVertInd]];
                
                body2UniqueVertInter.push_back(tempVecInter);
            }
            
            for (int edgeInd=0; edgeInd < this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].edgeIndices.size(); edgeInd++)
            {
                tempVecInter.lower = body1Current.r_BN_N + body1Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].edgeIndices[edgeInd][0]];
                tempVecInter.upper = body1Future.r_BN_N + body1Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].edgeIndices[edgeInd][0]];
                std::get<0>(tempEdgeInter) = tempVecInter;
                
                tempVecInter.lower = body1Current.r_BN_N + body1Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].edgeIndices[edgeInd][1]];
                tempVecInter.upper = body1Future.r_BN_N + body1Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].edgeIndices[edgeInd][1]];
                std::get<1>(tempEdgeInter) = tempVecInter;
                
                std::get<2>(tempEdgeInter) = body1Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceNormals[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceIndices[edgeInd])];
                
                std::get<3>(tempEdgeInter) = body1Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceIndices[edgeInd])].faceNormals[std::get<2>(this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceIndices[edgeInd])];
                
                body1EdgeInter.push_back(tempEdgeInter);
            }
            
            for (int edgeInd=0; edgeInd < this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].edgeIndices.size(); edgeInd++)
            {
                tempVecInter.lower = body2Current.r_BN_N + body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].edgeIndices[edgeInd][0]];
                tempVecInter.upper = body2Future.r_BN_N + body2Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].edgeIndices[edgeInd][0]];
                std::get<0>(tempEdgeInter) = tempVecInter;
                
                tempVecInter.lower = body2Current.r_BN_N + body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].edgeIndices[edgeInd][1]];
                tempVecInter.upper = body2Future.r_BN_N + body2Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].edgeIndices[edgeInd][1]];
                std::get<1>(tempEdgeInter) = tempVecInter;
                
                std::get<2>(tempEdgeInter) = body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceNormals[std::get<0>(this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceIndices[edgeInd])];
                
                std::get<3>(tempEdgeInter) = body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceIndices[edgeInd])].faceNormals[std::get<2>(this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceIndices[edgeInd])];
                
                body2EdgeInter.push_back(tempEdgeInter);
            }

            
            // Face of triange 1 with each vertex of triangle 2
            for (int faceInd=0; faceInd < this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles.size(); faceInd++)
            {
                usedVerts.clear();
                
                tempVecInter.lower = body1Current.r_BN_N + body1Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][0]];
                tempVecInter.upper = body1Future.r_BN_N + body1Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][0]];

                std::get<0>(faceVertInter) = tempVecInter;

                tempVecInter.lower = body1Current.r_BN_N + body1Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][1]];
                tempVecInter.upper = body1Future.r_BN_N + body1Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][1]];

                std::get<1>(faceVertInter) = tempVecInter;

                tempVecInter.lower = body1Current.r_BN_N + body1Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][2]];
                tempVecInter.upper = body1Future.r_BN_N + body1Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][0]].vertices[this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][2]];
                
                std::get<2>(faceVertInter) = tempVecInter;
                
                faceLegInterval1.lower = std::get<0>(faceVertInter).lower - std::get<1>(faceVertInter).lower;
                faceLegInterval1.upper = std::get<0>(faceVertInter).upper - std::get<1>(faceVertInter).upper;
                faceLegInterval2.lower = std::get<0>(faceVertInter).lower - std::get<2>(faceVertInter).lower;
                faceLegInterval2.upper = std::get<0>(faceVertInter).upper - std::get<2>(faceVertInter).upper;
                
                for (int vertInd=0; vertInd < body2UniqueVertInter.size(); vertInd++)
                {
                    supportInterval.lower = body2UniqueVertInter[vertInd].lower - std::get<0>(faceVertInter).lower;
                    supportInterval.upper = body2UniqueVertInter[vertInd].upper - std::get<0>(faceVertInter).upper;
                    
                    elemTest = this->IntervalDotProduct(supportInterval, this->IntervalCrossProduct(faceLegInterval1, faceLegInterval2));
                    
                    if (((elemTest[0] <= -1e-12) && (elemTest[1] >= 1e-12)) || ((elemTest[0] >= 1e-12) && (elemTest[1] <= -1e-12)))
                    {
                        intersectFlag = this->PointInTriangle(body2UniqueVertInter[vertInd].lower, std::get<0>(faceVertInter).lower, std::get<1>(faceVertInter).lower, std::get<2>(faceVertInter).lower, &contactPoint, &contactError);
                        
                        if (intersectFlag == 1)
                        {
                            //contactError = (contactPoint - body2VertInter[vertInd].lower).norm();
                            if ((contactError <= this->maxPosError) || (contactError <= (this->currentMinError + 1e-15)))
                            {
                                alreadyFound = false;
                                for (int impInd=0; impInd < impacts.size(); impInd++){
                                    if (((contactPoint - std::get<0>(impacts[impInd])).norm() < 1e-3) || ((body2UniqueVertInter[vertInd].lower - std::get<1>(impacts[impInd])).norm() < 1e-3))
                                    {
                                        alreadyFound = true;
                                        if (impInd == maxErrorInd)
                                        {
                                            currentMaxError = this->currentMinError;
                                        }
                                        impacts.erase(impacts.begin()+impInd);
                                        break;
                                    }
                                }
                                //if (alreadyFound) continue;
                                impacts.push_back(std::make_tuple(contactPoint, body2UniqueVertInter[vertInd].lower, body1Current.dcm_NB * -this->Bodies[this->closeBodies[groupIt1][0]].polyhedron[std::get<0>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceNormals[faceInd]));
                                usedVerts.push_back(vertInd);
                                std::cout << "Face 1 Verts 2: " << std::get<2>(impacts.back())[0] << " " << std::get<2>(impacts.back())[1] << " " << std::get<2>(impacts.back())[2] << std::endl;
                                if (contactError > currentMaxError)
                                {
                                    currentMaxError = contactError;
                                    maxErrorInd = impacts.size()-1;
                                }else{
                                    this->currentMinError = contactError;
                                }
                                
                            }
                        }
                    }
                }
                for (int vertInd=usedVerts.size()-1; vertInd >=0; vertInd--)
                {
                    body2UniqueVertInter.erase(body2UniqueVertInter.begin()+usedVerts[vertInd]);
                }
                
            }
            
            
            
            
            
            
            // Face of triange 2 with each vertex of triangle 1
            for (int faceInd=0; faceInd < this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles.size(); faceInd++)
            {
                usedVerts.clear();
                tempVecInter.lower = body2Current.r_BN_N + body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][0]];
                tempVecInter.upper = body2Future.r_BN_N + body2Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][0]];

                std::get<0>(faceVertInter) = tempVecInter;

                tempVecInter.lower = body2Current.r_BN_N + body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][1]];
                tempVecInter.upper = body2Future.r_BN_N + body2Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][1]];

                std::get<1>(faceVertInter) = tempVecInter;

                tempVecInter.lower = body2Current.r_BN_N + body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][2]];
                tempVecInter.upper = body2Future.r_BN_N + body2Future.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].vertices[this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceTriangles[faceInd][2]];
                
                std::get<2>(faceVertInter) = tempVecInter;
                
                faceLegInterval1.lower = std::get<0>(faceVertInter).lower - std::get<1>(faceVertInter).lower;
                faceLegInterval1.upper = std::get<0>(faceVertInter).upper - std::get<1>(faceVertInter).upper;
                faceLegInterval2.lower = std::get<0>(faceVertInter).lower - std::get<2>(faceVertInter).lower;
                faceLegInterval2.upper = std::get<0>(faceVertInter).upper - std::get<2>(faceVertInter).upper;
                
                for (int vertInd=0; vertInd < body1UniqueVertInter.size(); vertInd++)
                {
                    supportInterval.lower = body1UniqueVertInter[vertInd].lower - std::get<0>(faceVertInter).lower;
                    supportInterval.upper = body1UniqueVertInter[vertInd].upper - std::get<0>(faceVertInter).upper;
                    
                    elemTest = this->IntervalDotProduct(supportInterval, this->IntervalCrossProduct(faceLegInterval1, faceLegInterval2));
                    
                    if (((elemTest[0] <= -1e-12) && (elemTest[1] >= 1e-12)) || ((elemTest[0] >= 1e-12) && (elemTest[1] <= -1e-12)))
                    {
                        intersectFlag = this->PointInTriangle(body1UniqueVertInter[vertInd].lower, std::get<0>(faceVertInter).lower, std::get<1>(faceVertInter).lower, std::get<2>(faceVertInter).lower, &contactPoint, &contactError);
                        
                        
                        if (intersectFlag == 1)
                        {
                            if ((contactError <= this->maxPosError) || (contactError <= (this->currentMinError + 1e-15)))
                            {
                                alreadyFound = false;
                                for (int impInd=0; impInd < impacts.size(); impInd++){
                                    if (((body1UniqueVertInter[vertInd].lower - std::get<0>(impacts[impInd])).norm() < 1e-3) || ((contactPoint - std::get<1>(impacts[impInd])).norm() < 1e-3))
                                    {
                                        alreadyFound = true;
                                        if (impInd == maxErrorInd)
                                        {
                                            currentMaxError = this->currentMinError;
                                        }
                                        impacts.erase(impacts.begin()+impInd);
                                        break;
                                    }
                                }
                                //if (alreadyFound) continue;
                                
                                impacts.push_back(std::make_tuple(body1UniqueVertInter[vertInd].lower, contactPoint, body2Current.dcm_NB * this->Bodies[this->closeBodies[groupIt1][1]].polyhedron[std::get<1>(this->Bodies[this->closeBodies[groupIt1][0]].coarseSearchList.overlaps[polyPairInd])].faceNormals[faceInd]));
                                usedVerts.push_back(vertInd);
                                std::cout << "Face 2 Verts 1: " << std::get<2>(impacts.back())[0] << " " << std::get<2>(impacts.back())[1] << " " << std::get<2>(impacts.back())[2] << std::endl;
                                if (contactError > currentMaxError)
                                {
                                    currentMaxError = contactError;
                                    maxErrorInd = impacts.size()-1;
                                }else{
                                    this->currentMinError = contactError;
                                }
                            }
                        }
                    }
                }
                for (int vertInd=usedVerts.size()-1; vertInd >=0; vertInd--)
                {
                    body1UniqueVertInter.erase(body1UniqueVertInter.begin()+usedVerts[vertInd]);
                }
            }
            
            // Each edge of triangle 1 with each edge of triangle 2
            for (int vertInd1=0; vertInd1 < body1EdgeInter.size(); vertInd1++)
            {

                // Reuse the faceLegInterval variables, but these should be called edgeInterval
                faceLegInterval1.lower = std::get<1>(body1EdgeInter[vertInd1]).lower - std::get<0>(body1EdgeInter[vertInd1]).lower;
                faceLegInterval1.upper = std::get<1>(body1EdgeInter[vertInd1]).upper - std::get<0>(body1EdgeInter[vertInd1]).upper;
                
                for (int vertInd2=0; vertInd2 < body2EdgeInter.size(); vertInd2++)
                {

                    faceLegInterval2.lower = std::get<1>(body2EdgeInter[vertInd2]).lower - std::get<0>(body2EdgeInter[vertInd2]).lower;
                    faceLegInterval2.upper = std::get<1>(body2EdgeInter[vertInd2]).upper - std::get<0>(body2EdgeInter[vertInd2]).upper;
                    // Reuse supportInterval, but it should be called edgeIntervalMixed
                    supportInterval.lower = std::get<0>(body2EdgeInter[vertInd2]).lower - std::get<0>(body1EdgeInter[vertInd1]).lower;
                    supportInterval.upper = std::get<0>(body2EdgeInter[vertInd2]).upper - std::get<0>(body1EdgeInter[vertInd1]).upper;
                    
                    elemTest = this->IntervalDotProduct(supportInterval, this->IntervalCrossProduct(faceLegInterval1, faceLegInterval2));

                    
                    if (((elemTest[0] < 0) && (elemTest[1] > 0)) || ((elemTest[0] > 0) && (elemTest[1] < 0)))
                    {
                        intersectFlag = this->LineLineDistance(std::get<0>(body1EdgeInter[vertInd1]).lower, std::get<1>(body1EdgeInter[vertInd1]).lower, std::get<0>(body2EdgeInter[vertInd2]).lower, std::get<1>(body2EdgeInter[vertInd2]).lower, &contactPoint, &contactPoint2);
                        
                        
                        if ((intersectFlag == 0) || (intersectFlag == 1))
                        {
                            contactError = (contactPoint - contactPoint2).norm();
                            if ((contactError <= this->maxPosError) || (contactError <= (this->currentMinError + 1e-15)))
                            {
                                alreadyFound = false;
                                for (int impInd=0; impInd < impacts.size(); impInd++){
                                    if (((contactPoint - std::get<0>(impacts[impInd])).norm() < 1e-3) || ((contactPoint2 - std::get<1>(impacts[impInd])).norm() < 1e-3))
                                    {
                                        alreadyFound = true;
                                        if (impInd == maxErrorInd)
                                        {
                                            currentMaxError = this->currentMinError;
                                        }
                                        impacts.erase(impacts.begin()+impInd);
                                        break;
                                    }
                                }
                                //if (alreadyFound) continue;
                                contactNormal_N = ((contactPoint - contactPoint2) * 1.0e6).normalized();
                                contactVelocity_N = (body1Current.v_BN_N + body1Current.dcm_NB * (body1Current.omegaTilde_BN_B * body1Current.dcm_BN * (contactPoint - body1Current.r_BN_N))) - (body2Current.v_BN_N + body2Current.dcm_NB * (body2Current.omegaTilde_BN_B * body2Current.dcm_BN * (contactPoint2 - body2Current.r_BN_N)));
                                if (contactVelocity_N.dot(contactNormal_N) > 0)
                                {
                                    continue;
                                }
                                impacts.push_back(std::make_tuple(contactPoint, contactPoint2, (-contactVelocity_N).normalized()));
//                                impacts.push_back(std::make_tuple(contactPoint, contactPoint2, ((contactPoint - contactPoint2) * 1.0e6).normalized()));
                                std::cout << "Edge 0: " << std::get<2>(impacts.back())[0] << " " << std::get<2>(impacts.back())[1] << " " << std::get<2>(impacts.back())[2] << std::endl;
                                if (contactError > currentMaxError)
                                {
                                    currentMaxError = contactError;
                                    maxErrorInd = impacts.size()-1;
                                }else{
                                    this->currentMinError = contactError;
                                }
                            }
                        }else if (intersectFlag == 1)
                        {
                            contactError = (contactPoint - contactPoint2).norm();
                            if ((contactError <= this->maxPosError) || (contactError <= (this->currentMinError + 1e-10)))
                            {
                                alreadyFound = false;
                                for (int impInd=0; impInd < impacts.size(); impInd++){
                                    if (((contactPoint - std::get<0>(impacts[impInd])).norm() < 1e-3) || ((contactPoint2 - std::get<1>(impacts[impInd])).norm() < 1e-3))
                                    {
                                        alreadyFound = true;
                                        if (impInd == maxErrorInd)
                                        {
                                            currentMaxError = this->currentMinError;
                                        }
                                        impacts.erase(impacts.begin()+impInd);
                                        break;
                                    }
                                }
                                //if (alreadyFound) continue;
                                
                                contactNormal_N = ((faceLegInterval1.lower).cross(faceLegInterval2.lower)).normalized();
                                contactVelocity_N = (body1Current.v_BN_N + body1Current.dcm_NB * (body1Current.omegaTilde_BN_B * body1Current.dcm_BN * (contactPoint - body1Current.r_BN_N))) - (body2Current.v_BN_N + body2Current.dcm_NB * (body2Current.omegaTilde_BN_B * body2Current.dcm_BN * (contactPoint2 - body2Current.r_BN_N)));
                                if ((contactNormal_N.dot(std::get<2>(body2EdgeInter[vertInd2])) < 1e-12) && (contactNormal_N.dot(std::get<3>(body2EdgeInter[vertInd2])) < 1e-12))
                                {
                                    contactNormal_N = -contactNormal_N;
                                }
                                if (contactVelocity_N.dot(contactNormal_N) > 0)
                                {
                                    continue;
                                }
                                impacts.push_back(std::make_tuple(contactPoint, contactPoint2, contactNormal_N));
                                std::cout << "Edge 1: " << std::get<2>(impacts.back())[0] << " " << std::get<2>(impacts.back())[1] << " " << std::get<2>(impacts.back())[2] << std::endl;
                                if (contactError > currentMaxError)
                                {
                                    currentMaxError = contactError;
                                    maxErrorInd = impacts.size()-1;
                                }else{
                                    this->currentMinError = contactError;
                                }
                            }
                        }
                    }
                }
            }
        } // Fine collision detection end
        
        // Calculate total impact begin
        numImpacts = impacts.size();
        //std::cout << this->currentMinError << std::endl;
        //std::cout << numImpacts << std::endl;
        if (numImpacts == 0)
        {
            this->lockedToRand = true;
            this->timeFound = currentTime + timeStep + 1.0e-15;
            this->integrateTimeStep = timeStep;
            return;
        }
        if (this->lockedToRand)
        {
            if ((abs(timeStep - this->integrateTimeStep) < 1e-15))
            {
                this->forceExternal_N[0] = ((std::rand() % 1000) + 1000.0) / timeStep;
                this->forceExternal_N[1] = ((std::rand() % 1000) + 1000.0) / timeStep;
                this->forceExternal_N[2] = ((std::rand() % 1000) + 1000.0) / timeStep;
                this->torqueExternalPntB_B[0] = ((std::rand() % 1000) + 1000.0) / timeStep;
                this->torqueExternalPntB_B[1] = ((std::rand() % 1000) + 1000.0) / timeStep;
                this->torqueExternalPntB_B[2] = ((std::rand() % 1000) + 1000.0) / timeStep;
                return;
            }
            this->lockedToRand = false;
        }
        if (currentMaxError > this->maxPosError)
        {
            this->forceExternal_N[0] = ((std::rand() % 1000) + 1000.0) / timeStep;
            this->forceExternal_N[1] = ((std::rand() % 1000) + 1000.0) / timeStep;
            this->forceExternal_N[2] = ((std::rand() % 1000) + 1000.0) / timeStep;
            this->Bodies[this->closeBodies[groupIt1][1]].forceExternal_N = -this->forceExternal_N;
            this->torqueExternalPntB_B[0] = ((std::rand() % 1000) + 1000.0) / timeStep;
            this->torqueExternalPntB_B[1] = ((std::rand() % 1000) + 1000.0) / timeStep;
            this->torqueExternalPntB_B[2] = ((std::rand() % 1000) + 1000.0) / timeStep;
//            this->Bodies[this->closeBodies[groupIt1][1]].torqueExternalPntB_B = -this->torqueExternalPntB_B;
            this->lockedToRand = true;
            this->timeFound = currentTime + timeStep + 1.0e-15;
            this->integrateTimeStep = timeStep;
            return;
        }
        for (int impNum=0; impNum < numImpacts; impNum++)
        {
            //std::cout << std::get<0>(impacts[impNum]) << std::endl;
            // Create local contact frame
            cHat_3 = (std::get<2>(impacts[impNum])).normalized();
            //std::cout << std::get<2>(impacts[impNum])[0] << " " << std::get<2>(impacts[impNum])[1] << " " << std::get<2>(impacts[impNum])[2] << std::endl;
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
        if (this->Bodies[this->closeBodies[groupIt1][1]].isSpice)
        {
            for (int ii=0; ii < numImpacts; ii++)
            {
                for (int jj=0; jj < numImpacts; jj++)
                {
                    M_C = ((1.0 / this->Bodies[this->closeBodies[groupIt1][0]].states.m_SC) * Eigen::MatrixXd::Identity(3, 3) - eigenTilde(dcm_CN[ii] * std::get<0>(impacts[ii])) * (dcm_CB1[ii] * this->Bodies[this->closeBodies[groupIt1][0]].states.ISCPntB_B_inv * dcm_CB1[ii].transpose()) * eigenTilde(dcm_CN[ii] * std::get<0>(impacts[jj])));
                    
                    if (ii == jj)
                    {
                        M_tot.block(ii*3, jj*3, 3, 3) = M_C;
                    }else
                    {
                        M_tot.block(ii*3, jj*3, 3, 3) = M_C * (dcm_CB1[ii] * dcm_CB1[jj].transpose());
                    }
                }
            }
        }else
        {
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
        }
        
        // Solve for the critical values of friction and sliding direction
//        if (impacts.size() == 1)
//        {
//            f_vals.push_back(sqrt((pow(M_tot(0,1) * M_tot(1,2) - M_tot(1,1) * M_tot(0,2), 2) + pow(M_tot(1,0) * M_tot(0,2) - M_tot(0,0) * M_tot(1,2), 2)) / pow(M_tot(0,0) * M_tot(1,1) - M_tot(0,1) * M_tot(1,0), 2)));
//
//            phi_vals.push_back(atan2(M_tot(0,0) * M_tot(1,2) - M_tot(1,0) * M_tot(0,2), M_tot(1,1) * M_tot(0,2) - M_tot(0,1) * M_tot(1,3)));
//        }else
//        {
//            M_inv = M_tot.completeOrthogonalDecomposition().pseudoInverse();
//            M_inv_red1 = Eigen::MatrixXd::Zero(3*numImpacts, numImpacts);
//            tempSel = 2;
//            for (int ii=0; ii < numImpacts; ii++)
//            {
//                M_inv_red1.block(0, ii, 3*numImpacts, 1) = M_inv.block(0, tempSel, 3*numImpacts, 1);
//                tempSel += 3;
//            }
//            M_inv_red2 = Eigen::MatrixXd::Zero(numImpacts, numImpacts);
//            tempSel = 2;
//            for (int ii=0; ii < numImpacts; ii++)
//            {
//                M_inv_red2.block(ii, 0, 1, numImpacts) = M_inv_red1.block(tempSel, 0, 1, numImpacts);
//                tempSel += 3;
//            }
//
//            dv3dj = M_inv_red2.completeOrthogonalDecomposition().pseudoInverse() * Eigen::VectorXd::Ones(numImpacts);
//            tempVec = Eigen::VectorXd::Zero(3*numImpacts);
//            tempSel = 2;
//            for (int ii=0; ii < numImpacts; ii++)
//            {
//                tempVec(tempSel) = dv3dj(ii);
//                tempSel += 3;
//            }
//
//            tempVec = M_inv * tempVec;
//            tempSel = 0;
//            for (int ii=0; ii < numImpacts; ii++)
//            {
//                f_vals.push_back(sqrt(pow(tempVec(tempSel), 2) + pow(tempVec(tempSel+1), 2)));
//                phi_vals.push_back(atan2(tempVec(tempSel+1), tempVec(tempSel)));
//                tempSel += 3;
//            }
//        }
        
        // Create the initial collision state
        X_c = Eigen::VectorXd::Zero(numImpacts * 8);
        for (int impNum=0; impNum < numImpacts; impNum++)
        {
            // Velocity of the contact point on body 1 relative to the contact point on body 2, in the local contact frame
            X_c.segment(impNum * 3, 3) = dcm_CN[impNum] * ((body1Current.v_BN_N + body1Current.dcm_NB * (body1Current.omegaTilde_BN_B * (body1Current.dcm_BN * std::get<0>(impacts[impNum]) - body1Current.r_BN_N))) - (body2Current.v_BN_N + body2Current.dcm_NB * (body2Current.omegaTilde_BN_B * (body2Current.dcm_BN * std::get<1>(impacts[impNum]) - body2Current.r_BN_N))));
            
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
//        if ((currentMaxError <= this->maxPosError))
//        {
            loopMax = 1e9;
            colIntStep = this->collisionIntegrationStep;
//        }else
//        {
//            loopMax = 1e6;
//            colIntStep = 1e-2;
//        }
        std::cout << "Entering Impule Integration" << std::endl;
        while (!energyMet)
        {
            currLoop++;
            // Use RK4 to integrate the collision state
            k1 = this->CollisionStateDerivative(X_c, impacts, f_vals, phi_vals, M_tot, this->Bodies[this->closeBodies[groupIt1][0]].coefRestitution, this->Bodies[this->closeBodies[groupIt1][0]].coefFriction);
            k2 = this->CollisionStateDerivative(X_c + colIntStep * (k1 / 2.0), impacts, f_vals, phi_vals, M_tot, this->Bodies[this->closeBodies[groupIt1][0]].coefRestitution, this->Bodies[this->closeBodies[groupIt1][0]].coefFriction);
            k3 = this->CollisionStateDerivative(X_c + colIntStep * (k2 / 2.0), impacts, f_vals, phi_vals, M_tot, this->Bodies[this->closeBodies[groupIt1][0]].coefRestitution, this->Bodies[this->closeBodies[groupIt1][0]].coefFriction);
            k4 = this->CollisionStateDerivative(X_c + colIntStep * k3, impacts, f_vals, phi_vals, M_tot, this->Bodies[this->closeBodies[groupIt1][0]].coefRestitution, this->Bodies[this->closeBodies[groupIt1][0]].coefFriction);
            X_c = X_c + (colIntStep / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
            
            energyMet = true;
            // Check if there are any contact points that have not met the restitution energy requirements
            for (int impNum=0; impNum < numImpacts; impNum++)
            {
                if (X_c(numImpacts * 6 + impNum * 2 + 1) < (-1.0 * (pow(this->Bodies[this->closeBodies[groupIt1][0]].coefRestitution, 2.0) * X_c(numImpacts * 6 + impNum * 2))))
                {
                    energyMet = false;
                    break;
                }
            }
            
            // Hard cap on number of loops, which should never be reached.
            if (currLoop > loopMax)
            {
                if (currentMaxError <= this->maxPosError)
                {
                    std::cout << "hit cap" << std::endl;
                }
                energyMet = true;
            }
        }
        
        // Extract resulting force and torque
        for (int impNum=0; impNum < numImpacts; impNum++)
        {
            impulse_Body1_N = dcm_CN[impNum].transpose() * X_c.segment(numImpacts * 3 + impNum * 3, 3);
            forceExternalOther_N -= impulse_Body1_N / timeStep;
            torqueExternalOtherPntB_B -= body2Current.dcm_BN * (std::get<1>(impacts[impNum]) - body2Current.r_BN_N).cross(impulse_Body1_N / timeStep);
            this->forceExternal_N += impulse_Body1_N / timeStep;
            this->torqueExternalPntB_B += body1Current.dcm_BN * (std::get<0>(impacts[impNum]) - body1Current.r_BN_N).cross(impulse_Body1_N / timeStep);
        }
        
        if (currentMaxError <= this->maxPosError)
        {
            std::cout << this->forceExternal_N << std::endl;
            std::cout << this->torqueExternalPntB_B << std::endl;
            std::cout << std::endl << timeStep << std::endl;
            this->responseFound = true;
            this->timeFound = currentTime + timeStep + 1.0e-15;
            this->integrateTimeStep = timeStep;
            this->Bodies[this->closeBodies[groupIt1][0]].forceExternal_N = this->forceExternal_N;
            this->Bodies[this->closeBodies[groupIt1][1]].forceExternal_N = forceExternalOther_N;
            this->Bodies[this->closeBodies[groupIt1][0]].torqueExternalPntB_B = this->torqueExternalPntB_B;
            this->Bodies[this->closeBodies[groupIt1][1]].torqueExternalPntB_B = torqueExternalOtherPntB_B;
        }
        this->forceExternal_N = this->forceExternal_N;// / timeStep;
        this->torqueExternalPntB_B = this->torqueExternalPntB_B;// / timeStep;
        return;
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
        if (X_c(numImpacts * 6 + impNum * 2 + 1) < (-1.0 * (pow(coefRes, 2.0) * X_c(numImpacts * 6 + impNum * 2))))
        {
//            if (sqrt(pow(X_c(impNum * 3), 2) + pow(X_c(impNum * 3 + 1), 2)) < 1.0e-6)
//            {
//                Xdot_c(numImpacts * 3 + impNum * 3) = -f_vals[impNum] * cos(phi_vals[impNum]);
//                Xdot_c(numImpacts * 3 + impNum * 3 + 1) = -f_vals[impNum] * sin(phi_vals[impNum]);
//                Xdot_c(numImpacts * 3 + impNum * 3 + 2) = 1.0;
//            }else
//            {
                phi = atan2(X_c(impNum * 3 + 1), X_c(impNum * 3));
                Xdot_c(numImpacts * 3 + impNum * 3) = -coefFric * cos(phi);
                Xdot_c(numImpacts * 3 + impNum * 3 + 1) = -coefFric * sin(phi);
                Xdot_c(numImpacts * 3 + impNum * 3 + 2) = 1.0;
//            }
        }
        
        if (X_c(impNum * 3 + 2) < 0)
        {
            Xdot_c(numImpacts * 6 + impNum * 2) = X_c(impNum * 3 + 2);
        }else if (X_c(numImpacts * 6 + impNum * 2 + 1) < (-1.0 * (pow(coefRes, 2.0) * X_c(numImpacts * 6 + impNum * 2))))
        {
            Xdot_c(numImpacts * 6 + impNum * 2 + 1) = X_c(impNum * 3 + 2);
        }
    }
    
    Xdot_c.head(numImpacts * 3) = M_tot * Xdot_c.segment(numImpacts * 3, numImpacts * 3);

    return Xdot_c;
}



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
    this->currentMinError = 1.0;
    this->responseFound = false;
    this->lockedToRand = false;
    
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
//    std::cout << CurrentSimNanos*NANO2SEC << std::endl;
//    this->computeForceTorque(CurrentSimNanos*NANO2SEC, this->simTimeStep);
    
    return;
}

/*! This method is used to read the messages pertaining to all external bodies.
@return void
*/
void RigidBodyContactEffector::ReadInputs()
{

    for (int bodyIt=0; bodyIt<this->numBodies; bodyIt++)
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
            this->Bodies[bodyIt].states.ISCPntB_B = cArray2EigenMatrix3d(*this->Bodies[bodyIt].massStateInBuffer.ISC_PntB_B);
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
        
                layer1Box.overlaps.push_back(std::make_tuple(boxIt1, boxIt2));
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
    Eigen::Vector3d line13, line23, line43, line21, line24, line41, temp;
    double d1343, d4321, d1321, d4343, d2121;
    double numer, denom;
    double mua, mub;
    int retValue = 1;
    
    line13 = vertex1 - vertex3;
    line43 = vertex4 - vertex3;
    line21 = vertex2 - vertex1;
    if (line21.dot(line43) < 0.0)
    {
        temp = vertex3;
        vertex3 = vertex4;
        vertex4 = temp;
        line13 = vertex1 - vertex3;
        line43 = vertex4 - vertex3;
    }
    
    d1343 = line13[0] * line43[0] + line13[1] * line43[1] + line13[2] * line43[2];
    d4321 = line43[0] * line21[0] + line43[1] * line21[1] + line43[2] * line21[2];
    d1321 = line13[0] * line21[0] + line13[1] * line21[1] + line13[2] * line21[2];
    d4343 = line43[0] * line43[0] + line43[1] * line43[1] + line43[2] * line43[2];
    d2121 = line21[0] * line21[0] + line21[1] * line21[1] + line21[2] * line21[2];
    
    denom = d2121 * d4343 - d4321 * d4321;
    if (abs(denom) < 1e-9)
    {
        line23 = vertex2 - vertex3;
        line24 = vertex2 - vertex4;
        line41 = vertex4 - vertex1;
        //dotRes = (line13.normalized()).dot(line23.normalized());
        if ((line13.dot(line43) < 0.0) && (line24.dot(-line43) > 0.0))
        {
            *pointA = vertex1 + ((-line13).dot(line21) / d2121) * line21;
            *pointA = (*pointA + vertex2) / 2.0;
            *pointB = vertex3 + (line23.dot(line43) / d4343) * line43;
            *pointB = (*pointB + vertex3) / 2.0;
            return 0;
        }
        
        //dotRes = (line14.normalized()).dot(line24.normalized());
        if ((line13.dot(line43) < 0.0) && (line24.dot(-line43) < 0.0))
        {
            *pointA = vertex1 + ((-line13).dot(line21) / d2121) * line21;
            *pointA = (*pointA + (vertex1 + (line41.dot(line21) / d2121) * line21)) / 2.0;
            *pointB = (vertex3 + vertex4) / 2.0;
            return 0;
        }
        
        //dotRes = (line21.normalized()).dot(-line23.normalized());
        if (((-line43).dot(line24) < 0.0) && (line13.dot(line43) > 0.0))
        {
            *pointA = vertex1 + (line41.dot(line21) / d2121) * line21;
            *pointA = (*pointA + vertex1) / 2.0;
            *pointB = vertex3 + (line13.dot(line43) / d4343) * line43;
            *pointB = (*pointB + vertex4) / 2.0;
            return 0;
        }
        
        //dotRes = (line14.normalized()).dot(line21.normalized());
        if (((-line43).dot(line24) > 0.0) && (line13.dot(line43) > 0.0))
        {
            *pointA = (vertex1 + vertex2) / 2.0;
            *pointB = vertex3 + (line13.dot(line43) / d4343) * line43;
            *pointB = (*pointB + (vertex3 + (line23.dot(line43) / d4343) * line43)) / 2.0;
            return 0;
        }
        
        if ((line21.dot(-line23) >= -1e-6) && (line21.dot(-line23) <= 1e-6))
        {
            *pointA = vertex2;
            *pointB = vertex3;
            return 0;
        }
        
        if (((-line41).dot(line21) >= -1e-6) && ((-line41).dot(line21) <= 1e-6))
        {
            *pointA = vertex1;
            *pointB = vertex4;
            return 0;
        }
        
        if ((line21.dot(line13) >= -1e-6) && (line21.dot(line13) <= 1e-6) && (line21.dot(line24) >= -1e-6) && (line21.dot(line24) <= 1e-6))
        {
            *pointA = (vertex1 + vertex2) / 2.0;
            *pointB = (vertex3 + vertex4) / 2.0;
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

int RigidBodyContactEffector::PointInTriangle(Eigen::Vector3d supportPoint, Eigen::Vector3d triVertex0, Eigen::Vector3d triVertex1, Eigen::Vector3d triVertex2, Eigen::Vector3d *contactPoint, double *distance)
{
    Eigen::Vector3d u01 = triVertex1 - triVertex0;
    Eigen::Vector3d u02 = triVertex2 - triVertex0;
    Eigen::Vector3d u12 = triVertex2 - triVertex1;
    Eigen::Vector3d n = u01.cross(u02).normalized();
    Eigen::Vector3d w = supportPoint - triVertex0;
    double alpha = w.dot(n);
    Eigen::Vector3d vecToPlane = -alpha * n;
    
    *contactPoint = supportPoint + vecToPlane;
    *distance = abs(alpha);
    
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

