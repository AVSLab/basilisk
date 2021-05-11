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
void RigidBodyContactEffector::LoadMainBody(const char *objFile)
{
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;
    // - Use TinyOBJLoader
    bool ret = tinyobj::LoadObj(&this->mainBody.attrib, &this->mainBody.shapes, &materials, &err, objFile);
    // - Organize the vertices into a useful format
    for (int vertIt=0; vertIt<this->mainBody.attrib.vertices.size()/3; ++vertIt)
    {
        Eigen::Vector3d v(this->mainBody.attrib.vertices[3*vertIt + 0], this->mainBody.attrib.vertices[3*vertIt + 1], this->mainBody.attrib.vertices[3*vertIt + 2]);
        this->mainBody.vertices.push_back(v);
    }
    // - Orgazine the shape information into a half edge format
    this->mainBody.polyhedron = this->ComputeHalfEdge(this->mainBody.vertices, this->mainBody.shapes);
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
void RigidBodyContactEffector::AddOtherBody(const char *objFile, Message<SpicePlanetStateMsgPayload> *planetSpiceMsg, double boundingRadius, double coefRestitution, double coefFriction)
{
    geometry externalBody;
    externalBody.boundingRadius = boundingRadius;
    externalBody.coefRestitution = coefRestitution;
    externalBody.coefFriction = coefFriction;
    externalBody.planetInMsg = planetSpiceMsg->addSubscriber();
//    externalBody.modelTag = modelTag;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;
    // - Use TinyOBJLoader
    bool ret = tinyobj::LoadObj(&externalBody.attrib, &externalBody.shapes, &materials, &err, objFile);
    // - Organize the vertices into a useful format
    for (int vertIt=0; vertIt<externalBody.attrib.vertices.size()/3; ++vertIt)
    {
        Eigen::Vector3d v(externalBody.attrib.vertices[3*vertIt + 0], externalBody.attrib.vertices[3*vertIt + 1], externalBody.attrib.vertices[3*vertIt + 2]);
        externalBody.vertices.push_back(v);
    }
    // - Orgazine the shape information into a half edge format
    externalBody.polyhedron = this->ComputeHalfEdge(externalBody.vertices, externalBody.shapes);
    // - Add this body to the list of all external bodies
    this->externalBodies.push_back(externalBody);
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
    polyhedron.resize(shapes.size());
    int indexOffset;
    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d faceNormal;
    Eigen::Vector3d preCentroid;
    double shapeVolume;
    std::vector<int> tempTriangle;
    std::vector<int> searchEdge(2,0);
    
    for (int shapeIt=0; shapeIt<shapes.size(); ++shapeIt)
    {
        indexOffset = 0;
        preCentroid << 0.0, 0.0, 0.0;
        shapeVolume = 0;
        for (int faceIt=0; faceIt<shapes[shapeIt].mesh.num_face_vertices.size(); ++faceIt)
        {
            tempTriangle.clear();
            v1 =  vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index] - vertices[shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index];
            v2 = vertices[shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index] - vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index];
            faceNormal = v1.cross(v2);
            faceNormal.normalize();
            polyhedron[shapeIt].faceNormals.push_back( faceNormal);
            tempTriangle.push_back(shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index);
            tempTriangle.push_back(shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index);
            tempTriangle.push_back(shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index);
            polyhedron[shapeIt].faceTriangles.push_back(tempTriangle);
            
            shapeVolume += (1/6) * vertices[shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index].dot(polyhedron[shapeIt].faceNormals[faceIt]);
            
            preCentroid[0] += (1/24) * polyhedron[shapeIt].faceNormals[faceIt][0] * (pow((vertices[shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index][0] + vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index][0]),2) + pow((vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index][0] + vertices[shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index][0]),2) + pow((vertices[shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index][0] + vertices[shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index][0]),2));
            
            preCentroid[1] += (1/24) * polyhedron[shapeIt].faceNormals[faceIt][1] * (pow((vertices[shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index][1] + vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index][1]),2) + pow((vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index][1] + vertices[shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index][1]),2) + pow((vertices[shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index][1] + vertices[shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index][1]),2));
            
            preCentroid[2] += (1/24) * polyhedron[shapeIt].faceNormals[faceIt][2] * (pow((vertices[shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index][2] + vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index][2]),2) + pow((vertices[shapes[shapeIt].mesh.indices[indexOffset+1].vertex_index][2] + vertices[shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index][2]),2) + pow((vertices[shapes[shapeIt].mesh.indices[indexOffset+2].vertex_index][2] + vertices[shapes[shapeIt].mesh.indices[indexOffset+0].vertex_index][2]),2));
            
            for ( int inx=0; inx<shapes[shapeIt].mesh.num_face_vertices[faceIt]-1; ++inx)
            {
                std::vector<int> edgeGroup = {shapes[shapeIt].mesh.indices[indexOffset+inx].vertex_index, shapes[shapeIt].mesh.indices[indexOffset+inx+1].vertex_index};
                polyhedron[shapeIt].edgeIndices.push_back(edgeGroup);
                polyhedron[shapeIt].faceIndices.push_back(faceIt);
            }
            std::vector<int> edgeGroup = { shapes[shapeIt].mesh.indices[indexOffset+shapes[shapeIt].mesh.num_face_vertices[faceIt]-1].vertex_index, shapes[shapeIt].mesh.indices[indexOffset].vertex_index};
            polyhedron[shapeIt].edgeIndices.push_back(edgeGroup);
            polyhedron[shapeIt].faceIndices.push_back(faceIt);
            indexOffset = indexOffset + shapes[shapeIt].mesh.num_face_vertices[faceIt];
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
        
        if (shapeVolume != 0){
            polyhedron[shapeIt].centroid = (1 / (2 * shapeVolume)) * preCentroid;
        } else {
            polyhedron[shapeIt].centroid = preCentroid;
        }
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
void RigidBodyContactEffector::computeForceTorque(double integTime)
{
    this->forceExternal_N.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    
//     - Get all necessary state information
    this->mainBody.states.r_BN_N = (Eigen::Vector3d )this->mainBody.hubState.hubPosition->getState();
    this->mainBody.states.v_BN_N = (Eigen::Vector3d )this->mainBody.hubState.hubVelocity->getState();
    this->mainBody.states.m_SC = (double)(*this->mainBody.hubState.m_SC)(0,0);
    this->mainBody.states.c_B = (Eigen::Vector3d )*this->mainBody.hubState.c_B;
    this->mainBody.states.omega_BN_B = (Eigen::Vector3d )this->mainBody.hubState.hubOmega_BN_N->getState();
    this->mainBody.states.ISCPntB_B = *this->mainBody.hubState.ISCPntB_B;
    this->mainBody.states.sigma_BN = (Eigen::Vector3d )this->mainBody.hubState.hubSigma->getState();
    this->mainBody.states.dcm_NB = this->mainBody.states.sigma_BN.toRotationMatrix();
    this->mainBody.states.dcm_BN = this->mainBody.states.dcm_NB.transpose();
    
    int numStateVar = 9 + (4 * this->mainBody.collisionPoints.size());
    Eigen::VectorXd collisionStateVec = Eigen::VectorXd::Zero(numStateVar);
    Eigen::Vector3d v_CT_C = Eigen::Vector3d::Zero();
    Eigen::VectorXd compressionEnergy = Eigen::VectorXd::Zero(this->mainBody.collisionPoints.size());
    Eigen::VectorXd restitutionEnergy = Eigen::VectorXd::Zero(this->mainBody.collisionPoints.size());
    double totalSlip;
    Eigen::VectorXd k1;
    Eigen::VectorXd k2;
    Eigen::VectorXd k3;
    Eigen::VectorXd k4;

    
    // - Check if any new collisions will happen during this time step, and calculate the resulting forces and torques
    if (this->mainBody.collisionPoints.empty() == false)
    {
        if (this->mainBody.collisionPoints[0].empty() == false)
        {
            if ( integTime - this->currentSimSeconds < this->simTimeStep / 5.0)
            {
            // - Loop through every collision point
            for ( int bodyIt = 0; bodyIt < this->mainBody.collisionPoints.size(); ++bodyIt)
            {
                v_CT_C = this->mainBody.collisionPoints[bodyIt].back().dcm_CB * (this->mainBody.states.dcm_BN * (this->mainBody.states.v_BN_N - this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].states.v_BN_N) + eigenTilde(this->mainBody.states.omega_BN_B) * this->mainBody.collisionPoints[bodyIt].back().mainContactPoint - ((this->mainBody.states.dcm_BN * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].states.dcm_NB) * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].states.omegaTilde_BN_B *  (this->mainBody.states.dcm_BN * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].states.dcm_NB).transpose()) * this->mainBody.collisionPoints[bodyIt].back().mainContactPoint);
                
                collisionStateVec[bodyIt * 3] = sqrt(pow(v_CT_C[0], 2.0) + pow(v_CT_C[1], 2.0));
                collisionStateVec[bodyIt * 3 + 1] = atan2(v_CT_C[1], v_CT_C[0]);
                collisionStateVec[bodyIt * 3 + 2] = v_CT_C[2];
            }
            
            collisionStateVec.segment(this->mainBody.collisionPoints.size() * 3 + 3, 3) = this->mainBody.states.v_BN_N;
            collisionStateVec.segment(this->mainBody.collisionPoints.size() * 3 + 6, 3) = this->mainBody.states.omega_BN_B;
            
            do
            {
//                std::cout << collisionStateVec << '\n' << '\n';
                totalSlip = 0;
                // - Loop through every collision point
                for ( int bodyIt = 0; bodyIt < this->mainBody.collisionPoints.size(); ++bodyIt)
                {
                    totalSlip = totalSlip + collisionStateVec[bodyIt * 3];
                }
                
                k1 = this->collisionIntegrationStep * this->CollisionStateDerivative(collisionStateVec, totalSlip);
                k2 = this->collisionIntegrationStep * this->CollisionStateDerivative(collisionStateVec + 0.5 * k1, totalSlip);
                k3 = this->collisionIntegrationStep * this->CollisionStateDerivative(collisionStateVec + 0.5 * k2, totalSlip);
                k4 = this->collisionIntegrationStep * this->CollisionStateDerivative(collisionStateVec + k3, totalSlip);
                collisionStateVec = collisionStateVec + (1.0 / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
                
                
                for ( int bodyIt = 0; bodyIt < this->mainBody.collisionPoints.size(); ++bodyIt)
                {
                    if (collisionStateVec[bodyIt * 3] < this->slipTolerance)
                    {
                        this->mainBody.collisionPoints[bodyIt].back().slipHitZero = true;
                    }
                    if (collisionStateVec[this->mainBody.collisionPoints.size() * 3 + 9 + bodyIt] < compressionEnergy[bodyIt])
                    {
                        compressionEnergy[bodyIt] = collisionStateVec[ this->mainBody.collisionPoints.size() * 3 + 9 + bodyIt];
                        restitutionEnergy[bodyIt] = - pow(this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefRestitution, 2.0) * compressionEnergy[bodyIt];
                    }
                }
//                std::cout << collisionStateVec[this->mainBody.collisionPoints.size() * 3 + 9] << '\n' << '\n';
            } while ((collisionStateVec.segment( this->mainBody.collisionPoints.size() * 3 + 9, this->mainBody.collisionPoints.size()).array() < (compressionEnergy + restitutionEnergy).array()).any());
            
                std::cout << collisionStateVec.segment(this->mainBody.collisionPoints.size() * 3 + 3, 3) << '\n' << '\n';
//                std::cout << collisionStateVec.segment(this->mainBody.collisionPoints.size() * 3 + 6, 3) << '\n' << '\n';
            this->forceExternal_N = this->mainBody.states.m_SC * (collisionStateVec.segment(this->mainBody.collisionPoints.size() * 3 + 3, 3) - this->mainBody.states.v_BN_N) / (this->simTimeStep );
            
            this->torqueExternalPntB_B = this->mainBody.states.ISCPntB_B * ((collisionStateVec.segment(this->mainBody.collisionPoints.size() * 3 + 6, 3) - this->mainBody.states.omega_BN_B) / (this->simTimeStep));
            
                this->mainBody.collisionPoints[0].back().force_N = this->mainBody.states.m_SC * (collisionStateVec.segment(this->mainBody.collisionPoints.size() * 3 + 3, 3) - this->mainBody.states.v_BN_N) / (this->simTimeStep);
//
                this->mainBody.collisionPoints[0].back().torque_B =  this->mainBody.states.ISCPntB_B * ((collisionStateVec.segment(this->mainBody.collisionPoints.size() * 3 + 6, 3) - this->mainBody.states.omega_BN_B) / (this->simTimeStep));
//                this->mainBody.collisionPoints[0].back().force_N = collisionStateVec.segment(this->mainBody.collisionPoints.size() * 3 + 3, 3);
            
//                this->mainBody.collisionPoints[0].back().torque_B =  collisionStateVec.segment(this->mainBody.collisionPoints.size() * 3 + 6, 3);
            }else
            {
                this->forceExternal_N = this->mainBody.collisionPoints[0].back().force_N;
//
                this->torqueExternalPntB_B = this->mainBody.collisionPoints[0].back().torque_B;
//                this->forceExternal_N = this->mainBody.states.m_SC * (this->mainBody.collisionPoints[0].back().force_N - this->mainBody.states.v_BN_N) / (this->simTimeStep );

//                this->torqueExternalPntB_B = this->mainBody.states.ISCPntB_B * ((this->mainBody.collisionPoints[0].back().torque_B - this->mainBody.states.omega_BN_B) / (this->simTimeStep));
            }
        }
    }
    
    
}

Eigen::VectorXd RigidBodyContactEffector::CollisionStateDerivative( Eigen::VectorXd X_c, double totalSlip)
{
    Eigen::VectorXd Xdot_c = Eigen::VectorXd::Zero(X_c.size());
    Eigen::Vector3d dPcont;
    Eigen::Vector3d dPdp = Eigen::Vector3d::Zero();
    Eigen::Vector3d dv;
    double eta;
    
    // - Loop through every collision point
    for ( int bodyIt = 0; bodyIt < this->mainBody.collisionPoints.size(); ++bodyIt)
    {
        if ((totalSlip < this->slipTolerance) && (this->mainBody.collisionPoints[bodyIt].back().critCoeffFric <= this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction))
        {
            dPcont << - this->mainBody.collisionPoints[bodyIt].back().critCoeffFric * cos(this->mainBody.collisionPoints[bodyIt].back().critSlideDir), - this->mainBody.collisionPoints[bodyIt].back().critCoeffFric * sin(this->mainBody.collisionPoints[bodyIt].back().critSlideDir), 1.0;
            dPcont = this->mainBody.collisionPoints[bodyIt].back().dcm_CB.transpose() * dPcont;
            
            Xdot_c.segment(this->mainBody.collisionPoints.size() * 3 + 6, 3) = Xdot_c.segment(this->mainBody.collisionPoints.size() * 3 + 6, 3) + eigenTilde( this->mainBody.collisionPoints[bodyIt].back().mainContactPoint) * dPcont;
            
            dPcont = this->mainBody.states.dcm_BN.transpose() * dPcont;
            
            Xdot_c[this->mainBody.collisionPoints.size() * 3] = Xdot_c[this->mainBody.collisionPoints.size() * 3] + dPcont[0];
            Xdot_c[this->mainBody.collisionPoints.size() * 3 + 1] = Xdot_c[this->mainBody.collisionPoints.size() * 3 + 1] + dPcont[1];
            Xdot_c[this->mainBody.collisionPoints.size() * 3 + 2] = Xdot_c[this->mainBody.collisionPoints.size() * 3 + 2] + dPcont[2];
//            std::cout << Xdot_c << '\n';
        } else
        {
            eta = X_c[bodyIt * 3] / this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction;
            
            if ((this->mainBody.collisionPoints[bodyIt].back().slipHitZero) && (this->mainBody.collisionPoints[bodyIt].back().critCoeffFric > this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction))
            {
                dPcont << -this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * cos(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir), -this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * sin(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir), 1.0;
            }else
            {
                dPcont << -this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * cos(X_c[bodyIt * 3 + 1]), -this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * sin(X_c[bodyIt * 3 + 1]), 1.0;
            }
            dPcont = this->mainBody.collisionPoints[bodyIt].back().dcm_CB.transpose() * dPcont;
            
            Xdot_c.segment(this->mainBody.collisionPoints.size() * 3 + 6, 3) = Xdot_c.segment(this->mainBody.collisionPoints.size() * 3 + 6, 3) + eigenTilde( this->mainBody.collisionPoints[bodyIt].back().mainContactPoint) * (eta * dPcont);
            
            dPcont = this->mainBody.states.dcm_BN.transpose() * dPcont;
            
            Xdot_c[this->mainBody.collisionPoints.size() * 3] = Xdot_c[this->mainBody.collisionPoints.size() * 3] + (eta * dPcont[0]);
            Xdot_c[this->mainBody.collisionPoints.size() * 3 + 1] = Xdot_c[this->mainBody.collisionPoints.size() * 3 + 1] + (eta * dPcont[1]);
            Xdot_c[this->mainBody.collisionPoints.size() * 3 + 2] = Xdot_c[this->mainBody.collisionPoints.size() * 3 + 2] + (eta * dPcont[2]);
            
            dPdp = dPdp + dPcont;
        }
    }
    for ( int bodyIt = 0; bodyIt < this->mainBody.collisionPoints.size(); ++bodyIt)
    {
        
        if (totalSlip < this->slipTolerance)
        {
            dv << this->mainBody.collisionPoints[bodyIt].back().dcm_CB * this->mainBody.collisionPoints[bodyIt].back().MSCPntC_B * this->mainBody.states.dcm_BN * Xdot_c.segment(this->mainBody.collisionPoints.size() * 3, 3);
//            dv[0] = ceil(dv[0] * pow(10.0, 14)) / pow(10.0, 14);
//            dv[1] = ceil(dv[1] * pow(10.0, 14)) / pow(10.0, 14);
            Xdot_c[bodyIt * 3] = cos(X_c[bodyIt * 3 + 1]) * dv[0] + sin(X_c[bodyIt * 3 + 1]) * dv[1];
            Xdot_c[bodyIt * 3 + 1] = (cos(X_c[bodyIt * 3 + 1]) / X_c[bodyIt * 3]) * dv[1] - (sin(X_c[bodyIt * 3 + 1]) / X_c[bodyIt * 3]) * dv[0];
            if (std::isnan(Xdot_c[bodyIt * 3 + 1]))
            {
                Xdot_c[bodyIt * 3 + 1] = 0;
            }
            Xdot_c[bodyIt * 3 + 2] = dv[2];
            Xdot_c[this->mainBody.collisionPoints.size() * 3 + 9 + bodyIt] = X_c[bodyIt * 3 + 2];
//            std::cout << Xdot_c << '\n';
        }else
        {
            dv << this->mainBody.collisionPoints[bodyIt].back().dcm_CB * this->mainBody.collisionPoints[bodyIt].back().MSCPntC_B * this->mainBody.states.dcm_BN * dPdp;
            Xdot_c[bodyIt * 3] = (X_c[bodyIt * 3] / this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction) * (cos(X_c[bodyIt * 3 + 1]) * dv[0] + sin(X_c[bodyIt * 3 + 1]) * dv[1]);
            Xdot_c[bodyIt * 3 + 1] = (cos(X_c[bodyIt * 3 + 1]) * dv[1] - sin(X_c[bodyIt * 3 + 1]) * dv[0]) / this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction;
            Xdot_c[bodyIt * 3 + 2] = (X_c[bodyIt * 3] / this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction) * dv[2];
            Xdot_c[this->mainBody.collisionPoints.size() * 3 + 9 + bodyIt] = (X_c[bodyIt * 3] / this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction) * X_c[bodyIt * 3 + 2];
        }
    }
    
    Xdot_c.segment(this->mainBody.collisionPoints.size() * 3 + 3, 3) = (1.0 / this->mainBody.states.m_SC) * Xdot_c.segment(this->mainBody.collisionPoints.size() * 3, 3);
    Xdot_c.segment(this->mainBody.collisionPoints.size() * 3 + 6, 3) = this->mainBody.states.ISCPntB_B_inv * Xdot_c.segment(this->mainBody.collisionPoints.size() * 3 + 6, 3);
//    std::cout << Xdot_c << '\n';
    
    return Xdot_c;
}


void RigidBodyContactEffector::CalcCollisionProps()
{
    Eigen::Vector3d cHat_1;
    Eigen::Vector3d cHat_2;
    Eigen::Vector3d cHat_3;
    Eigen::Vector3d zDirection;
    zDirection << 0, 0, 1;
    Eigen::Matrix3d MSCPntC_C;
    Eigen::Matrix3d dcm_CN;
    double slipReverseDirp1;
    double h;
    double hp;
    double hpp;
    double hppp;
    // - Check if any new collisions will happen during this time step, and calculate the resulting forces and torques
    if (this->mainBody.collisionPoints.empty() == false)
    {
        if (this->mainBody.collisionPoints[0].empty() == false)
        {
            // - Loop through every collision point
            for ( int bodyIt = 0; bodyIt < this->mainBody.collisionPoints.size(); ++bodyIt)
            {
                cHat_3 = - this->mainBody.collisionPoints[bodyIt].back().contactNormal;
                cHat_1 = cHat_3.cross( this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].states.dcm_NB * zDirection);
                cHat_1.normalize();
                cHat_2 = cHat_3.cross(cHat_1);
                cHat_2.normalize();
                dcm_CN << cHat_1[0], cHat_1[1], cHat_1[2], cHat_2[0], cHat_2[1], cHat_2[2], cHat_3[0], cHat_3[1], cHat_3[2];
                this->mainBody.collisionPoints[bodyIt].back().dcm_CB = dcm_CN * this->mainBody.states.dcm_NB;
                this->mainBody.collisionPoints[bodyIt].back().MSCPntC_B = ((1.0 / this->mainBody.states.m_SC) * Eigen::Matrix3d::Identity()) - (eigenTilde(this->mainBody.collisionPoints[bodyIt].back().mainContactPoint) * this->mainBody.states.ISCPntB_B_inv * eigenTilde(this->mainBody.collisionPoints[bodyIt].back().mainContactPoint));
                MSCPntC_C = (this->mainBody.collisionPoints[bodyIt].back().dcm_CB) * this->mainBody.collisionPoints[bodyIt].back().MSCPntC_B * (this->mainBody.collisionPoints[bodyIt].back().dcm_CB).transpose();
                this->mainBody.collisionPoints[bodyIt].back().critSlideDir = atan2((MSCPntC_C(0,0) * MSCPntC_C(1,2) - MSCPntC_C(1,0) * MSCPntC_C(0,2)) , (MSCPntC_C(1,1) * MSCPntC_C(0,2) - MSCPntC_C(0,1) * MSCPntC_C(1,2)));
                this->mainBody.collisionPoints[bodyIt].back().critCoeffFric = sqrt((pow(MSCPntC_C(0,1) * MSCPntC_C(1,2) - MSCPntC_C(1,1) * MSCPntC_C(0,2), 2.0) + pow(MSCPntC_C(1,0) * MSCPntC_C(0,2) - MSCPntC_C(0,0) * MSCPntC_C(1,2), 2.0)) / pow(MSCPntC_C(0,0) * MSCPntC_C(1,1) - MSCPntC_C(0,1) * MSCPntC_C(1,0), 2.0));
                
                if (this->mainBody.collisionPoints[bodyIt].back().critCoeffFric > this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction)
                {
                    slipReverseDirp1 = this->mainBody.collisionPoints[bodyIt].back().critSlideDir;
                    do
                    {
                        this->mainBody.collisionPoints[bodyIt].back().slipReverseDir = slipReverseDirp1;
                        h = -MSCPntC_C(0,2) * sin(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + MSCPntC_C(1,2) * cos(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + 0.5 *  this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * (MSCPntC_C(0,0) - MSCPntC_C(1,1)) * sin(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * cos(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir);
                        hp = -MSCPntC_C(0,2) * cos(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - MSCPntC_C(1,2) * sin(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * (MSCPntC_C(0,0) - MSCPntC_C(1,1)) * cos(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + 2.0 * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * sin(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir);
                        
                        hpp = MSCPntC_C(0,2) * sin(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - MSCPntC_C(1,2) * cos(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - 2.0 * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * (MSCPntC_C(0,0) - MSCPntC_C(1,1)) * sin(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + 4.0 * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * cos(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir);
                        
                        hppp = MSCPntC_C(0,2) * cos(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) + MSCPntC_C(1,2) * sin(this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - 4.0 * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * (MSCPntC_C(0,0) - MSCPntC_C(1,1)) * cos(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) - 8.0 * this->externalBodies[ this->mainBody.collisionPoints[ bodyIt].back().otherBodyIndex].coefFriction * sin(2.0 * this->mainBody.collisionPoints[bodyIt].back().slipReverseDir);
                        
                        slipReverseDirp1 = this->mainBody.collisionPoints[bodyIt].back().slipReverseDir - (6.0 * h * pow(hpp, 2.0) -3.0 * pow(h, 2.0) * hpp) / (6.0 * pow(hp, 3.0) - 6.0 * h * hp * hpp + pow(h, 2.0) * hppp);
                    }
                    while (abs(slipReverseDirp1 - this->mainBody.collisionPoints[bodyIt].back().slipReverseDir) > 1e-6);
                    this->mainBody.collisionPoints[bodyIt].back().slipReverseDir = slipReverseDirp1;
                }
            }
            
        }
    }
    return;
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
    this->mainBody.states.r_BN_N = (Eigen::Vector3d )this->mainBody.hubState.hubPosition->getState();
    this->mainBody.states.v_BN_N = (Eigen::Vector3d )this->mainBody.hubState.hubVelocity->getState();
    this->mainBody.states.m_SC = (double)(*this->mainBody.hubState.m_SC)(0,0);
    this->mainBody.states.c_B = (Eigen::Vector3d )*this->mainBody.hubState.c_B;
    this->mainBody.states.omega_BN_B = (Eigen::Vector3d )this->mainBody.hubState.hubOmega_BN_N->getState();
    this->mainBody.states.ISCPntB_B = *this->mainBody.hubState.ISCPntB_B;
    this->mainBody.states.ISCPntB_B_inv = this->mainBody.states.ISCPntB_B.inverse();
    this->mainBody.states.sigma_BN = (Eigen::Vector3d )this->mainBody.hubState.hubSigma->getState();
    this->mainBody.states.dcm_NB = this->mainBody.states.sigma_BN.toRotationMatrix();
    this->mainBody.states.dcm_BN = this->mainBody.states.dcm_NB.transpose();
    this->mainBody.states.omegaTilde_BN_B = eigenTilde(this->mainBody.states.omega_BN_B);
    
    this->ReadInputs();
    this->ExtractFromBuffer();
    this->mainBody.collisionPoints.clear();
    this->mainBody.penetrationData.clear();
    this->CheckBoundingSphere();
    this->overlapFace1 = true;
    this->overlapFace2 = true;
    this->overlapEdge = true;
    this->isOverlap = false;
    
    for ( std::vector<int>::iterator bodyIt = this->closeBodies.begin(); bodyIt != this->closeBodies.end(); ++bodyIt){
        this->isOverlap = this->Overlap(this->externalBodies[*bodyIt], *bodyIt);
    }
    this->CalcCollisionProps();
    
    return;
}

/*! This method is used to read the messages pertaining to all external bodies.
@return void
*/
void RigidBodyContactEffector::ReadInputs()
{

    for (int bodyIt=0; bodyIt<this->externalBodies.size(); ++bodyIt)
    {
        this->externalBodies[bodyIt].plMsg = this->externalBodies[bodyIt].planetInMsg();
    }

    return;
}

/*! This method extracts all important information for each external body.
@return void
*/
void RigidBodyContactEffector::ExtractFromBuffer()
{
    Eigen::Matrix3d dcm_BN_dot;
    for( int bodyIt=0; bodyIt<this->externalBodies.size(); ++bodyIt)
    {
        this->externalBodies[bodyIt].states.r_BN_N = cArray2EigenVector3d(this->externalBodies[bodyIt].plMsg.PositionVector);
        this->externalBodies[bodyIt].states.v_BN_N = cArray2EigenVector3d(this->externalBodies[bodyIt].plMsg.VelocityVector);
        this->externalBodies[bodyIt].states.dcm_BN = cArray2EigenMatrix3d(*this->externalBodies[bodyIt].plMsg.J20002Pfix);
        dcm_BN_dot = cArray2EigenMatrix3d(*this->externalBodies[bodyIt].plMsg.J20002Pfix_dot);
        this->externalBodies[bodyIt].states.omegaTilde_BN_B = - dcm_BN_dot * this->externalBodies[bodyIt].states.dcm_BN.transpose();
        this->externalBodies[bodyIt].states.sigma_BN = eigenC2MRP(this->externalBodies[bodyIt].states.dcm_BN);
        this->externalBodies[bodyIt].states.dcm_NB = this->externalBodies[bodyIt].states.dcm_BN.transpose();
//        double effectiveMass = (this->mainBody.states.m_SC * 78e9) / (this->mainBody.states.m_SC + 78e9);
        
        // - Calculate the spring constant and damping constant from the Coefficent of Restitution
        this->externalBodies[bodyIt].springConstant = (  ( this->mainBody.states.m_SC) ) / pow(4.0 * this->simTimeStep, 2) * ( pow(M_PI, 2) + pow(log(this->externalBodies[bodyIt].coefRestitution), 2));
        this->externalBodies[bodyIt].dampingConstant = - (2 * this->mainBody.states.m_SC) / ( this->simTimeStep) * log(this->externalBodies[bodyIt].coefRestitution);
    }
    return;
}



/*! This method checks if the primary body is within the bounding sphere of any external bodies.
@return void
*/
void RigidBodyContactEffector::CheckBoundingSphere()
{
    this->closeBodies.clear();
    Eigen::Vector3d bodyDifference;
    double bodyDistance;
    
    for(int bodyIt=0; bodyIt < this->externalBodies.size(); ++bodyIt)
    {
        bodyDifference = this->mainBody.states.r_BN_N - this->externalBodies[bodyIt].states.r_BN_N;
        bodyDistance = bodyDifference.norm();
        
        if (bodyDistance < (this->mainBody.boundingRadius + this->externalBodies[bodyIt].boundingRadius))
            this->closeBodies.push_back(bodyIt);
    }
    return;
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
        for (int otherShapeIt=0; otherShapeIt<foriegnBody.polyhedron.size(); ++otherShapeIt)
        {
            currentCollisionList.clear();
            newCollisionPoint.area = 0;
            
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
                            currentCollisionList.push_back(newCollisionPoint);
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
                            newCollisionPoint.contactNormal = foriegnBody.states.dcm_NB * foriegnBody.polyhedron[otherShapeIt].faceNormals[otherFaceIt];
                            if ((this->mainBody.states.dcm_NB * newCollisionPoint.mainContactPoint - faceContactPoint_N).dot(newCollisionPoint.contactNormal) > 0.0)
                            {
                            newCollisionPoint.contactNormal = -newCollisionPoint.contactNormal.normalized();
                            }
                            currentCollisionList.push_back(newCollisionPoint);
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
                                newCollisionPoint.contactNormal = (foriegnBody.states.dcm_NB * (foriegnBody.vertices[edgePair[1][1]] - foriegnBody.vertices[edgePair[1][0]])).cross(this->mainBody.states.dcm_NB * (this->mainBody.vertices[edgePair[0][1]] - this->mainBody.vertices[edgePair[0][0]]));
                                newCollisionPoint.contactNormal.normalize();
                                if ( newCollisionPoint.contactNormal.dot(this->mainBody.states.dcm_NB * (this->mainBody.vertices[edgePair[0][0]] - this->mainBody.polyhedron[mainShapeIt].centroid)) < 0){
                                    newCollisionPoint.contactNormal = -newCollisionPoint.contactNormal;
                                }
                            }else
                            {
                                newCollisionPoint.contactNormal = (foriegnBody.states.dcm_BN * foriegnBody.polyhedron[otherShapeIt].centroid + foriegnBody.states.r_BN_N) - (this->mainBody.states.dcm_NB * this->mainBody.polyhedron[mainShapeIt].centroid + this->mainBody.states.r_BN_N);
                                newCollisionPoint.contactNormal.normalize();
                                
                            }
                            currentCollisionList.push_back(newCollisionPoint);
                        }
                    }
                }
                
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
                this->mainBody.collisionPoints.push_back(currentCollisionList);
            
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
                    
                    this->mainBody.penetrationData.push_back(newPenetration);
                    continue;
                }
                
                if (((faceB.separation > faceA.separation) || (inTriangleA == -1)) && ((faceB.separation > edge.separation) || (colinear < 0)) && (inTriangleB > -1))
                {
                    newPenetration.otherContactPoint = foriegnBody.states.dcm_BN * (faceBContactPoint_N - foriegnBody.states.r_BN_N);
                    newPenetration.springLine_N = foriegnBody.states.dcm_NB * -foriegnBody.polyhedron[otherShapeIt].faceNormals[faceB.faceIndex];
                    newPenetration.contactCase = 1;
                    newPenetration.faceData = faceB;
                   
                    this->mainBody.penetrationData.push_back(newPenetration);
                    continue;
                }
                
                if (((edge.separation > faceB.separation) || (inTriangleB == -1)) && ((edge.separation > faceA.separation) || (inTriangleA == -1)) && (colinear > -1))
                {
                    newPenetration.mainContactPoint = this->mainBody.states.dcm_BN * (mainEdgeContact_N - this->mainBody.states.r_BN_N);
                    newPenetration.otherContactPoint = foriegnBody.states.dcm_BN * (otherEdgeContact_N - foriegnBody.states.r_BN_N);
                    newPenetration.springLine_N =  (mainEdgeContact_N - otherEdgeContact_N).normalized();
                    newPenetration.contactCase = 2;
                    newPenetration.edgeData = edge;
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
            *pointA = (*pointA + vertex2) / 2;
            *pointB = vertex3 + ((vertex2 - vertex3).dot(line43) / line43.dot(line43)) * line43;
            *pointB = (*pointB + vertex3) / 2;
            return 0;
        }
        line24 = vertex2 - vertex4;
        line14 = vertex1 - vertex4;
        dotRes = (line14.normalized()).dot(line24.normalized());
        if (dotRes < 0)
        {
            *pointA = vertex1 + ((vertex4 - vertex1).dot(line21) / line21.dot(line21)) * line21;
            *pointA = (*pointA + vertex1) / 2;
            *pointB = vertex3 + ((vertex1 - vertex3).dot(line43) / line43.dot(line43)) * line43;
            *pointB = (*pointB + vertex4) / 2;
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
        if ( ((triVertex0 - *contactPoint).cross(triVertex1 - *contactPoint)).dot(n) < 0 )
        {
            return -1;
        }
        if ( ((triVertex0 - *contactPoint).cross(triVertex1 - *contactPoint)).dot(n) > 0 )
        {
            return 1;
        }
        if ( ((triVertex0 - *contactPoint).cross(triVertex1 - *contactPoint)).dot(n) == 0 )
        {
            return 0;
        }
    }
    if ( (f3 <= 0) && (f2 > 0))
    {
        if ( ((triVertex1 - *contactPoint).cross(triVertex2 - *contactPoint)).dot(n) < 0 )
        {
            return -1;
        }
        if ( ((triVertex1 - *contactPoint).cross(triVertex2 - *contactPoint)).dot(n) > 0 )
        {
            return 1;
        }
        if ( ((triVertex1 - *contactPoint).cross(triVertex2 - *contactPoint)).dot(n) == 0 )
        {
            return 0;
        }
    }
    if ( (f1 <= 0) && (f3 > 0))
    {
        if ( ((triVertex2 - *contactPoint).cross(triVertex0 - *contactPoint)).dot(n) < 0 )
        {
            return -1;
        }
        if ( ((triVertex2 - *contactPoint).cross(triVertex0 - *contactPoint)).dot(n) > 0 )
        {
            return 1;
        }
        if ( ((triVertex2 - *contactPoint).cross(triVertex0 - *contactPoint)).dot(n) == 0 )
        {
            return 0;
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

