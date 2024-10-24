/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "polyhedralGravityModel.h"
#include "simulation/dynamics/_GeneralModuleFiles/gravityEffector.h"

namespace {
// Computes the facet solid angle.
inline double computeSolidangle(const Eigen::Vector3d ri, const Eigen::Vector3d rj, const Eigen::Vector3d rk)
{
    // Computes solid angle
    double wy, wx, wf;
    wy = ri.transpose()*rj.cross(rk);
    wx = ri.norm()*rj.norm()*rk.norm()
         + ri.norm()*rj.transpose()*rk
         + rj.norm()*rk.transpose()*ri
         + rk.norm()*ri.transpose()*rj;
    wf = 2*atan2(wy, wx);

    return wf;
}
}

std::optional<std::string> PolyhedralGravityModel::initializeParameters()
{
    // If data hasn't been loaded, quit and return failure
    if (this->xyzVertex.size() == 0 || this->orderFacet.size() == 0)
    {
        return "Could not initialize polyhedral data: the vertex (xyzVertex) or facet (orderFacet) "
        "were not provided.";;
    }

    // Initializes facet parameters
    this->initializeFacets();

    // Initializes edges parameters
    this->initializeEdges();

    return {};
}

std::optional<std::string> PolyhedralGravityModel::initializeParameters(const GravBodyData& body)
{
    this->muBody = body.mu;
    return this->initializeParameters();
}

Eigen::Vector3d
PolyhedralGravityModel::computeField(const Eigen::Vector3d& pos_BP_P) const
{
    const size_t nFacet = this->orderFacet.rows();
    const size_t nEdge = int(3*nFacet/2);

    // For the facet in loop: declare vertex indexes
    // and relative positions w.r.t. spacecraft
    Eigen::Vector3i v;
    int i, j, k;
    Eigen::Vector3d ri, rj, rk;

    // For the facet-edge in loop: declare their
    // relative positions w.r.t. spacecraft
    // and their dyad products
    Eigen::Vector3d re, rf;
    Eigen::Matrix3d Ee, Ff;

    // Declare edge lengths, wire potential
    // and facet solid angle
    double a, b, e;
    double Le, wf;

    // Preallocate edges and facets
    // contribution
    Eigen::Vector3d dUe, dUf;
    dUe.setZero(3);
    dUf.setZero(3);

    // Loop through edges
    for (unsigned int n = 0; n < nEdge; n++){
        // Get edge dyad matrix
        Ee = this->EeDyad[n];

        // Compute vector from spacecraft to an edge point
        re = this->xyzVertex.row(this->edgeVertex(n,0)).transpose()
             - pos_BP_P;

        // Compute edge wire potential
        a = (this->xyzVertex.row(this->edgeVertex(n,0)).transpose()
             - pos_BP_P).norm();
        b = (this->xyzVertex.row(this->edgeVertex(n,1)).transpose()
             - pos_BP_P).norm();
        e = this->edgeLength(n);
        Le = log((a+b+e) / (a+b-e));

        // Add current edge contribution
        dUe += Ee*re*Le;

        // Loop through facets
        if (n < nFacet){
            // Get facet dyad matrix
            Ff = this->FfDyad[n];

            // Obtain vertex indexes of the facet
            v = this->orderFacet.row(n);
            i = v[0] - 1;
            j = v[1] - 1;
            k = v[2] - 1;

            // Compute facet vertexes relative position w.r.t. spacecraft
            ri = this->xyzVertex.row(i).transpose() - pos_BP_P;
            rj = this->xyzVertex.row(j).transpose() - pos_BP_P;
            rk = this->xyzVertex.row(k).transpose() - pos_BP_P;

            // Compute facet solid angle
            wf = computeSolidangle(ri, rj, rk);

            // Pick one vector from the facet to the spacecraft
            rf = ri;

            // Add facet contribution
            dUf += Ff*rf*wf;
        }
    }

    // Compute gravity acceleration
    Eigen::Vector3d acc;
    acc = (this->muBody/this->volPoly)*(-dUe + dUf);

    return acc;
}

double
PolyhedralGravityModel::computePotentialEnergy(const Eigen::Vector3d& pos_BP_P) const
{
    const size_t nFacet = this->orderFacet.rows();
    const size_t nEdge = int(3*nFacet/2);

    // For the facet in loop: declare vertex indexes
    // and positions
    Eigen::Vector3i v;
    int i, j, k;
    Eigen::Vector3d ri, rj, rk;

    // For the facet-edge in loop: declare their
    // relative positions w.r.t. spacecraft
    // and their dyad products
    Eigen::Vector3d re, rf;
    Eigen::Matrix3d Ee, Ff;

    // Declare edge lengths, wire potential
    // and facet solid angle
    double a, b, e;
    double Le, wf;

    // Preallocate edges and facets
    // contribution
    double Ue, Uf;
    Ue = 0;
    Uf = 0;

    // Loop through edges
    for (unsigned int n = 0; n < nEdge; n++){
        // Get edge dyad matrix
        Ee = this->EeDyad[n];

        // Compute vector from spacecraft to an edge point
        re = this->xyzVertex.row(this->edgeVertex(n,0)).transpose()
             - pos_BP_P;

        // Compute edge wire potential
        a = (this->xyzVertex.row(this->edgeVertex(n,0)).transpose()
             - pos_BP_P).norm();
        b = (this->xyzVertex.row(this->edgeVertex(n,1)).transpose()
             - pos_BP_P).norm();
        e = this->edgeLength(n);
        Le = log((a+b+e) / (a+b-e));

        // Add current edge contribution
        Ue += re.dot(Ee*re)*Le;

        // Loop through facets
        if (n < nFacet){
            // Get facet dyad matrix
            Ff = this->FfDyad[n];

            // Obtain vertex indexes of the facet
            v = this->orderFacet.row(n);
            i = v[0] - 1;
            j = v[1] - 1;
            k = v[2] - 1;

            // Compute facet vertexes relative position w.r.t. spacecraft
            ri = this->xyzVertex.row(i).transpose() - pos_BP_P;
            rj = this->xyzVertex.row(j).transpose() - pos_BP_P;
            rk = this->xyzVertex.row(k).transpose() - pos_BP_P;

            // Compute facet solid angle
            wf = computeSolidangle(ri, rj, rk);

            // Pick one vector from the facet to the spacecraft
            rf = ri;

            // Add facet contribution
            Uf += rf.dot(Ff*rf)*wf;
        }
    }

    /* Compute gravity potential */
    double U;
    U = (this->muBody/this->volPoly) * (Ue - Uf)/2;

    return U;
}


void PolyhedralGravityModel::initializeFacets()
{
    const size_t nFacet = this->orderFacet.rows();
    const size_t nEdge = int(3*nFacet/2);

    // Preallocate facet normal and center
    Eigen::Vector3d nf;
    this->normalFacet.setZero(nFacet, 3);
    this->xyzFacet.setZero(nFacet, 3);

    // Preallocate facet and vertex indexes
    // per each edge
    this->edgeFacet.setZero(nEdge, 2);
    this->edgeVertex.setZero(nEdge, 2);

    // Initialize polyhedron volume
    this->volPoly = 0.0;

    // For the facet in loop: declare vertex indexes,
    // positions and lines between vertexes
    Eigen::Vector3i v;
    int i, j, k;
    Eigen::Vector3d xyz1, xyz2, xyz3, e21, e32;

    // Declare all edges vertexes for facet in loop
    Eigen::MatrixXi edgeCurrentFacet;
    edgeCurrentFacet.setZero(3, 2);

    // Declare flag telling if an edge has been already
    // stored. Initialize non-repeated edges index
    bool isEdgeRepeat;
    int idxEdge = 0;

    // Loop through each facet
    for (unsigned int m = 0; m < nFacet; m++)
    {
        // Obtain vertex indexes of the facet
        v = this->orderFacet.row(m);
        i = v[0] - 1;
        j = v[1] - 1;
        k = v[2] - 1;

        // Extract vertexes position
        xyz1 = this->xyzVertex.row(i);
        xyz2 = this->xyzVertex.row(j);
        xyz3 = this->xyzVertex.row(k);

        // Compute facet normal and center
        e21 = xyz2 - xyz1;
        e32 = xyz3 - xyz2;
        nf = e21.cross(e32) / e21.cross(e32).norm();
        this->normalFacet.row(m) = nf.transpose();
        this->xyzFacet.row(m) = (xyz3 + xyz2 + xyz1) / 3;

        // Add facet contribution to volume
        this->volPoly += abs(xyz1.cross(xyz2).transpose()*xyz3)/6;

        // Store facet dyad product
        this->FfDyad.push_back(nf*nf.transpose());

        // Get edge vertexes for this facet
        edgeCurrentFacet << i, j,
            j, k,
            k, i;

        // Loop through each facet edge
        for (unsigned int n = 0; n < 3; n++){
            // Add edge if non-repeated */
            isEdgeRepeat = this->addEdge(edgeCurrentFacet.row(n),
                                         idxEdge, m);

            // If not repeated, advance edge index
            if (isEdgeRepeat == false){
                idxEdge += 1;
            }
        }
    }
}

void PolyhedralGravityModel::initializeEdges()
{
    const size_t nFacet = this->orderFacet.rows();
    const size_t nEdge = int(3*nFacet/2);

    // Declare shared-facet normals, edge line
    // and outward normals to edge-facet
    // (Figure 7 Werner&Scheeres 1996)
    Eigen::Vector3d nFA, nFB, edgeLine, n12, n21;

    // Preallocate edges length
    this->edgeLength.setZero(nEdge);

    // Loop through edges
    for (unsigned int n = 0; n < nEdge; n++){
        // Obtain normal of facets sharing the edge
        nFA = this->normalFacet.row(this->edgeFacet(n, 0)).transpose();
        nFB = this->normalFacet.row(this->edgeFacet(n, 1)).transpose();

        // Compute the edge line and length
        edgeLine = (this->xyzVertex.row(this->edgeVertex(n, 1))
                    - this->xyzVertex.row(this->edgeVertex(n, 0))).transpose();
        this->edgeLength(n) = edgeLine.norm();
        edgeLine /= this->edgeLength(n);

        // Compute outward normals to edge-facet
        n12 = edgeLine.cross(nFA);
        n21 = -edgeLine.cross(nFB);

        // Store edge dyad product
        this->EeDyad.push_back(nFA*n12.transpose() + nFB*n21.transpose());
    }
}

bool PolyhedralGravityModel::addEdge(Eigen::Vector2i edge, int idxEdge, int idxFacet)
{
    // Flag telling if an edge is already stored
    bool isEdgeRepeat = false;

    // Loop through previously stored edges
    for (int i = 0; i < idxEdge; i++){
        // Check if the edge is already stored
        if ((this->edgeVertex(i, 0) == edge(0) && this->edgeVertex(i, 1) == edge(1))
            || (this->edgeVertex(i, 1) == edge(0) && this->edgeVertex(i, 0) == edge(1))){
            // If edge is repeated set flag to true and just store the other facet
            isEdgeRepeat = true;
            this->edgeFacet(i, 1) = idxFacet;

            return isEdgeRepeat;
        }
    }

    // If edge is not stored, store edge vertexes and facet */
    this->edgeVertex.row(idxEdge) = edge;
    this->edgeFacet(idxEdge, 0) = idxFacet;

    return isEdgeRepeat;
}
