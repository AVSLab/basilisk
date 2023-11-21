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

std::optional<std::string> PolyhedralGravityModel::initializeParameters()
{
    // If data hasn't been loaded, quit and return failure
    if (this->xyzVertex.size() == 0 || this->orderFacet.size() == 0) {
        return "Could not initialize polyhedral data: the vertex (xyzVertex) or facet (orderFacet) "
               "were not provided.";
    }

    const size_t nFacet = this->orderFacet.rows();
    int i, j, k;
    Eigen::Vector3i v;
    Eigen::Vector3d xyz1, xyz2, xyz3, e21, e32;

    /* Initialize normal */
    this->normalFacet.setZero(nFacet, 3);

    /* Loop through each facet to compute volume */
    for (unsigned int m = 0; m < nFacet; m++) {
        /* Fill auxiliary variables with vertex order on each facet */
        v = this->orderFacet.row(m);
        i = v[0] - 1;
        j = v[1] - 1;
        k = v[2] - 1;

        xyz1 = this->xyzVertex.row(i);
        xyz2 = this->xyzVertex.row(j);
        xyz3 = this->xyzVertex.row(k);

        /* Compute two edge vectors and normal to facet */
        e21 = xyz2 - xyz1;
        e32 = xyz3 - xyz2;
        this->normalFacet.row(m) = e21.cross(e32) / e21.cross(e32).norm();

        /* Add volume contribution */
        this->volPoly += abs(xyz1.cross(xyz2).transpose() * xyz3) / 6;
    }

    return {};
}

std::optional<std::string> PolyhedralGravityModel::initializeParameters(const GravBodyData& body)
{
    this->muBody = body.mu;
    return this->initializeParameters();
}

Eigen::Vector3d
PolyhedralGravityModel::computeField(const Eigen::Vector3d& position_planetFixed) const
{
    int i, j, k;
    Eigen::Vector3i v;
    Eigen::Vector3d ri, rj, rk;
    Eigen::Vector3d nf;
    Eigen::Vector3d r1, r2, re;
    Eigen::Vector3d r21, n21;
    Eigen::Matrix3d Ee;

    int idx_min;
    double a, b, e, Le;
    double wy, wx, wf;

    Eigen::Vector3d dUe, dUf, acc;
    dUe.setZero(3);
    dUf.setZero(3);

    const size_t nFacet = this->orderFacet.rows();

    /* Loop through each facet */
    for (unsigned int m = 0; m < nFacet; m++) {
        /* Fill auxiliary variables with vertex order on each facet */
        v = this->orderFacet.row(m);
        i = v[0] - 1;
        j = v[1] - 1;
        k = v[2] - 1;

        /* Compute vectors and norm from each vertex to the evaluation position */
        ri = this->xyzVertex.row(i).transpose() - position_planetFixed;
        rj = this->xyzVertex.row(j).transpose() - position_planetFixed;
        rk = this->xyzVertex.row(k).transpose() - position_planetFixed;

        /* Extract normal to facet */
        nf = this->normalFacet.row(m).transpose();

        /* Loop through each facet edge */
        for (unsigned int n = 0; n <= 2; n++) {
            switch (n) {
            case 0:
                idx_min = std::min(i, j);
                r1 = ri;
                r2 = rj;
                re = this->xyzVertex.row(idx_min).transpose() - position_planetFixed;

                a = ri.norm();
                b = rj.norm();
                break;
            case 1:
                idx_min = std::min(j, k);
                r1 = rj;
                r2 = rk;
                re = this->xyzVertex.row(idx_min).transpose() - position_planetFixed;

                a = rj.norm();
                b = rk.norm();
                break;
            case 2:
                idx_min = std::min(i, k);
                r1 = rk;
                r2 = ri;
                re = this->xyzVertex.row(idx_min).transpose() - position_planetFixed;

                a = rk.norm();
                b = ri.norm();
                break;
            }

            /* Compute along edge vector and norm */
            r21 = r2 - r1;
            e = r21.norm();
            n21 = r21.cross(nf) / r21.cross(nf).norm();

            /* Dimensionless per edge factor */
            Le = log((a + b + e) / (a + b - e));

            /* Compute dyad product */
            Ee = nf * n21.transpose();

            /* Add current facet distribution */
            dUe += Ee * re * Le;
        }

        /* Compute solid angle for the current facet */
        wy = ri.transpose() * rj.cross(rk);
        wx = ri.norm() * rj.norm() * rk.norm() + ri.norm() * rj.transpose() * rk +
             rj.norm() * rk.transpose() * ri + rk.norm() * ri.transpose() * rj;
        wf = 2 * atan2(wy, wx);

        /* Add current solid angle facet */
        dUf += nf * (nf.transpose() * ri) * wf;
    }

    /* Compute acceleration contribution */
    return (this->muBody / this->volPoly) * (-dUe + dUf);
}

double
PolyhedralGravityModel::computePotentialEnergy(const Eigen::Vector3d& positionWrtPlanet_N) const
{
    return -this->muBody / positionWrtPlanet_N.norm();
}
