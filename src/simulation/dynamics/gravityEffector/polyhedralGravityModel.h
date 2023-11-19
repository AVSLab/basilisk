/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef POLY_GRAVITY_MODEL_H
#define POLY_GRAVITY_MODEL_H

#include "simulation/dynamics/_GeneralModuleFiles/gravityModel.h"

/** The Polyhedral gravity model.
 *
 * In this class, a polyhedron is defined by its triangular facets.
 * Each facet is defined by three vertices (they are triangles), and
 * each vertex is defined by its position relative to the center of
 * mass of the body.
 */
class PolyhedralGravityModel : public GravityModel {
  public:

    /** Initialize all parameters necessary for the computation of gravity.
     *
     * The attribute `muBody` must be set separately.
     *
     * Will return an error message (string) if `xyzVertex` or `orderFacet` were not set.
     * Otherwise, returns an empty optional.
     */
    std::optional<std::string> initializeParameters() override;

    /** Initialize all parameters necessary for the computation of gravity.
     *
     * The attribute `muBody` is read from the given `GravBodyData`.
     *
     * Will return an error message (string) if `xyzVertex` or `orderFacet` were not set.
     * Otherwise, returns an empty optional.
     */
    std::optional<std::string> initializeParameters(const GravBodyData&) override;

    /** Returns the gravity acceleration at a position around this body.
     *
     * The position is given in the body-fixed reference frame.
     * Likewise, the resulting acceleration should be given in the
     * body-fixed reference frame.
     */
    Eigen::Vector3d computeField(const Eigen::Vector3d& position_planetFixed) const override;

    /** Returns the gravitational potential energy at a position around this body.
     *
     * The current implementation returns the potential energy of a point-mass
     * (the polyhedral shape of the body is ignored)
     *
     * The position is given relative to the body and in the inertial
     * reference frame.
     */
    double computePotentialEnergy(const Eigen::Vector3d& positionWrtPlanet_N) const;

  public:
    double muBody = 0;  /**< [m^3/s^2] Gravitation parameter for the planet */

    /**
     * This matrix contains the position of every vertex of this
     * polyhedron, in meters. Each row corresponds to a different
     * vertex, while each column corresponds to x, y, z respectively.
     */
    Eigen::MatrixX3d xyzVertex;

    /**
     * This matrix defines the facets of the matrix. Each row
     * contains three numbers, each of them corresponding to the
     * index of a vertex, as defined in xyzVertex. These three
     * vertices define a single facet of the polyhedron.
     *
     * Note that the order of the vertex index is important: the facets
     * must all be outward pointing.
     */
    Eigen::MatrixX3i orderFacet;

  private:
    double volPoly = 0;  /**< [m^3] Volume of the polyhedral */
    Eigen::MatrixX3d normalFacet;  /**< [-] Normal of a facet */
};

#endif /* POLY_GRAVITY_MODEL_H */
