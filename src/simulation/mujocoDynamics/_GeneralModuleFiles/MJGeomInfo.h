/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef MJGEOMINFO_H
#define MJGEOMINFO_H

#include <string>
#include <vector>

// SWIG defines the SWIG macro when parsing — guard the mujoco header so the
// simplified SWIG preprocessor never sees it.  The struct itself is visible to
// both C++ (with the real mjGEOM_NONE default) and SWIG (with a plain 0).
#ifndef SWIG
#  include <mujoco/mujoco.h>
#endif

/**
 * @brief Describes the geometry (shape, size, position, orientation, color) of a single
 * MuJoCo geom, suitable for visualization.
 *
 * The @p type field stores a @c mjtGeom integer value from @c mujoco/mujoco.h.
 * Common values: @c mjGEOM_SPHERE=2, @c mjGEOM_CAPSULE=3, @c mjGEOM_ELLIPSOID=4,
 * @c mjGEOM_CYLINDER=5, @c mjGEOM_BOX=6.
 */
struct MJGeomInfo {
    std::string bodyName;                              ///< Name of the body this geom belongs to.
#ifndef SWIG
    int type = mjGEOM_NONE;                            ///< MuJoCo geom type (mjtGeom value).
#else
    int type = 0;                                      ///< MuJoCo geom type (mjtGeom value).
#endif
    std::vector<double> size = std::vector<double>(3); ///< Size parameters (3 elements).
    std::vector<double> pos  = std::vector<double>(3); ///< Position in body frame (3 elements).
    std::vector<double> quat = std::vector<double>(4); ///< Quaternion in body frame (4 elements: w, x, y, z).
    std::vector<double> rgba = std::vector<double>(4); ///< Color (4 elements, 0–1 range).
};

#endif
