/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef MJEQUALITY_H
#define MJEQUALITY_H

#include <mujoco/mujoco.h>
#include "architecture/messaging/messaging.h"
#include "MJObject.h"

class MJSpec;

/// @cond
/**
 * @brief Specialization of getObjectTypeName for mjsEquality.
 *
 * Provides a human-readable name for the MuJoCo equality constraint type.
 *
 * @return A string view representing the equality constraint type.
 */
template <>
constexpr std::string_view MJBasilisk::detail::getObjectTypeName<mjsEquality>()
{
    return "equality";
}
/// @endcond

/**
 * @brief Represents a MuJoCo equality constraint.
 *
 * This class provides functionality to manage an equality constraint in MuJoCo, allowing
 * the constraint to be activated or deactivated as needed.
 */
class MJEquality : public MJObject<mjsEquality>
{
public:
    /**
     * @brief Constructs an MJEquality object with a given equality constraint.
     *
     * @param mjsequality Pointer to the MuJoCo equality constraint.
     * @param spec Reference to the `MJSpec` object where this equality is defined.
     */
    MJEquality(mjsEquality* mjsequality, MJSpec& spec) : MJObject(mjsequality), spec(spec) {}

    /**
     * @brief Sets the active state of the equality constraint.
     *
     * @param active Boolean indicating whether the constraint should be active.
     */
    void setActive(bool active);

    /**
     * @brief Set the solref parameters for this equality constraint.
     *
     * The solref attribute controls the reference acceleration that the solver
     * tries to enforce. It can be specified in two formats:
     *
     * - Positive values: (timeconst, dampratio)
     *   - @p timeconst : Time constant τ (seconds), must be ≥ 2x timestep.
     *   - @p dampratio : Damping ratio ζ, where 1.0 = critical damping.
     *
     * - Negative values: (-stiffness, -damping)
     *   - @p stiffness : Direct stiffness constant (N/m).
     *   - @p damping   : Direct damping constant (Ns/m).
     *
     * See more in the MuJoCo documentation about "Solver parameters".
     *
     * Example usage:
     * @code
     * eq.setSolref(0.05, 1.0);   // time-constant / damping ratio form
     * eq.setSolref(-1000, -5.0); // stiffness / damping form
     * @endcode
     *
     * @param val1 Either time constant (τ) or -stiffness depending on sign.
     * @param val2 Either damping ratio (ζ) or -damping depending on sign.
     */
    void setSolref(double val1, double val2);

    /**
     * @brief Set the solimp parameters for this equality constraint.
     *
     * The solimp attribute defines how the impedance d(r) varies with constraint
     * violation r, allowing smooth, position-dependent stiffness/damping.
     *
     * See more in the MuJoCo documentation about "Solver parameters".
     *
     * Example usage:
     * @code
     * eq.setSolimp(0.9, 0.95, 0.001, 0.5, 2.0);
     * @endcode
     *
     * @param d0       Impedance at zero violation (0 < d0 < 1).
     * @param dwidth   Impedance at max violation width (0 < dwidth < 1).
     * @param width    Transition distance (positive scalar).
     * @param midpoint Inflection point in [0, 1], relative to width.
     * @param power    Power ≥ 1, controls curve steepness.
     */
    void setSolimp(double d0, double dwidth, double width,
                   double midpoint, double power);

protected:
    MJSpec& spec; ///< Reference to the object where this equality is defined.
};

/**
 * @brief Represents an equality constraint applied to a 1-degree-of-freedom joint.
 *
 * This equality can be used to enforce a specific state on said joint.
 *
 * For revolute joints, this is the angle (in radians) of the joint with respect
 * to its zero-pose.
 *
 * For linear joints, this is the displacement (in meters) of the joint with
 * respect to its zero-pose.
 */
class MJSingleJointEquality : public MJEquality
{
public:
    using MJEquality::MJEquality;

    /**
     * @brief Sets the value that the state of the joint should have.
     *
     * @param val The value to apply to the joint constraint.
     */
    void setJointOffsetConstraint(double val);
};

#endif
