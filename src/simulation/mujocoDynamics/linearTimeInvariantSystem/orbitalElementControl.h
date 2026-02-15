/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab

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

#ifndef ORBITAL_ELEMENT_CONTROL_H
#define ORBITAL_ELEMENT_CONTROL_H

#include "linearTimeInvariantSystem.h"

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/ClassicElementsMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdForceInertialMsgPayload.h"

/**
 * @class OrbitalElementControl
 * @brief LTI based orbital element feedback controller that outputs an
 *        inertial force command.
 *
 * This class derives from LinearTimeInvariantSystem and implements a
 * controller in classical orbital element coordinates. The LTI input
 * is a 6 element vector of orbital element errors
 *
 *   u = [da/a, de, di, dOmega, domega, dM]^T
 *
 * constructed from target and current classical elements. The LTI
 * output y is also a 6 element vector in the same orbital element
 * space.
 *
 * The module then uses the Gauss planetary control matrix B(oe) to map
 * y to a Hill frame force, and finally transforms this force into the
 * inertial frame using the current orbit geometry. The inertial force
 * is written to a CmdForceInertialMsgPayload.
 *
 * The internal state dimension and A, B, C, D matrices are configured
 * through the LinearTimeInvariantSystem interface and are not fixed by
 * this class.
 */
class OrbitalElementControl : public LinearTimeInvariantSystem
{
public:
    /**
     * @brief Default constructor.
     *
     * Constructs an OrbitalElementControl instance with mu set to zero
     * and unconnected input and output messages. The user must set mu,
     * configure the LTI matrices, and connect the input and output
     * messages before the module is used in a simulation.
     */
    OrbitalElementControl() = default;

    /**
     * @brief Set the proportional gain matrix for the orbital element controller.
     *
     * This method configures the proportional feedback gain used in the
     * internal LinearTimeInvariantSystem realization of the control law.
     *
     * The proportional term maps the instantaneous 6x1 orbital-element error
     * vector into a 6x1 control output before it is transformed by the Gauss
     * planetary-equations B-matrix in writeOutput().
     *
     * Internally, this method assigns the gain matrix K to the D matrix of the
     * underlying LTI system:
     *
     *      y = C x + D u
     *
     * where:
     *   - u is the six-element orbital-element error vector
     *   - D = K implements the proportional action
     *
     * @param K 6x6 proportional gain matrix.
     */
    void setProportionalGain(const Eigen::MatrixXd& K);


    /**
     * @brief Set the integral gain matrix for the orbital element controller.
     *
     * This method configures the integral feedback gain for the controller,
     * implemented using the state dynamics of the underlying
     * LinearTimeInvariantSystem. The integral term accumulates the
     * orbital-element error over time and contributes to the output force.
     *
     * The structure set here is:
     *
     *      ẋ = B u                with B = I [6x6]
     *      y = C x + D u          with C = K_I
     *
     * This realizes:
     *
     *      x(t) = ∫ u(t) dt        (integrator state)
     *      y = K_I x               (integral feedback)
     *
     * @param K 6x6 integral gain matrix.
     */
    void setIntegralGain(const Eigen::MatrixXd& K);


    /**
     * @brief Reset the module and validate configuration.
     *
     * This method first calls LinearTimeInvariantSystem::Reset to
     * validate A, B, C, and D against the sizes returned by
     * getStateSize, getInputSize, and getOutputSize. It then checks
     * that the gravitational parameter mu is positive and that both
     * targetOEInMsg and currentOEInMsg are linked.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void Reset(uint64_t CurrentSimNanos) override;

    /**
     * @brief Size of the LTI input vector.
     *
     * The input to the LTI system is the 6 element orbital element
     * error vector
     *
     *   u = [da/a, de, di, dOmega, domega, dM]^T.
     *
     * @return Number of input elements, always 6.
     */
    size_t getInputSize() const override { return 6; }

    /**
     * @brief Size of the LTI output vector.
     *
     * The LTI output y is a 6 element vector in orbital element space.
     * It is not directly the applied force. Instead, writeOutput uses y
     * together with the Gauss control matrix B to compute a Hill frame
     * force, then transforms that force into the inertial frame and
     * writes it to the forceOutMsg.
     *
     * @return Number of output elements, always 6.
     */
    size_t getOutputSize() const override { return 6; }

    /**
     * @brief Read the current input vector for the LTI system.
     *
     * This method reads the target and current classical orbital element
     * messages and constructs the 6 element orbital element error vector
     *
     *   u = [da/a, de, di, dOmega, domega, dM]^T,
     *
     * where:
     *   - da/a is the relative semi major axis error
     *   - de     is the eccentricity error
     *   - di     is the inclination error
     *   - dOmega is the RAAN error
     *   - domega is the argument of periapsis error
     *   - dM     is the mean anomaly error
     *
     * Angular differences are wrapped into [-pi, pi].
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     * @return Input vector u as an Eigen::VectorXd of length
     *         getInputSize().
     */
    Eigen::VectorXd readInput(uint64_t CurrentSimNanos) override;

    /**
     * @brief Map the LTI output vector to an inertial force command and
     *        write the output message.
     *
     * The argument y is the output of the LTI system in orbital element
     * space, with dimension getOutputSize() = 6. This method performs
     * the following steps:
     *
     *   1. Read the current classical orbital elements from
     *      currentOEInMsg and construct a ClassicElements struct.
     *   2. Compute the 6 by 3 Gauss control matrix B(oe) that maps
     *      Hill frame force to orbital element rates.
     *   3. Interpret y as an orbital elements feedback vector and
     *      compute a Hill frame force, for example via
     *      f_H = B^T * y.
     *   4. Convert the current elements to inertial position and
     *      velocity and compute the Hill to inertial direction cosine
     *      matrix.
     *   5. Rotate the Hill frame force into the inertial frame and
     *      write it into forceOutMsg as a CmdForceInertialMsgPayload.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     * @param y LTI output vector in orbital element space with length
     *          getOutputSize().
     */
    void writeOutput(uint64_t CurrentSimNanos,
                     const Eigen::VectorXd &y) override;

public:
    /**
     * @brief Gravitational parameter mu [m^3/s^2].
     *
     * This parameter must be set to the central body's gravitational
     * parameter before use. The Reset method checks that mu is positive
     * and reports an error if it is not.
     */
    double mu = 0.0;

    /** @brief Target classical orbital elements input. */
    ReadFunctor<ClassicElementsMsgPayload> targetOEInMsg;

    /**
     * @brief Current classical orbital elements input. */
    ReadFunctor<ClassicElementsMsgPayload> currentOEInMsg;

    /**
     * @brief Inertial force command output message. */
    Message<CmdForceInertialMsgPayload> forceOutMsg;
};

#endif /* ORBITAL_ELEMENT_CONTROL_H */
