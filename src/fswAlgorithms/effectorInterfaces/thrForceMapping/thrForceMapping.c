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
/*
    Simple Thruster Force Evaluation from
 */

#include "fswAlgorithms/effectorInterfaces/thrForceMapping/thrForceMapping.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define CONTROL_AXES_ORTHO_TOL 1e-6 /* [-] max |cos| between two control axes before they are flagged non-orthogonal */
#define SIGN_SOLVE_TOL (64.0 * DBL_EPSILON) /* [-] relative gradient and rank tolerance after normalization */
#define SIGN_SOLVE_MAX_STEPS (6 * MAX_EFF_CNT)

static int findNonpositiveForce(thrForceMappingConfig* configData,
                                double D[3][MAX_EFF_CNT],
                                double target[3],
                                double F[MAX_EFF_CNT]);

/*! self init method

 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the configData
 */
void SelfInit_thrForceMapping(thrForceMappingConfig* configData, int64_t moduleID)
{
    (void)moduleID;
    THRArrayCmdForceMsg_C_init(&configData->thrForceCmdOutMsg);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Reset_thrForceMapping(thrForceMappingConfig* configData, uint64_t callTime, int64_t moduleID)
{
    (void)callTime;
    (void)moduleID;
    double* pAxis; /* pointer to the current control axis */
    uint32_t i;
    THRArrayConfigMsgPayload localThrusterData; /* local copy of the thruster data message */

    /*! - configure the number of axes that are controlled */
    configData->numControlAxes = 0;
    for (i = 0; i < 3; i++) {
        pAxis = configData->controlAxes_B + 3 * configData->numControlAxes;
        if (v3Norm(pAxis) > configData->epsilon) {
            v3Normalize(pAxis, pAxis);
            configData->numControlAxes += 1;
        } else {
            break;
        }
    }
    if (configData->numControlAxes == 0) {
        _bskError(configData->bskLogger, "thrForceMapping() is not setup to control any axes!");
    }
    /*! - check that the control axes are mutually orthogonal, as assumed by the torque reduction [C]^T [C] Lr */
    for (i = 0; i < configData->numControlAxes; i++) {
        for (uint32_t j = i + 1; j < configData->numControlAxes; j++) {
            if (fabs(v3Dot(configData->controlAxes_B + 3 * i, configData->controlAxes_B + 3 * j)) >
                CONTROL_AXES_ORTHO_TOL) {
                _bskLog(configData->bskLogger,
                        BSK_WARNING,
                        "thrForceMapping.controlAxes_B are not orthogonal. The reduced torque and outTorqAngErr "
                        "assume orthonormal control axes.");
            }
        }
    }
    if (configData->thrForceSign == 0) {
        _bskError(configData->bskLogger, "thrForceMapping() must have thrForceSign set to either +1 or -1");
    }

    // check if the required input messages are included
    if (!THRArrayConfigMsg_C_isLinked(&configData->thrConfigInMsg)) {
        _bskError(configData->bskLogger, "Error: thrForceMapping.thrConfigInMsg wasn't connected.");
    }
    if (!VehicleConfigMsg_C_isLinked(&configData->vehConfigInMsg)) {
        _bskError(configData->bskLogger, "Error: thrForceMapping.vehConfigInMsg wasn't connected.");
    }
    if (!CmdTorqueBodyMsg_C_isLinked(&configData->cmdTorqueInMsg)) {
        _bskError(configData->bskLogger, "Error: thrForceMapping.cmdTorqueInMsg wasn't connected.");
    }

    /*! - read in the support thruster and vehicle configuration messages */
    localThrusterData = THRArrayConfigMsg_C_read(&configData->thrConfigInMsg);
    configData->sc = VehicleConfigMsg_C_read(&configData->vehConfigInMsg);

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    configData->numThrusters = (uint32_t)localThrusterData.numThrusters;
    for (i = 0; i < configData->numThrusters; i = i + 1) {
        v3Copy(localThrusterData.thrusters[i].rThrust_B, configData->rThruster_B[i]);
        v3Copy(localThrusterData.thrusters[i].tHatThrust_B, configData->gtThruster_B[i]);
        if (localThrusterData.thrusters[i].maxThrust <= 0.0) {
            _bskError(configData->bskLogger, "A configured thruster has a non-sensible saturation limit of <= 0 N!");
        } else {
            configData->thrForcMag[i] = localThrusterData.thrusters[i].maxThrust;
        }
    }
}

/*! The module takes a body frame torque vector and projects it onto available RCS or DV thrusters.

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Update_thrForceMapping(thrForceMappingConfig* configData, uint64_t callTime, int64_t moduleID)
{
    uint32_t counterPosForces;   /* []      counter for number of positive thruster forces */
    double F[MAX_EFF_CNT];       /* [N]     vector of commanded thruster forces */
    double Fbar[MAX_EFF_CNT];    /* [N]     vector of intermediate thruster forces */
    double D[3][MAX_EFF_CNT];    /* [m]     mapping matrix from thruster forces to body torque */
    double Dbar[3][MAX_EFF_CNT]; /* [m]     reduced mapping matrix*/
    double C[3][3];              /* [m]     control mapping matrix*/
    double Lr_B[3];              /* [Nm]    commanded ADCS control torque */
    double Lr_offset[3];
    double LrLocal[3];             /* [Nm]    Torque provided by indiviual thruster */
    int thrusterUsed[MAX_EFF_CNT]; /* []      Array of flags indicating if this thruster is used for the Lr_j */
    double rThrusterRelCOM_B[MAX_EFF_CNT][3]; /* [m]     local copy of the thruster locations relative to COM */
    double Lr_B_Bar[3];                       /* [Nm]    Control torque that we actually control*/
    double LrReduced_B[3]; /* [Nm]    reduced control torque [C]^T [C] Lr_B in body frame components */
    double maxFractUse;    /* []      ratio of maximum requested thruster force relative to maximum thruster limit */
    double rCrossGt[3];
    CmdTorqueBodyMsgPayload LrInputMsg;
    THRArrayCmdForceMsgPayload thrusterForceOut;

    /*! - zero all output message copies */
    thrusterForceOut = THRArrayCmdForceMsg_C_zeroMsgPayload();

    /*! - clear arrays of the thruster mapping algorithm */
    vSetZero(F, MAX_EFF_CNT);
    mSetZero(D, 3, MAX_EFF_CNT);
    mSetZero(Dbar, 3, MAX_EFF_CNT);
    mSetZero(C, 3, 3);

    /*! - Read the input messages */
    LrInputMsg = CmdTorqueBodyMsg_C_read(&configData->cmdTorqueInMsg);
    configData->sc = VehicleConfigMsg_C_read(&configData->vehConfigInMsg);

    /*! - copy the request 3D attitude control torque vector */
    v3Copy(LrInputMsg.torqueRequestBody, Lr_B);

    /*! - compute thruster locations relative to COM */
    for (uint32_t i = 0; i < configData->numThrusters; i++) {
        v3Subtract(configData->rThruster_B[i], configData->sc.CoM_B, rThrusterRelCOM_B[i]); /* Part 1 of Eq. 4 */
    }

    /*! - compute general thruster force mapping matrix */
    v3SetZero(Lr_offset);

    for (uint32_t i = 0; i < configData->numThrusters; i = i + 1) {
        v3Cross(rThrusterRelCOM_B[i], configData->gtThruster_B[i], rCrossGt); /* Eq. 6 */
        for (uint32_t j = 0; j < 3; j++) {
            D[j][i] = rCrossGt[j];
        }
        if (configData->thrForceSign <
            0) /* Handles the case where there is translational motion imparted during off-pulsing*/
        {
            v3Scale(configData->thrForcMag[i],
                    rCrossGt,
                    LrLocal); /* Computing local torques from each thruster -- Individual terms in Eq. 7*/
            v3Subtract(Lr_offset, LrLocal, Lr_offset); /* Summing of individual torques -- Eq. 5 & Eq. 7 */
        }
    }

    v3Add(Lr_offset, Lr_B, Lr_B);

    /*! - copy the control axes into [C] */
    for (uint32_t i = 0; i < configData->numControlAxes; i++) {
        v3Copy(&configData->controlAxes_B[3 * i], C[i]);
    }

    /*! - map the control torque onto the control axes*/
    m33MultV3(RECAST3X3 C, Lr_B, Lr_B_Bar); /* Note: Lr_B_Bar is projected only onto the available control axes. i.e. if
                                               using DV thrusters with only 1 control axis, Lr_B_Bar = [#, 0, 0] */

    /*! - map the reduced control torque back into body frame components to compare it with [D] F */
    m33tMultV3(RECAST3X3 C, Lr_B_Bar, LrReduced_B);

    /*! - 1st iteration of finding a set of force vectors to implement the control torque */
    findMinimumNormForce(configData, D, Lr_B_Bar, configData->numThrusters, F);

    /*! - Remove forces components that are contributing to the RCS Null space (this is due to the geometry of the
     * thrusters) */
    if (configData->thrForceSign > 0) {
        substractMin(F, configData->numThrusters);
    }

    if (configData->thrForceSign < 0 || configData->use2ndLoop) {
        counterPosForces = 0;
        memset(thrusterUsed, 0x0, MAX_EFF_CNT * sizeof(int));
        for (uint32_t i = 0; i < configData->numThrusters; i++) {
            if (F[i] * configData->thrForceSign > 0) {
                thrusterUsed[i] = 1; /* Eq. 11 */
                for (uint32_t j = 0; j < 3; j++) {
                    Dbar[j][counterPosForces] = D[j][i]; /* Eq. 12 */
                }
                counterPosForces += 1;
            }
        }

        findMinimumNormForce(configData, Dbar, Lr_B_Bar, counterPosForces, Fbar);
        if (configData->thrForceSign > 0) {
            substractMin(Fbar, counterPosForces);
        }
        uint32_t c = 0;
        for (uint32_t i = 0; i < configData->numThrusters; i++) {
            if (thrusterUsed[i]) {
                F[i] = Fbar[c];
                c += 1;
            } else {
                F[i] = 0.0;
            }
        }
    }

    /* The second unconstrained solve can reverse a selected thruster's sign.  Preserve valid allocations,
       but reconsider all thrusters with a nonnegative least-squares solve when off-pulsing needs repair. */
    if (configData->thrForceSign < 0) {
        double forceScale = 0.0; /* [N] magnitude used to distinguish roundoff from a sign violation */
        for (uint32_t i = 0; i < configData->numThrusters; i++) {
            forceScale = fmax(forceScale, fabs(F[i]));
        }
        int repairRequired = 0;
        for (uint32_t i = 0; i < configData->numThrusters; i++) {
            if (F[i] > 0.0) {
                if (F[i] <= SIGN_SOLVE_TOL * forceScale) {
                    F[i] = 0.0; /* [N] remove a roundoff-sized positive command without changing the allocation */
                } else {
                    repairRequired = 1;
                }
            }
        }
        if (repairRequired && !findNonpositiveForce(configData, D, Lr_B_Bar, F)) {
            _bskLog(configData->bskLogger,
                    BSK_WARNING,
                    "thrForceMapping off-pulsing sign solver did not converge; using its last nonpositive "
                    "force allocation.");
        }
    }

    configData->outTorqAngErr = computeTorqueAngErr(
        D, LrReduced_B, configData->numThrusters, configData->epsilon, F, configData->thrForcMag); /* Eq. 21 */
    maxFractUse = 0.0;
    /*  check if the angle between the request and actual torque exceeds a limit.  If so, then uniformly scale
        all thruster forces values to not exceed saturation.
        If the angle threshold is negative, then this scaling is bypassed.*/
    if (configData->outTorqAngErr > configData->angErrThresh) {
        for (uint32_t i = 0; i < configData->numThrusters; i++) {
            if (configData->thrForcMag[i] > 0.0 &&
                fabs(F[i]) / configData->thrForcMag[i] > maxFractUse) /* confirming that maxThrust > 0 */
            {
                maxFractUse = fabs(F[i]) / configData->thrForcMag[i];
            }
        }
        /* only scale the requested thruster force if one or more thrusters are saturated */
        if (maxFractUse > 1.0) {
            vScale(1.0 / maxFractUse, F, configData->numThrusters, F);
            configData->outTorqAngErr = computeTorqueAngErr(
                D, LrReduced_B, configData->numThrusters, configData->epsilon, F, configData->thrForcMag);
        }
    }

    /* store the output message */
    mCopy(F, configData->numThrusters, 1, thrusterForceOut.thrForce);
    THRArrayCmdForceMsg_C_write(&thrusterForceOut, &configData->thrForceCmdOutMsg, moduleID, callTime);

    return;
}

/*!
 @brief Solve a least-squares problem on up to three independent thruster columns.
 @param columns Negative control-axis torque map divided by one common scale [-].
 @param target Normalized control-axis torque request [-].
 @param passive Indices of the currently free thrusters.
 @param count Number of free thrusters, at most three.
 @param trial Unconstrained normalized force coefficients, indexed by thruster [-].
 @return One on success, zero if the selected columns are numerically dependent.
 @note Reorthogonalized QR avoids squaring the condition number in the normal equations.
 */
static int solvePassiveForces(double columns[MAX_EFF_CNT][3],
                              double target[3],
                              uint32_t passive[3],
                              uint32_t count,
                              double trial[MAX_EFF_CNT])
{
    double basis[3][3] = { { 0.0 } };
    double upper[3][3] = { { 0.0 } };
    double rhs[3] = { 0.0 };
    vSetZero(trial, MAX_EFF_CNT);
    for (uint32_t i = 0; i < count; i++) {
        v3Copy(columns[passive[i]], basis[i]);
        for (uint32_t pass = 0; pass < 2; pass++) {
            for (uint32_t j = 0; j < i; j++) {
                double projection = v3Dot(basis[j], basis[i]);
                upper[j][i] += projection;
                for (uint32_t axis = 0; axis < 3; axis++) {
                    basis[i][axis] -= projection * basis[j][axis];
                }
            }
        }
        upper[i][i] = v3Norm(basis[i]);
        if (upper[i][i] <= SIGN_SOLVE_TOL) {
            return 0;
        }
        v3Scale(1.0 / upper[i][i], basis[i], basis[i]);
        rhs[i] = v3Dot(basis[i], target);
    }
    for (uint32_t remaining = count; remaining > 0; remaining--) {
        uint32_t i = remaining - 1;
        double coefficient = rhs[i];
        for (uint32_t j = i + 1; j < count; j++) {
            coefficient -= upper[i][j] * trial[passive[j]];
        }
        trial[passive[i]] = coefficient / upper[i][i];
    }
    return 1;
}

/*!
 @brief Project feasible free forces onto the minimum-norm allocation with the same achieved torque.
 @param columns Commonly scaled negative control-axis torque map [-].
 @param feasible Current feasible normalized force coefficients [-].
 @param freeThruster Flags identifying forces not fixed at zero.
 @param numThrusters Number of configured thrusters.
 @param trial Minimum-norm normalized force coefficients [-].
 @param multiplier Equality-constraint multiplier in normalized control-axis components [-].
 @note Column-pivoted, reorthogonalized QR of the transposed free map handles dependent control axes
 without forming normal equations. With orthonormal row-space basis Q, the projection Q Q^T feasible
 preserves the achieved torque. Using the feasible forces avoids cancellation in a separate triangular
 solve for the torque, which could spuriously activate and release a zero-force bound. At most three
 basis vectors are needed, regardless of thruster count.
 */
static void solveMinimumForceNorm(double columns[MAX_EFF_CNT][3],
                                  double feasible[MAX_EFF_CNT],
                                  int freeThruster[MAX_EFF_CNT],
                                  uint32_t numThrusters,
                                  double trial[MAX_EFF_CNT],
                                  double multiplier[3])
{
    double basis[3][MAX_EFF_CNT] = { { 0.0 } };
    double upper[3][3] = { { 0.0 } };
    double weights[3] = { 0.0 };
    uint32_t permutation[3] = { 0, 1, 2 };
    uint32_t rank = 0;
    double scale = 0.0; /* [-] largest column norm of the transposed free map */
    vSetZero(trial, MAX_EFF_CNT);
    v3SetZero(multiplier);
    for (uint32_t axis = 0; axis < 3; axis++) {
        for (uint32_t i = 0; i < numThrusters; i++) {
            if (freeThruster[i]) {
                basis[axis][i] = columns[i][axis];
            }
        }
        scale = fmax(scale, vNorm(basis[axis], numThrusters));
    }
    for (uint32_t axis = 0; axis < 3; axis++) {
        uint32_t pivot = axis;
        double largest = vNorm(basis[axis], numThrusters);
        for (uint32_t candidate = axis + 1; candidate < 3; candidate++) {
            double magnitude = vNorm(basis[candidate], numThrusters);
            if (magnitude > largest) {
                largest = magnitude;
                pivot = candidate;
            }
        }
        if (largest <= SIGN_SOLVE_TOL * scale) {
            break;
        }
        if (pivot != axis) {
            for (uint32_t i = 0; i < numThrusters; i++) {
                double temporary = basis[axis][i];
                basis[axis][i] = basis[pivot][i];
                basis[pivot][i] = temporary;
            }
            for (uint32_t i = 0; i < axis; i++) {
                double temporary = upper[i][axis];
                upper[i][axis] = upper[i][pivot];
                upper[i][pivot] = temporary;
            }
            uint32_t temporary = permutation[axis];
            permutation[axis] = permutation[pivot];
            permutation[pivot] = temporary;
        }
        /* Earlier eliminations can lose orthogonality for nearly dependent axes.  Reorthogonalize
           against the complete established basis before using Q to form the minimum-norm solution. */
        for (uint32_t pass = 0; pass < 2; pass++) {
            for (uint32_t previous = 0; previous < axis; previous++) {
                double projection = vDot(basis[previous], numThrusters, basis[axis]);
                upper[previous][axis] += projection;
                for (uint32_t i = 0; i < numThrusters; i++) {
                    basis[axis][i] -= projection * basis[previous][i];
                }
            }
        }
        largest = vNorm(basis[axis], numThrusters);
        if (largest <= SIGN_SOLVE_TOL * scale) {
            break;
        }
        upper[axis][axis] = largest;
        vScale(1.0 / largest, basis[axis], numThrusters, basis[axis]);
        for (uint32_t candidate = axis + 1; candidate < 3; candidate++) {
            for (uint32_t pass = 0; pass < 2; pass++) {
                double projection = vDot(basis[axis], numThrusters, basis[candidate]);
                upper[axis][candidate] += projection;
                for (uint32_t i = 0; i < numThrusters; i++) {
                    basis[candidate][i] -= projection * basis[axis][i];
                }
            }
        }
        rank++;
    }
    for (uint32_t axis = 0; axis < rank; axis++) {
        weights[axis] = vDot(basis[axis], numThrusters, feasible);
    }
    for (uint32_t i = 0; i < numThrusters; i++) {
        for (uint32_t axis = 0; axis < rank; axis++) {
            trial[i] += basis[axis][i] * weights[axis];
        }
    }
    for (uint32_t remaining = rank; remaining > 0; remaining--) {
        uint32_t axis = remaining - 1;
        double rhs = weights[axis];
        for (uint32_t i = axis + 1; i < rank; i++) {
            rhs -= upper[axis][i] * multiplier[permutation[i]];
        }
        multiplier[permutation[axis]] = rhs / upper[axis][axis];
    }
}

/*!
 @brief Minimize force norm without changing the torque obtained by nonnegative least squares.
 @param columns Commonly scaled negative control-axis torque map [-].
 @param requested Original normalized torque request in control-axis components [-].
 @param numThrusters Number of configured thrusters.
 @param coefficients Nonnegative normalized forces, initially feasible and updated in place [-].
 @param steps Shared active-set step count, bounded by SIGN_SOLVE_MAX_STEPS.
 @return One when the minimum-norm optimality conditions hold, zero on early termination.
 @note Columns with a strictly negative NNLS residual gradient cannot contribute to another allocation
 with the same optimal torque.  All other columns start free; bounds may subsequently be added and released.
 */
static int minimizeForceNorm(double columns[MAX_EFF_CNT][3],
                             double requested[3],
                             uint32_t numThrusters,
                             double coefficients[MAX_EFF_CNT],
                             uint32_t* steps)
{
    int freeThruster[MAX_EFF_CNT] = { 0 };
    int eligible[MAX_EFF_CNT] = { 0 };
    double target[3] = { 0.0 };
    double trial[MAX_EFF_CNT];
    double multiplier[3];
    for (uint32_t i = 0; i < numThrusters; i++) {
        for (uint32_t axis = 0; axis < 3; axis++) {
            target[axis] += columns[i][axis] * coefficients[i];
        }
    }
    double residual[3];
    v3Subtract(requested, target, residual);
    double gradientTolerance = SIGN_SOLVE_TOL * (1.0 + vNorm(coefficients, numThrusters));
    for (uint32_t i = 0; i < numThrusters; i++) {
        eligible[i] = coefficients[i] > 0.0 || v3Dot(columns[i], residual) >= -gradientTolerance;
        freeThruster[i] = eligible[i];
    }
    while (*steps < SIGN_SOLVE_MAX_STEPS) {
        (*steps)++;
        solveMinimumForceNorm(columns, coefficients, freeThruster, numThrusters, trial, multiplier);
        double forceScale = 0.0; /* [-] largest normalized trial or current force */
        for (uint32_t i = 0; i < numThrusters; i++) {
            forceScale = fmax(forceScale, fmax(fabs(trial[i]), coefficients[i]));
        }
        double fraction = 1.0; /* [-] feasible fraction of the minimum-norm step */
        uint32_t leaving = numThrusters;
        for (uint32_t i = 0; i < numThrusters; i++) {
            if (freeThruster[i] && trial[i] < -SIGN_SOLVE_TOL * forceScale) {
                double candidate = coefficients[i] / (coefficients[i] - trial[i]);
                if (leaving == numThrusters || candidate < fraction) {
                    fraction = candidate;
                    leaving = i;
                }
            }
        }
        if (leaving != numThrusters) {
            for (uint32_t i = 0; i < numThrusters; i++) {
                coefficients[i] = fmax(0.0, coefficients[i] + fraction * (trial[i] - coefficients[i]));
            }
            coefficients[leaving] = 0.0;
            freeThruster[leaving] = 0;
            continue;
        }
        for (uint32_t i = 0; i < numThrusters; i++) {
            coefficients[i] = fmax(0.0, trial[i]);
        }
        double dualTolerance = SIGN_SOLVE_TOL * v3Norm(multiplier);
        double bestGradient = dualTolerance;
        uint32_t entering = numThrusters;
        for (uint32_t i = 0; i < numThrusters; i++) {
            double gradient = v3Dot(columns[i], multiplier);
            if (eligible[i] && !freeThruster[i] && gradient > bestGradient + dualTolerance) {
                bestGradient = gradient;
                entering = i;
            }
        }
        if (entering == numThrusters) {
            return 1;
        }
        freeThruster[entering] = 1;
    }
    return 0;
}

/*!
 @brief Repair an off-pulsing allocation by minimizing control-axis torque error with F <= 0.
 @param configData Module configuration, including the control axes and thruster count.
 @param D Body-frame torque map [m].
 @param target Nominal-thrust-compensated request in control-axis components [N*m].
 @param F Repaired signed force commands [N].
 @return One when both torque-error and force-norm optimality tests are satisfied, zero on early termination.
 @note The first active-set stage minimizes torque error from zero, using up to three independent columns.
 The second minimizes force norm while holding the achieved torque fixed. One common map scale preserves
 relative torque authority. Both stages share SIGN_SOLVE_MAX_STEPS active-set steps; early termination
 retains a nonpositive allocation. Thrust limits remain governed by the existing angle-threshold policy.
 */
static int findNonpositiveForce(thrForceMappingConfig* configData,
                                double D[3][MAX_EFF_CNT],
                                double target[3],
                                double F[MAX_EFF_CNT])
{
    double columns[MAX_EFF_CNT][3] = { { 0.0 } };
    double mapScale = 0.0; /* [m] common scale preserving relative thruster torque authority */
    double normalizedTarget[3];
    double coefficients[MAX_EFF_CNT] = { 0.0 };
    double trial[MAX_EFF_CNT];
    uint32_t passive[3] = { 0 };
    uint32_t count = 0;
    uint32_t steps = 0;
    int freeThruster[MAX_EFF_CNT] = { 0 };
    int rejected[MAX_EFF_CNT] = { 0 };
    int converged = 0;
    double targetNorm = v3Norm(target); /* [N*m] */

    vSetZero(F, MAX_EFF_CNT);
    if (targetNorm == 0.0) {
        return 1;
    }
    v3Scale(1.0 / targetNorm, target, normalizedTarget);
    for (uint32_t i = 0; i < configData->numThrusters; i++) {
        for (uint32_t axis = 0; axis < configData->numControlAxes; axis++) {
            for (uint32_t body = 0; body < 3; body++) {
                columns[i][axis] -= configData->controlAxes_B[3 * axis + body] * D[body][i];
            }
        }
        mapScale = fmax(mapScale, v3Norm(columns[i]));
    }
    if (mapScale == 0.0) {
        return 1;
    }
    for (uint32_t i = 0; i < configData->numThrusters; i++) {
        v3Scale(1.0 / mapScale, columns[i], columns[i]);
    }

    while (steps < SIGN_SOLVE_MAX_STEPS) {
        double residual[3];
        v3Copy(normalizedTarget, residual);
        for (uint32_t i = 0; i < configData->numThrusters; i++) {
            for (uint32_t axis = 0; axis < 3; axis++) {
                residual[axis] -= columns[i][axis] * coefficients[i];
            }
        }
        double gradientTolerance = SIGN_SOLVE_TOL * (1.0 + vNorm(coefficients, configData->numThrusters));
        double bestGradient = gradientTolerance;
        uint32_t entering = configData->numThrusters;
        for (uint32_t i = 0; i < configData->numThrusters; i++) {
            double gradient = v3Dot(columns[i], residual);
            if (!freeThruster[i] && !rejected[i] && gradient > bestGradient + gradientTolerance) {
                bestGradient = gradient;
                entering = i;
            }
        }
        if (entering == configData->numThrusters) {
            converged = 1;
            for (uint32_t i = 0; i < configData->numThrusters; i++) {
                if (rejected[i] && v3Dot(columns[i], residual) > gradientTolerance) {
                    converged = 0;
                }
            }
            break;
        }
        if (count == 3) {
            break;
        }
        passive[count++] = entering;
        steps++;
        if (!solvePassiveForces(columns, normalizedTarget, passive, count, trial)) {
            count--;
            rejected[entering] = 1;
            continue;
        }
        freeThruster[entering] = 1;
        memset(rejected, 0, sizeof(rejected));

        /* Move only as far as the first zero bound, then release that thruster and solve again. */
        while (1) {
            double fraction = 1.0; /* [-] fraction of the step to the unconstrained solution */
            uint32_t leaving = count;
            for (uint32_t i = 0; i < count; i++) {
                uint32_t index = passive[i];
                if (trial[index] <= 0.0) {
                    double denominator = coefficients[index] - trial[index];
                    double candidate = denominator > 0.0 ? coefficients[index] / denominator : 0.0;
                    if (leaving == count || candidate < fraction) {
                        fraction = candidate;
                        leaving = i;
                    }
                }
            }
            if (leaving == count) {
                vCopy(trial, MAX_EFF_CNT, coefficients);
                break;
            }
            for (uint32_t i = 0; i < count; i++) {
                uint32_t index = passive[i];
                coefficients[index] = fmax(0.0, coefficients[index] + fraction * (trial[index] - coefficients[index]));
            }
            coefficients[passive[leaving]] = 0.0;
            freeThruster[passive[leaving]] = 0;
            for (uint32_t i = leaving + 1; i < count; i++) {
                passive[i - 1] = passive[i];
            }
            count--;
            if (steps == SIGN_SOLVE_MAX_STEPS) {
                break;
            }
            steps++;
            /* A subset of independent columns remains independent. */
            solvePassiveForces(columns, normalizedTarget, passive, count, trial);
        }
    }
    if (converged) {
        converged = minimizeForceNorm(columns, normalizedTarget, configData->numThrusters, coefficients, &steps);
    }
    for (uint32_t i = 0; i < configData->numThrusters; i++) {
        F[i] = -coefficients[i] * (targetNorm / mapScale);
    }
    return converged;
}

/*!
 Take a stack of force values find the smallest value, and subtract if from all force values.  Here the smallest values
 will become zero, while other forces increase.  This assumes that the thrusters are aligned such that if all
 thrusters are firing, then no torque or force is applied.  This ensures only positive force values are computed.
 */
void substractMin(double* F, uint32_t size)
{
    uint32_t i;
    double minValue = 0.0;
    for (i = 0; i < size; i++) {
        if (F[i] < minValue) {
            minValue = F[i];
        }
    }
    for (i = 0; i < size; i++) {
        F[i] = F[i] - minValue;
    }
    return;
}

/*!
 Use a least square inverse to determine the smallest set of thruster forces that yield the desired torque vector.  Note
 that this routine does not constrain yet the forces to be either positive or negative
 */
void findMinimumNormForce(thrForceMappingConfig* configData,
                          double D[3][MAX_EFF_CNT],
                          double Lr_B_Bar[3],
                          uint32_t numForces,
                          double F[MAX_EFF_CNT])
{

    uint32_t i, j, k;          /* []     counters */
    double C[3][3];            /* [m^2]  (C) matrix */
    double CD[3][MAX_EFF_CNT]; /* [m^2]  [C].[D] matrix -- Thrusters in body frame mapped on control axes */
    double CDCDT[3][3];        /* [m^2]  [CD].[CD]^T matrix */
    double CDCDTInv[3][3];     /* [m^2]  ([CD].[CD]^T)^-1 matrix */
    double CDCDTInvLr[3];

    vSetZero(F, MAX_EFF_CNT); /* zero the output force vector */
    m33SetZero(C);            /* zero the control basis */

    /*! - copy the control axes into [C] */
    for (i = 0; i < configData->numControlAxes; i++) {
        v3Copy(&configData->controlAxes_B[3 * i], C[i]);
    }

    /* find [D].[D]^T */
    mMultM(C, 3, 3, D, 3, MAX_EFF_CNT, CD);
    m33SetIdentity(CDCDT);
    for (i = 0; i < configData->numControlAxes; i++) {
        for (j = 0; j < configData->numControlAxes; j++) {
            CDCDT[i][j] = 0.0;
            for (k = 0; k < numForces; k++) {
                CDCDT[i][j] += CD[i][k] * CD[j][k]; /* Part of Eq. 9 */
            }
        }
    }

    if (m33Determinant(CDCDT) > configData->epsilon) {
        m33Inverse(CDCDT, CDCDTInv);
    } else {
        m33SetZero(CDCDTInv);
    }

    m33MultV3(CDCDTInv,
              Lr_B_Bar,
              CDCDTInvLr); /* If fewer than 3 control axes, then the 1's along the diagonal of DDTInv will not conflict
                              with the mapping, as Lr_B_Bar contains the nessessary 0s to inhibit projection */
    mtMultV(CD, 3, MAX_EFF_CNT, CDCDTInvLr, F); /* Eq. 15 */

    return;
}

/*!
 @brief Determine the angle between the desired torque vector and the actual torque vector.

 @param D Mapping matrix from thruster forces to body frame torque [m]
 @param BLr_B Desired reduced control torque [C]^T [C] Lr in body frame components [Nm]
 @param numForces Number of thruster forces to evaluate
 @param epsilon Small value below which the desired torque is treated as zero [Nm]
 @param F Commanded thruster forces [N]
 @param FMag Thruster force saturation limits [N]
 @return Angle between the desired and the actual body frame torque vector [rad]
 @note Both torque vectors must be expressed in body frame components.
 */
double computeTorqueAngErr(double D[3][MAX_EFF_CNT],
                           double BLr_B[3],
                           uint32_t numForces,
                           double epsilon,
                           double F[MAX_EFF_CNT],
                           double FMag[MAX_EFF_CNT])
{
    double returnAngle = 0.0; /* [rad]  angle between requested and actual torque vector */
    /*! - make sure a control torque is requested, otherwise just return a zero angle error */
    if (v3Norm(BLr_B) > epsilon) {

        double tauActual_B[3];  /* [Nm]   control torque with current thruster solution */
        double BLr_hat_B[3];    /* []     normalized BLr_B vector */
        double LrEffector_B[3]; /* [Nm]   torque of an individual thruster effector */
        double thrusterForce;   /* [N]    saturation constrained thruster force */

        double DT[MAX_EFF_CNT][3];
        mTranspose(D, 3, MAX_EFF_CNT, DT);
        v3Normalize(BLr_B, BLr_hat_B);
        v3SetZero(tauActual_B);

        /*! - loop over all thrusters and compute the actual torque to be applied */
        for (uint32_t i = 0; i < numForces; i++) {
            /* This could produce inf's as F[i] approaches 0 if FMag[i] is 0, as such
               we check if FMag[i] is equal to zero in reset() */
            thrusterForce = fabs(F[i]) < FMag[i] ? F[i] : FMag[i] * fabs(F[i]) / F[i];
            v3Scale(thrusterForce, DT[i], LrEffector_B);
            v3Add(tauActual_B, LrEffector_B, tauActual_B);
        }

        /*! - evaluate the angle between the requested and thruster implemented torque vector */
        v3Normalize(tauActual_B, tauActual_B);
        if (v3Dot(BLr_hat_B, tauActual_B) < 1.0) {
            returnAngle = safeAcos(v3Dot(BLr_hat_B, tauActual_B)); /* Eq. 21 */
        }
    }
    return (returnAngle);
}
