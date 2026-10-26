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

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/LambertProblemMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "fswAlgorithms/orbitControl/lambertSolver/lambertSolver.h"
#include <array>
#include <cfenv>
#include <cstddef>
#include <gtest/gtest.h>
#include <initializer_list>
#include <limits>

namespace {
/** @brief Construct an ordinary prograde transfer for validation tests.
    @return Valid input payload using the default zero-revolution Izzo method.
*/
LambertProblemMsgPayload validProblem()
{
    LambertProblemMsgPayload problem{};
    problem.mu = 3.986004360e14;   // [m^3/s^2]
    problem.transferTime = 2000.0; // [s]
    problem.r1_N[0] = 8.0e6;       // [m]
    problem.r2_N[1] = 8.0e6;       // [m]
    return problem;
}
}

/** @brief Zero-initialized messages retain the default Izzo method and produce a valid solution. */
TEST(LambertInputValidation, DefaultSolverMethodIsIzzo)
{
    auto problem = validProblem();
    ASSERT_EQ(problem.solverMethod, IZZO);
    EXPECT_EQ(problem.solverMethod, 0);
    Message<LambertProblemMsgPayload> message;
    message.write(&problem, 0, 0);
    LambertSolver solver;
    solver.lambertProblemInMsg.subscribeTo(&message);
    auto solutionReader = solver.lambertSolutionOutMsg.addSubscriber();
    solver.Reset(0);
    solver.UpdateState(0);
    EXPECT_EQ(solutionReader().valid, 1);
}

/** @brief Preserve invalid method integers in transport and reject them before enum conversion. */
TEST(LambertInputValidation, RejectsUnknownSolverMethodsAndRecovers)
{
    for (SolverMethod acceptedMethod : { GOODING, IZZO }) {
        SCOPED_TRACE(acceptedMethod);
        auto problem = validProblem();
        Message<LambertProblemMsgPayload> message;
        LambertSolver solver;
        solver.bskLogger.setLogLevel(BSK_SILENT);
        solver.lambertProblemInMsg.subscribeTo(&message);
        auto solutionReader = solver.lambertSolutionOutMsg.addSubscriber();
        solver.Reset(0);
        for (int method : { std::numeric_limits<int>::min(), -1, 2, std::numeric_limits<int>::max() }) {
            SCOPED_TRACE(method);
            problem.solverMethod = method;
            message.write(&problem, 0, 0);
            EXPECT_EQ(solver.lambertProblemInMsg().solverMethod, method);
            EXPECT_THROW(solver.UpdateState(0), BasiliskError);

            problem.solverMethod = acceptedMethod;
            message.write(&problem, 0, 0);
            EXPECT_NO_THROW(solver.UpdateState(0));
            EXPECT_EQ(solutionReader().valid, 1);
        }
    }
}

/** @brief Reject zero and nonfinite inputs without dividing by zero or normalizing NaNs. */
TEST(LambertInputValidation, RejectsMalformedInputsBeforeGeometry)
{
    for (SolverMethod method : { GOODING, IZZO }) {
        for (int field = 0; field < 4; ++field) {
            for (double value : { 0.0,
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity() }) { // Units follow the selected field.
                const int componentCount = field < 2 ? 1 : 3;
                for (int component = 0; component < componentCount; ++component) {
                    SCOPED_TRACE(method);
                    SCOPED_TRACE(field);
                    SCOPED_TRACE(value);
                    SCOPED_TRACE(component);
                    auto problem = validProblem();
                    problem.solverMethod = method;
                    if (field == 0) {
                        problem.mu = value;
                    } else if (field == 1) {
                        problem.transferTime = value;
                    } else {
                        double* position = field == 2 ? problem.r1_N : problem.r2_N;
                        position[0] = position[1] = position[2] = 0.0; // [m]
                        position[component] = value;
                    }
                    Message<LambertProblemMsgPayload> message;
                    message.write(&problem, 0, 0);
                    LambertSolver solver;
                    solver.bskLogger.setLogLevel(BSK_SILENT);
                    solver.lambertProblemInMsg.subscribeTo(&message);
                    solver.Reset(0);
                    std::feclearexcept(FE_ALL_EXCEPT);
                    EXPECT_THROW(solver.UpdateState(0), BasiliskError);
                    EXPECT_EQ(std::fetestexcept(FE_INVALID | FE_DIVBYZERO), 0);
                }
            }
        }
    }
}

/** @brief A tiny positive threshold must reject zero or roundoff-only orbit normals. */
TEST(LambertInputValidation, TinyThresholdStillRejectsCollinearPositions)
{
    for (SolverMethod method : { GOODING, IZZO }) {
        auto problem = validProblem();
        problem.solverMethod = method;
        Message<LambertProblemMsgPayload> message;
        LambertSolver solver;
        solver.bskLogger.setLogLevel(BSK_SILENT);
        solver.setAlignmentThreshold(std::numeric_limits<double>::denorm_min()); // [deg]
        solver.lambertProblemInMsg.subscribeTo(&message);
        auto solutionReader = solver.lambertSolutionOutMsg.addSubscriber();
        solver.Reset(0);
        for (const auto& radial : { std::array<double, 3>{ 1.0, 0.0, 0.0 }, std::array<double, 3>{ 1.0, 2.0, 3.0 } }) {
            for (double radiusRatio : { -3.0, 3.0 }) {
                SCOPED_TRACE(method);
                SCOPED_TRACE(radiusRatio);
                for (std::size_t axis = 0; axis < radial.size(); ++axis) {
                    problem.r1_N[axis] = 1.0e6 * radial[axis]; // [m]
                    problem.r2_N[axis] = radiusRatio * problem.r1_N[axis];
                }
                message.write(&problem, 0, 0);
                std::feclearexcept(FE_ALL_EXCEPT);
                solver.UpdateState(0);
                EXPECT_EQ(std::fetestexcept(FE_INVALID | FE_DIVBYZERO), 0);
                const auto solution = solutionReader();
                EXPECT_EQ(solution.valid, 0);
                EXPECT_EQ(solution.validSol2, 0);
            }
        }
    }
}

/** @brief Reject antiparallel endpoints before lambda or time scaling can raise floating-point exceptions. */
TEST(LambertInputValidation, RejectsCollinearityBeforeLambdaAndTimeScaling)
{
    for (SolverMethod method : { GOODING, IZZO }) {
        for (double threshold : { 1.0e-15, 1.0 }) { // [deg]
            // The first case can round 1-c/s negative; the second underflows s^3
            // to zero despite having finite, nonzero endpoint norms.
            for (const auto& position : { std::array<double, 3>{ -5.10e6, -8.05e6, 3.59e6 },
                                          std::array<double, 3>{ -5.10e-110, -8.05e-110, 3.59e-110 } }) { // [m]
                SCOPED_TRACE(method);
                SCOPED_TRACE(threshold);
                SCOPED_TRACE(position[0]);
                auto problem = validProblem();
                problem.solverMethod = method;
                problem.transferTime = 10000.0; // [s]
                for (std::size_t axis = 0; axis < position.size(); ++axis) {
                    problem.r1_N[axis] = position[axis];
                    problem.r2_N[axis] = -7.0 * position[axis];
                }
                Message<LambertProblemMsgPayload> message;
                message.write(&problem, 0, 0);
                LambertSolver solver;
                solver.bskLogger.setLogLevel(BSK_SILENT);
                solver.setAlignmentThreshold(threshold);
                solver.lambertProblemInMsg.subscribeTo(&message);
                auto solutionReader = solver.lambertSolutionOutMsg.addSubscriber();
                auto performanceReader = solver.lambertPerformanceOutMsg.addSubscriber();
                solver.Reset(0);
                std::feclearexcept(FE_ALL_EXCEPT);
                solver.UpdateState(0);
                EXPECT_EQ(std::fetestexcept(FE_INVALID | FE_DIVBYZERO), 0);
                const auto solution = solutionReader();
                const auto performance = performanceReader();
                EXPECT_EQ(solution.valid, 0);
                EXPECT_EQ(solution.validSol2, 0);
                for (std::size_t axis = 0; axis < position.size(); ++axis) {
                    EXPECT_DOUBLE_EQ(solution.v1_N[axis], 0.0);
                    EXPECT_DOUBLE_EQ(solution.v2_N[axis], 0.0);
                    EXPECT_DOUBLE_EQ(solution.v1Sol2_N[axis], 0.0);
                    EXPECT_DOUBLE_EQ(solution.v2Sol2_N[axis], 0.0);
                }
                EXPECT_DOUBLE_EQ(performance.x, 0.0);
                EXPECT_DOUBLE_EQ(performance.xSol2, 0.0);
                EXPECT_EQ(performance.numIter, 0);
                EXPECT_EQ(performance.numIterSol2, 0);
                EXPECT_DOUBLE_EQ(performance.errX, 0.0);
                EXPECT_DOUBLE_EQ(performance.errXSol2, 0.0);
            }
        }
    }
}
