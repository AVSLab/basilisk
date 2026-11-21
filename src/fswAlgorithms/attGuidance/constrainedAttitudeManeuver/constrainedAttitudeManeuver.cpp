/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "fswAlgorithms/attGuidance/constrainedAttitudeManeuver/constrainedAttitudeManeuver.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <math.h>
#include <stdexcept>
#include <utility>

namespace {
bool validListIndex(const NodeList& nodes, int index)
{
    return index >= 0 && index < nodes.N && nodes.N <= static_cast<int>(std::size(nodes.list));
}

void validateCone(const Eigen::Vector3d& boresight, double halfAngle, const char* category)
{
    if (!(std::abs(boresight.squaredNorm() - 1.0) <= 1e-12)) { // [-] Allow roundoff in a unit vector.
        BSKLogger().bskError("ConstrainedAttitudeManeuver: %s boresight must be a finite unit vector.", category);
    }
    if (!(halfAngle >= 0.0 && halfAngle <= MPI)) {
        BSKLogger().bskError("ConstrainedAttitudeManeuver: %s cone half-angle must be in [0, pi] radians.", category);
    }
}

void normalizeDirection(double direction[3], const char* source, const char* quantity)
{
    Eigen::Map<Eigen::Vector3d> axis(direction);
    const double scale = axis.cwiseAbs().maxCoeff();
    if (!axis.allFinite() || scale == 0.0) {
        BSKLogger().bskError("ConstrainedAttitudeManeuver: %s %s must be finite and nonzero.", source, quantity);
    }
    axis = (axis / scale).normalized();
}

void setBoresight(double direction[3], double halfAngle, const char* category, double output[3])
{
    Eigen::Vector3d axis = cArray2EigenVector3d(direction);
    normalizeDirection(axis.data(), category, "boresight");
    validateCone(axis, halfAngle, category);
    eigenVector3d2CArray(axis, output);
}

void validateActiveCones(const scBoresightStruct& boresights, bool keepOut, bool keepIn)
{
    const auto check = [](const auto& axes, const auto& angles, int count, const char* category) {
        if (count < 0 || count > static_cast<int>(std::size(angles))) {
            BSKLogger().bskError("ConstrainedAttitudeManeuver: invalid %s boresight count.", category);
        }
        for (int i = 0; i < count; ++i) {
            validateCone(Eigen::Map<const Eigen::Vector3d>(axes[i]), angles[i], category);
        }
    };
    if (keepOut) {
        check(boresights.keepOutBoresight_B, boresights.keepOutFov, boresights.keepOutBoresightCount, "keep-out");
    }
    if (keepIn) {
        check(boresights.keepInBoresight_B, boresights.keepInFov, boresights.keepInBoresightCount, "keep-in");
    }
}

// Peak MRP norm at an interior closest approach to the pole (Euler parameters [-1, 0, 0, 0]).
// The caller scores both endpoints using the supplied MRP norms.
double interiorPeakNorm(const Eigen::Vector4d& from, const Eigen::Vector4d& to)
{
    const auto a = from.tail<3>();
    const auto b = to.tail<3>();
    const Eigen::Vector3d tangent = from[0] * b - to[0] * a;
    if (a.dot(tangent) <= 0.0 || b.dot(tangent) >= 0.0) {
        return 0.0; // The scalar Euler parameter has no interior minimum on this short arc.
    }
    // Distance of the pole from the plane of the unit quaternion great circle.
    const double cross = a.cross(b).norm();
    const double sine = cross / std::hypot(cross, tangent.norm());
    if (sine <= 4 * std::numeric_limits<double>::epsilon()) {
        return std::numeric_limits<double>::infinity();
    }
    return (1.0 + std::sqrt(std::max(0.0, 1.0 - sine * sine))) / sine;
}
} // namespace

/*! @brief Construct an unclassified grid node. */
Node::Node() = default;

/*! The constructor requires the MRP set, the constraint structure and the sc boresight structure */
Node::Node(double sigma_BN[3], constraintStruct constraints, scBoresightStruct boresights)
{
    validateActiveCones(boresights, constraints.keepOut, constraints.keepIn);
    if (constraints.keepOut) {
        normalizeDirection(constraints.keepOutDir_N, "keep-out", "inertial axis");
    }
    if (constraints.keepIn) {
        normalizeDirection(constraints.keepInDir_N, "keep-in", "inertial axis");
    }
    // MRPswitch(sigma_BN, 1, this->sigma_BN);
    v3Copy(sigma_BN, this->sigma_BN);
    this->isBoundary = abs(v3Norm(this->sigma_BN) - 1) < 1e-5;

    // check constraint compliance
    this->isFree = true;
    double BN[3][3];
    MRP2C(this->sigma_BN, BN);
    const auto inCone = [&BN](double boresight_B[3], double direction_N[3], double halfAngle) {
        double boresight_N[3], cross[3];
        v3tMultM33(boresight_B, BN, boresight_N);
        v3Cross(boresight_N, direction_N, cross);
        const double separation = std::atan2(v3Norm(cross), v3Dot(boresight_N, direction_N));
        constexpr double angularRoundoff = 8.0 * std::numeric_limits<double>::epsilon(); // [rad]
        return separation <= halfAngle + angularRoundoff;
    };
    // keep-out
    if (constraints.keepOut) {
        for (int n = 0; n < boresights.keepOutBoresightCount; n++) {
            if (inCone(boresights.keepOutBoresight_B[n], constraints.keepOutDir_N, boresights.keepOutFov[n])) {
                this->isFree = false;
                return;
            }
        }
    }
    // keep-in
    if (constraints.keepIn) {
        for (int n = 0; n < boresights.keepInBoresightCount; n++) {
            if (inCone(boresights.keepInBoresight_B[n], constraints.keepInDir_N, boresights.keepInFov[n])) {
                return;
            }
        }
        this->isFree = false;
    }

    return;
}

/*! Module Destructor.  */
Node::~Node()
{
    return;
}

/*! This method appends a pointer to neighboring node to the neighbors class variable */
void Node::appendNeighbor(Node* node)
{
    if (this->neighborCount < 0 || this->neighborCount >= static_cast<int>(std::size(this->neighbors))) {
        BSKLogger().bskError("ConstrainedAttitudeManeuver: node neighbor capacity exceeded.");
    }
    this->neighbors[this->neighborCount] = node;
    this->neighborCount += 1;
    return;
}

/*! This is the constructor for the NodeList class. */
NodeList::NodeList()
{
    this->N = 0;
    return;
}

NodeList::NodeList(const NodeList& source)
    : NodeList()
{
    *this = source;
}

NodeList& NodeList::operator=(const NodeList& source)
{
    if (this == &source) {
        return *this;
    }
    if (source.N < 0 || source.N > static_cast<int>(std::size(source.list))) {
        BSKLogger().bskError("ConstrainedAttitudeManeuver: invalid source node count.");
    }
    for (int i = 0; i < source.N; ++i) {
        if (source.owners.count(source.list[i]) == 0) {
            BSKLogger().bskError(
                "ConstrainedAttitudeManeuver: cannot copy a list containing grid-backed or borrowed nodes.");
        }
    }
    auto newOwners = source.owners;
    const bool changed = N != source.N || !std::equal(source.list, source.list + source.N, list);
    std::copy_n(source.list, source.N, list);
    N = source.N;
    owners.swap(newOwners);
    if (changed) {
        notifyChange();
    }
    return *this;
}

/*! Class Destructor. */
NodeList::~NodeList()
{
    return;
}

void NodeList::notifyChange() const
{
    if (onChange) {
        onChange();
    }
}

/*! @brief Append a node and retain its optional storage owner until its last entry is removed.
 @param node Pointer to the node to append.
 @param owner Optional handle keeping the node alive.
 */
void NodeList::append(Node* node, std::shared_ptr<void> owner)
{
    if (this->N < 0 || this->N >= static_cast<int>(std::size(this->list))) {
        BSKLogger().bskError("ConstrainedAttitudeManeuver: node list capacity exceeded.");
    }
    if (owner) {
        owners.try_emplace(node, std::move(owner));
    }
    this->list[this->N] = node;
    this->N += 1;
    notifyChange();
}

/*! This method removes the pointer at index M from the node list. */
void NodeList::pop(int M)
{
    if (!validListIndex(*this, M)) {
        BSKLogger().bskError("ConstrainedAttitudeManeuver: node list index %d is out of range.", M);
    }
    Node* removed = this->list[M];
    for (int m = M; m < this->N - 1; m++) {
        this->list[m] = this->list[m + 1];
    }
    this->N -= 1;
    notifyChange();
    if (!owners.empty() && !contains(removed)) {
        owners.erase(removed);
    }
}

/*! This method resets the list counter to 0, effectively clearing the list. */
void NodeList::clear()
{
    if (this->N != 0) {
        this->N = 0;
        notifyChange();
    }
    owners.clear();
}

/*! This method swaps the two pointers at indices m and n. */
void NodeList::swap(int m, int n)
{
    if (!validListIndex(*this, m) || !validListIndex(*this, n)) {
        BSKLogger().bskError("ConstrainedAttitudeManeuver: node list swap indices (%d, %d) are out of range.", m, n);
    }
    if (this->list[m] != this->list[n]) {
        std::swap(this->list[m], this->list[n]);
        notifyChange();
    }
}

/*! This method sorts the nodes in the list according to their priority. */
void NodeList::sort()
{
    for (int n = 0; n + 1 < this->N; n++) {
        int best = n;
        for (int m = n + 1; m < this->N; m++) {
            if (this->list[m]->priority < this->list[best]->priority) {
                best = m;
            }
        }
        swap(n, best);
    }
}

/*! This method returns true if a node is contained in the list, false otherwise. */
bool NodeList::contains(Node* node)
{
    bool flag = false;
    for (int n = 0; n < this->N; n++) {
        if (node == this->list[n]) {
            flag = true;
        }
    }
    return flag;
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
ConstrainedAttitudeManeuver::ConstrainedAttitudeManeuver()
{
    path.onChange = [this] { invalidateWaypoints(); };
    std::fill(std::begin(sigma_BN_goal), std::end(sigma_BN_goal), std::numeric_limits<double>::quiet_NaN());
}

/*! This is the constructor for the module class. It sets default variable
    values and initializes the various parts of the model */
ConstrainedAttitudeManeuver::ConstrainedAttitudeManeuver(int N)
    : ConstrainedAttitudeManeuver()
{
    this->N = N;
    this->scStateMsgBuffer = this->scStateInMsg.zeroMsgPayload;
    this->keepOutCelBodyMsgBuffer = this->keepOutCelBodyInMsg.zeroMsgPayload;
    this->keepInCelBodyMsgBuffer = this->keepInCelBodyInMsg.zeroMsgPayload;

    return;
}

/*! Module Destructor.  */
ConstrainedAttitudeManeuver::~ConstrainedAttitudeManeuver()
{
    return;
}

/*! Initialize C-wrapped output messages */
void ConstrainedAttitudeManeuver::SelfInit()
{
    AttRefMsg_C_init(&this->attRefOutMsgC);
}

/*! Add keep-out body-frame direction  */
void ConstrainedAttitudeManeuver::appendKeepOutDirection(double direction[3], double Fov)
{
    int N = this->boresights.keepOutBoresightCount;
    if (N < 0 || N >= static_cast<int>(std::size(this->boresights.keepOutFov))) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: cannot append a keep-out boresight at index %d.", N);
    }
    setBoresight(direction, Fov, "keep-out", this->boresights.keepOutBoresight_B[N]);
    this->boresights.keepOutFov[N] = Fov;
    this->boresights.keepOutBoresightCount += 1;
}

/*! Add keep-in body-frame direction  */
void ConstrainedAttitudeManeuver::appendKeepInDirection(double direction[3], double Fov)
{
    int N = this->boresights.keepInBoresightCount;
    if (N < 0 || N >= static_cast<int>(std::size(this->boresights.keepInFov))) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: cannot append a keep-in boresight at index %d.", N);
    }
    setBoresight(direction, Fov, "keep-in", this->boresights.keepInBoresight_B[N]);
    this->boresights.keepInFov[N] = Fov;
    this->boresights.keepInBoresightCount += 1;
}

/*! This method is used to reset the module. The input messages are read here,
the grid is generated and the graph search is performed. */
void ConstrainedAttitudeManeuver::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{
    ReadInputs();

    Node startNode, goalNode;
    v3Copy(this->scStateMsgBuffer.sigma_BN, startNode.sigma_BN);
    v3Copy(this->sigma_BN_goal, goalNode.sigma_BN);
    GenerateGrid(startNode, goalNode);
    if (this->costFcnType == 0) {
        AStar();
    } else if (this->costFcnType == 1) {
        effortBasedAStar();
    } else {
        bskLogger.bskError("ConstraintAttitudeManeuver: costFcnType has not been specified.");
    }
    pathHandle();
    spline();
    this->pathCost = effortEvaluation();
    if (!std::isfinite(this->pathCost)) {
        invalidateFit();
        bskLogger.bskError("ConstrainedAttitudeManeuver: the fitted trajectory has nonfinite control effort.");
    }
    this->referenceReady = true;
    return;
}

/*! This method is the state update. It reads the information from the interpolated
trajectory and writes the output message.

 @param CurrentSimNanos The current simulation time for system
 */
void ConstrainedAttitudeManeuver::UpdateState(uint64_t CurrentSimNanos)
{
    if (!this->referenceReady) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: UpdateState requires a successful reset.");
    }
    double t = static_cast<double>(CurrentSimNanos) * NANO2SEC;
    double sigma_RN[3], sigmaDot_RN[3], sigmaDDot_RN[3], omega_RN_R[3], omegaDot_RN_R[3];
    this->Output.getData(t, sigma_RN, sigmaDot_RN, sigmaDDot_RN);

    dMRP2Omega(sigma_RN, sigmaDot_RN, omega_RN_R);
    ddMRP2dOmega(sigma_RN, sigmaDot_RN, sigmaDDot_RN, omegaDot_RN_R);

    // create the attitude output message buffer
    AttRefMsgPayload attMsgBuffer;
    // zero output message
    attMsgBuffer = this->attRefOutMsg.zeroMsgPayload;

    // compute direction cosine matrix [RN]
    double RN[3][3];
    MRP2C(sigma_RN, RN);

    v3Copy(sigma_RN, attMsgBuffer.sigma_RN);
    v3tMultM33(omega_RN_R, RN, attMsgBuffer.omega_RN_N);
    v3tMultM33(omegaDot_RN_R, RN, attMsgBuffer.domega_RN_N);

    // write output attitude reference message
    this->attRefOutMsg.write(&attMsgBuffer, this->moduleID, CurrentSimNanos);

    /* Write to the C-wrapped output messages */
    AttRefMsg_C_write(&attMsgBuffer, &this->attRefOutMsgC, this->moduleID, CurrentSimNanos);

    return;
}

/*! This method reads the input messages in from the system and sets the
 appropriate parameters

 */
void ConstrainedAttitudeManeuver::ReadInputs()
{
    this->inputsReady = false;
    clearGrid();
    this->constraints = {};

    if (!this->scStateInMsg.isLinked() || !this->scStateInMsg.isWritten()) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: scStateInMsg must be linked and written before reset.");
    }
    if (!this->vehicleConfigInMsg.isLinked() || !this->vehicleConfigInMsg.isWritten()) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: vehicleConfigInMsg must be linked and written before reset.");
    }
    this->scStateMsgBuffer = this->scStateInMsg();
    this->vehicleConfigMsgBuffer = this->vehicleConfigInMsg();
    const bool useOut = this->keepOutConfigured();
    const bool useIn = this->keepInConfigured();
    validateActiveCones(this->boresights, useOut, useIn);
    if (useOut && !this->keepOutCelBodyInMsg.isWritten()) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: keepOutCelBodyInMsg has not been written.");
    }
    if (useIn && !this->keepInCelBodyInMsg.isWritten()) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: keepInCelBodyInMsg has not been written.");
    }
    const auto setDirection = [this](double bodyPosition[3], double direction[3], const char* name) {
        double relative[3]; // [m]
        v3Subtract(bodyPosition, this->scStateMsgBuffer.r_BN_N, relative);
        normalizeDirection(relative, name, "line of sight");
        v3Copy(relative, direction);
    };
    if (useOut) {
        this->keepOutCelBodyMsgBuffer = this->keepOutCelBodyInMsg();
        setDirection(
            this->keepOutCelBodyMsgBuffer.PositionVector, this->constraints.keepOutDir_N, "keepOutCelBodyInMsg");
    }
    if (useIn) {
        this->keepInCelBodyMsgBuffer = this->keepInCelBodyInMsg();
        setDirection(this->keepInCelBodyMsgBuffer.PositionVector, this->constraints.keepInDir_N, "keepInCelBodyInMsg");
    }
    this->constraints.keepOut = useOut;
    this->constraints.keepIn = useIn;
    if (!useOut && !useIn) {
        bskLogger.bskLog(BSK_WARNING, "ConstrainedAttitudeManeuver: no pointing constraints are active.");
    }
    this->inputsReady = true;
}

void ConstrainedAttitudeManeuver::requireInputs()
{
    if (!this->inputsReady) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: a successful ReadInputs() is required.");
    }
}

void ConstrainedAttitudeManeuver::invalidateFit()
{
    static const OutputDataSet emptyOutput;
    this->Output = emptyOutput;
    this->pathCost = std::numeric_limits<double>::quiet_NaN();
    this->referenceReady = false;
}

void ConstrainedAttitudeManeuver::invalidateWaypoints()
{
    static const InputDataSet emptyInput;
    this->Input = emptyInput;
    invalidateFit();
}

void ConstrainedAttitudeManeuver::clearPath()
{
    if (this->path.N == 0) {
        // An empty list can still have manually supplied spline inputs.
        invalidateWaypoints();
    }
    this->path.clear();
}

void ConstrainedAttitudeManeuver::clearGrid()
{
    clearPath();
    this->NodesMap.clear();
    std::fill(std::begin(this->keyS), std::end(this->keyS), invalidGridIndex);
    std::fill(std::begin(this->keyG), std::end(this->keyG), invalidGridIndex);
}

/*! @brief Rebuild the MRP grid and connect the free neighboring nodes.
 @param startNode Node supplying the initial attitude.
 @param goalNode Node supplying the target attitude.
 @note Requires a refinement level from 2 through 20, distinct endpoint nodes and a constraint-compliant goal.
 */
void ConstrainedAttitudeManeuver::GenerateGrid(Node startNode, Node goalNode)
{
    clearGrid();
    if (this->constraints.keepOut != this->keepOutConfigured() ||
        this->constraints.keepIn != this->keepInConfigured()) {
        bskLogger.bskError(
            "ConstrainedAttitudeManeuver: constraint activation changed. Call ReadInputs() before GenerateGrid().");
    }
    int N = this->N;
    double u[20];
    if (N < 2 || N > static_cast<int>(std::size(u))) {
        bskLogger.bskError(
            "ConstrainedAttitudeManeuver: grid refinement N must be between 2 and %zu (received %d).", std::size(u), N);
    }
    if (!cArray2EigenMRPd(goalNode.sigma_BN).coeffs().allFinite()) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: a finite goal attitude is required (sigma_BN_goal at reset).");
    }
    Node start(startNode.sigma_BN, this->constraints, this->boresights);
    Node goal(goalNode.sigma_BN, this->constraints, this->boresights);
    if (!start.isFree) {
        bskLogger.bskLog(BSK_WARNING,
                         "ConstrainedAttitudeManeuver: the initial attitude of the S/C is not constraint-compliant.");
    }
    for (int n = 0; n < N; n++) {
        u[n] = n / ((double)N - 1);
    }

    // add internal nodes (|sigma_BN| < 1)
    int indices[3], mirrorIndices[8][3];
    double sigma_BN[3];
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                if (pow(u[i], 2) + pow(u[j], 2) + pow(u[k], 2) <= 1) {
                    indices[0] = i;
                    indices[1] = j;
                    indices[2] = k;
                    mirrorFunction(indices, mirrorIndices);
                    for (int m = 0; m < 8; m++) {
                        if (this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(mirrorIndices[m][2]) == 0) {
                            for (int p = 0; p < 3; p++) {
                                if (indices[p] != 0) {
                                    sigma_BN[p] = mirrorIndices[m][p] / indices[p] * u[indices[p]];
                                } else {
                                    sigma_BN[p] = 0;
                                }
                            }
                            this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] =
                                Node(sigma_BN, this->constraints, this->boresights);
                        }
                    }
                }
            }
        }
    }
    // add missing boundary nodes (|sigma_BN| = 1)
    for (int i = 0; i < N - 1; i++) {
        for (int j = 0; j < N - 1; j++) {
            for (int k = 0; k < N - 1; k++) {
                if (this->NodesMap[i][j].count(k) == 1) {
                    if (this->NodesMap[i][j][k].isBoundary == false) {
                        // along i direction
                        if (this->NodesMap[i + 1][j].count(k) == 0) {
                            indices[0] = i + 1;
                            indices[1] = j;
                            indices[2] = k;
                            mirrorFunction(indices, mirrorIndices);
                            for (int m = 0; m < 8; m++) {
                                if (this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(
                                        mirrorIndices[m][2]) == 0) {
                                    sigma_BN[0] =
                                        mirrorIndices[m][0] / indices[0] * pow(1 - pow(u[j], 2) - pow(u[k], 2), 0.5);
                                    if (indices[1] != 0) {
                                        sigma_BN[1] = mirrorIndices[m][1] / indices[1] * u[indices[1]];
                                    } else {
                                        sigma_BN[1] = 0;
                                    }
                                    if (indices[2] != 0) {
                                        sigma_BN[2] = mirrorIndices[m][2] / indices[2] * u[indices[2]];
                                    } else {
                                        sigma_BN[2] = 0;
                                    }
                                    this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] =
                                        Node(sigma_BN, this->constraints, this->boresights);
                                }
                            }
                        }
                        // along j direction
                        if (this->NodesMap[i][j + 1].count(k) == 0) {
                            indices[0] = i;
                            indices[1] = j + 1;
                            indices[2] = k;
                            mirrorFunction(indices, mirrorIndices);
                            for (int m = 0; m < 8; m++) {
                                if (this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(
                                        mirrorIndices[m][2]) == 0) {
                                    if (indices[0] != 0) {
                                        sigma_BN[0] = mirrorIndices[m][0] / indices[0] * u[indices[0]];
                                    } else {
                                        sigma_BN[0] = 0;
                                    }
                                    sigma_BN[1] =
                                        mirrorIndices[m][1] / indices[1] * pow(1 - pow(u[i], 2) - pow(u[k], 2), 0.5);
                                    if (indices[2] != 0) {
                                        sigma_BN[2] = mirrorIndices[m][2] / indices[2] * u[indices[2]];
                                    } else {
                                        sigma_BN[2] = 0;
                                    }
                                    this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] =
                                        Node(sigma_BN, this->constraints, this->boresights);
                                }
                            }
                        }
                        // along k direction
                        if (this->NodesMap[i][j].count(k + 1) == 0) {
                            indices[0] = i;
                            indices[1] = j;
                            indices[2] = k + 1;
                            mirrorFunction(indices, mirrorIndices);
                            for (int m = 0; m < 8; m++) {
                                if (this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]].count(
                                        mirrorIndices[m][2]) == 0) {
                                    if (indices[0] != 0) {
                                        sigma_BN[0] = mirrorIndices[m][0] / indices[0] * u[indices[0]];
                                    } else {
                                        sigma_BN[0] = 0;
                                    }
                                    if (indices[1] != 0) {
                                        sigma_BN[1] = mirrorIndices[m][1] / indices[1] * u[indices[1]];
                                    } else {
                                        sigma_BN[1] = 0;
                                    }
                                    sigma_BN[2] =
                                        mirrorIndices[m][2] / indices[2] * pow(1 - pow(u[i], 2) - pow(u[j], 2), 0.5);
                                    this->NodesMap[mirrorIndices[m][0]][mirrorIndices[m][1]][mirrorIndices[m][2]] =
                                        Node(sigma_BN, this->constraints, this->boresights);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    // link nodes to adjacent neighbors
    int neighbors[26][3];
    for (std::map<int, std::map<int, std::map<int, Node>>>::iterator it1 = this->NodesMap.begin();
         it1 != this->NodesMap.end();
         it1++) {
        for (std::map<int, std::map<int, Node>>::iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++) {
            for (std::map<int, Node>::iterator it3 = it2->second.begin(); it3 != it2->second.end(); it3++) {
                indices[0] = it1->first;
                indices[1] = it2->first;
                indices[2] = it3->first;
                neighboringNodes(indices, neighbors);
                for (int n = 0; n < 26; n++) {
                    if (this->NodesMap[neighbors[n][0]][neighbors[n][1]].count(neighbors[n][2]) == 1) {
                        if (this->NodesMap[indices[0]][indices[1]][indices[2]].isFree &&
                            this->NodesMap[neighbors[n][0]][neighbors[n][1]][neighbors[n][2]].isFree) {
                            this->NodesMap[indices[0]][indices[1]][indices[2]].appendNeighbor(
                                &this->NodesMap[neighbors[n][0]][neighbors[n][1]][neighbors[n][2]]);
                        }
                    }
                }
            }
        }
    }
    // link boundary nodes to neighbors of shadow set
    bool flag;
    for (std::map<int, std::map<int, std::map<int, Node>>>::iterator it1 = this->NodesMap.begin();
         it1 != this->NodesMap.end();
         it1++) {
        for (std::map<int, std::map<int, Node>>::iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++) {
            for (std::map<int, Node>::iterator it3 = it2->second.begin(); it3 != it2->second.end(); it3++) {
                if (it3->second.isBoundary && it3->second.isFree) {
                    int i, j, k;
                    i = it1->first;
                    j = it2->first;
                    k = it3->first;
                    for (int n = 0; n < this->NodesMap[-i][-j][-k].neighborCount; n++) {
                        flag = true;
                        for (int m = 0; m < this->NodesMap[i][j][k].neighborCount; m++) {
                            if (this->NodesMap[-i][-j][-k].neighbors[n] == this->NodesMap[i][j][k].neighbors[m]) {
                                flag = false;
                            }
                        }
                        if (flag == true) {
                            it3->second.appendNeighbor(this->NodesMap[-i][-j][-k].neighbors[n]);
                        }
                    }
                }
            }
        }
    }

    // add start and goal node to grid and connecting them to the neighboring nodes:
    double ds = std::numeric_limits<double>::infinity(); // [-] MRP distance
    double dg = std::numeric_limits<double>::infinity(); // [-] MRP distance
    double d1, d2;
    int keyS[3] = {};
    int keyG[3] = {};
    bool foundStart = false;
    bool foundGoal = false;
    for (std::map<int, std::map<int, std::map<int, Node>>>::iterator it1 = this->NodesMap.begin();
         it1 != this->NodesMap.end();
         it1++) {
        for (std::map<int, std::map<int, Node>>::iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++) {
            for (std::map<int, Node>::iterator it3 = it2->second.begin(); it3 != it2->second.end(); it3++) {
                if (it3->second.isFree) {
                    d1 = distance(start, it3->second);
                    if (foundStart && abs(d1 - ds) < 1e-6) {
                        if (v3Norm(it3->second.sigma_BN) < v3Norm(NodesMap[keyS[0]][keyS[1]][keyS[2]].sigma_BN)) {
                            ds = d1;
                            keyS[0] = it1->first;
                            keyS[1] = it2->first;
                            keyS[2] = it3->first;
                        }
                    } else {
                        if (d1 < ds) {
                            foundStart = true;
                            ds = d1;
                            keyS[0] = it1->first;
                            keyS[1] = it2->first;
                            keyS[2] = it3->first;
                        }
                    }
                    d2 = distance(goal, it3->second);
                    if (foundGoal && abs(d2 - dg) < 1e-6) {
                        if (v3Norm(it3->second.sigma_BN) < v3Norm(NodesMap[keyG[0]][keyG[1]][keyG[2]].sigma_BN)) {
                            dg = d2;
                            keyG[0] = it1->first;
                            keyG[1] = it2->first;
                            keyG[2] = it3->first;
                        }
                    } else {
                        if (d2 < dg) {
                            foundGoal = true;
                            dg = d2;
                            keyG[0] = it1->first;
                            keyG[1] = it2->first;
                            keyG[2] = it3->first;
                        }
                    }
                }
            }
        }
    }
    if (!foundStart || !foundGoal) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: no constraint-free grid node could be selected for the start "
                           "and goal attitudes.");
    }
    if (!goal.isFree) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: the target attitude violates a pointing constraint.");
    }
    // Only antipodal boundary samples represent the same grid attitude.
    const auto alignBoundaryKey = [this](Node& endpoint, int (&key)[3]) {
        if (!endpoint.isBoundary) {
            return;
        }
        Node* sample = gridEndpoint(key);
        if (sample->isBoundary && v3Dot(sample->sigma_BN, endpoint.sigma_BN) < 0.0) {
            const int oppositeKey[3] = { -key[0], -key[1], -key[2] };
            const Node* opposite = gridEndpoint(oppositeKey);
            if (opposite->isBoundary && opposite->isFree) {
                std::copy(std::begin(oppositeKey), std::end(oppositeKey), key);
            }
        }
    };
    alignBoundaryKey(start, keyS);
    alignBoundaryKey(goal, keyG);
    Node& gridStart = *gridEndpoint(keyS);
    Node& gridGoal = *gridEndpoint(keyG);
    const bool sameNode = &gridStart == &gridGoal;
    if (!sameNode && distance(start, goal) <= 4 * std::numeric_limits<double>::epsilon()) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: start and goal represent the same attitude.");
    }
    const bool antipodalBoundary = gridStart.isBoundary && gridGoal.isBoundary && keyS[0] == -keyG[0] &&
                                   keyS[1] == -keyG[1] && keyS[2] == -keyG[2];
    if (sameNode || antipodalBoundary) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: start and goal attitudes map to the same grid node. "
                           "For distinct attitudes, try a finer grid.");
    }
    for (int n = 0; n < gridStart.neighborCount; n++) {
        start.appendNeighbor(gridStart.neighbors[n]);
    }
    gridStart = start;
    for (int n = 0; n < gridGoal.neighborCount; n++) {
        Node* neighbor = gridGoal.neighbors[n];
        auto end = neighbor->neighbors + neighbor->neighborCount;
        if (std::find(neighbor->neighbors, end, &gridGoal) == end) {
            neighbor->appendNeighbor(&gridGoal);
        }
    }
    gridGoal = goal;
    this->keyS[0] = keyS[0];
    this->keyS[1] = keyS[1];
    this->keyS[2] = keyS[2];
    this->keyG[0] = keyG[0];
    this->keyG[1] = keyG[1];
    this->keyG[2] = keyG[2];
}

/*! This method is used inside A* to track the path from goal to start, order it from start to goal and store in class
   variable path

 */
void ConstrainedAttitudeManeuver::backtrack(Node* p)
{
    if (p == &NodesMap[this->keyS[0]][this->keyS[1]][this->keyS[2]]) {
        this->path.append(p);
        return;
    } else {
        backtrack(p->backPointer);
        this->path.append(p);
        return;
    }
}

/*! @brief Look up an endpoint in the generated grid.
 @param key Three grid indices identifying the endpoint.
 @return Pointer to the stored endpoint node.
 */
Node* ConstrainedAttitudeManeuver::gridEndpoint(const int key[3])
{
    try {
        return &this->NodesMap.at(key[0]).at(key[1]).at(key[2]);
    } catch (const std::out_of_range&) {
        bskLogger.bskError(
            "ConstrainedAttitudeManeuver: A* requires a generated grid with valid start and goal nodes.");
    }
}

/*! This method applies standard distance-based A* to find a valid path

 */
void ConstrainedAttitudeManeuver::AStar()
{
    clearPath();
    Node* const start = gridEndpoint(this->keyS);
    Node* const goal = gridEndpoint(this->keyG);
    for (std::map<int, std::map<int, std::map<int, Node>>>::iterator it1 = NodesMap.begin(); it1 != NodesMap.end();
         it1++) {
        for (std::map<int, std::map<int, Node>>::iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++) {
            for (std::map<int, Node>::iterator it3 = it2->second.begin(); it3 != it2->second.end(); it3++) {
                it3->second.heuristic = distance(it3->second, *goal);
            }
        }
    }
    int Nmax = 4 * this->N * this->N;
    int n = 0;
    double p;
    Node* key;
    NodeList O, C;
    O.append(start);

    while (O.N > 0 && O.list[0] != goal && n < Nmax) {
        n += 1;
        C.append(O.list[0]);

        for (int k = 0; k < O.list[0]->neighborCount; k++) {
            key = O.list[0]->neighbors[k];

            if (C.contains(key) == false) {
                p = key->heuristic + distance(*O.list[0], *key) + O.list[0]->priority - O.list[0]->heuristic;
                if (O.contains(key)) {
                    if (p < key->priority) {
                        key->priority = p;
                        key->backPointer = O.list[0];
                    }
                } else {
                    key->priority = p;
                    key->backPointer = O.list[0];
                    O.append(key);
                }
            }
        }

        O.pop(0);
        O.sort();
    }

    if (O.N == 0 || O.list[0] != goal) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: distance-based A* did not reach the goal (%s).",
                           O.N == 0 ? "open list exhausted" : "iteration limit reached");
    }
    backtrack(goal);

    // Uncomment to print path node coordinates
    /*
    for (int n = 0; n < this->path.N; n++) {
        std::cout << this->path.list[n]->sigma_BN[0] << " " << this->path.list[n]->sigma_BN[1] << " " <<
    this->path.list[n]->sigma_BN[2] << "\n";
    }*/
}

/*! @brief Require a finite, positive rate for maneuver timing. */
void ConstrainedAttitudeManeuver::validateManeuverRate()
{
    if (!std::isfinite(this->avgOmega) || this->avgOmega <= 0.0) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: avgOmega must be finite and positive.");
    }
}

/*! This method applies the effort-based A* to find a valid path

 */
void ConstrainedAttitudeManeuver::effortBasedAStar()
{
    clearPath();
    Node* const start = gridEndpoint(this->keyS);
    Node* const goal = gridEndpoint(this->keyG);
    requireInputs();
    validateManeuverRate();
    int Nmax = 100;
    int n = 0;
    double p;
    Node* key;
    NodeList O, C;
    O.append(start);

    while (O.N > 0 && O.list[0] != goal && n < Nmax) {
        n += 1;
        // std::cout << "N = " << n << "\n"; // uncomment to show the number of nodes explored
        C.append(O.list[0]);

        for (int k = 0; k < O.list[0]->neighborCount; k++) {

            key = O.list[0]->neighbors[k];

            if (C.contains(key) == false) {
                try {
                    backtrack(O.list[0]);
                    this->path.append(key);
                    if (key != goal) {
                        this->path.append(goal);
                    }
                    if (prepareWaypoints() != nullptr) {
                        clearPath();
                        continue;
                    }
                    spline();
                    p = effortEvaluation();
                } catch (...) {
                    clearPath();
                    throw;
                }
                clearPath();
                if (!std::isfinite(p)) {
                    continue;
                }
                if (O.contains(key)) {
                    if (p < key->priority) {
                        key->priority = p;
                        key->backPointer = O.list[0];
                    }
                } else {
                    key->priority = p;
                    key->backPointer = O.list[0];
                    O.append(key);
                }
            }
        }

        O.pop(0);
        O.sort();
    }

    if (O.N == 0 || O.list[0] != goal) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: effort-based A* did not reach the goal (%s).",
                           O.N == 0 ? "open list exhausted" : "iteration limit reached");
    }
    backtrack(goal);

    // Uncomment to print path node coordinates
    /*
    std::cout << "Waypoints: \n";
    for (int n = 0; n < this->path.N; n++) {
        std::cout << this->path.list[n]->sigma_BN[0] << " " << this->path.list[n]->sigma_BN[1] << " " <<
    this->path.list[n]->sigma_BN[2] << "\n";
    } */
}

/*! @brief Prepare the current path for spline fitting. */
void ConstrainedAttitudeManeuver::pathHandle()
{
    if (const char* error = prepareWaypoints()) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: %s", error);
    }
}

/*! @brief Convert the path to time-tagged MRP waypoints.
 *  @return Null on success, or a description of why the path cannot be fitted.
 */
const char* ConstrainedAttitudeManeuver::prepareWaypoints()
{
    invalidateWaypoints();
    validateManeuverRate();
    if (this->path.N < 2) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: a trajectory requires at least two waypoints.");
    }
    Eigen::VectorXd T(this->path.N);
    T[0] = 0;
    for (int n = 0; n < this->path.N - 1; n++) {
        T[n + 1] = T[n] + 4.0 * distance(*path.list[n], *path.list[n + 1]) / this->avgOmega;
        if (!std::isfinite(T[n + 1]) || T[n + 1] <= T[n]) {
            return "path waypoint times must be finite and strictly increasing.";
        }
    }

    // For each of the two MRP sets, retain the smallest possible peak over the remaining path.
    // Ordinarily only one successor follows the short arc. At 180 degrees both signs are feasible.
    constexpr double roundoff = 4 * std::numeric_limits<double>::epsilon();
    Eigen::Array<bool, Eigen::Dynamic, 2> nextShadow(this->path.N, 2);
    Eigen::Vector2d peaks;
    Eigen::Vector4d nextEP;
    for (int n = this->path.N - 1; n >= 0; --n) {
        Eigen::Vector4d ep;
        MRP2EP(path.list[n]->sigma_BN, ep.data());
        const double norm = v3Norm(path.list[n]->sigma_BN);
        Eigen::Vector2d scores(norm, norm == 0.0 ? std::numeric_limits<double>::infinity() : 1.0 / norm);
        if (n + 1 < this->path.N) {
            for (int set = 0; set < 2; ++set) {
                const Eigen::Vector4d from = set ? -ep : ep;
                double best = std::numeric_limits<double>::infinity();
                for (int next = 0; next < 2; ++next) {
                    const Eigen::Vector4d to = next ? -nextEP : nextEP;
                    if (from.dot(to) < -roundoff) {
                        continue;
                    }
                    const double score = std::max({ scores[set], interiorPeakNorm(from, to), peaks[next] });
                    if (score < best) {
                        best = score;
                        nextShadow(n, set) = next != 0;
                    }
                }
                scores[set] = best;
            }
        }
        nextEP = ep;
        peaks = scores;
    }
    if (std::isinf(peaks.minCoeff())) {
        return "the path crosses the singularities of both MRP sets. Split the maneuver.";
    }
    // Favor a principal start on tied peaks, including equivalent inputs on the unit sphere.
    Eigen::Index largest;
    nextEP.cwiseAbs().maxCoeff(&largest);
    bool shadow = (std::abs(nextEP[0]) <= roundoff ? nextEP[largest] : nextEP[0]) < 0.0;
    if (peaks[!shadow] * (1.0 + roundoff) < peaks[shadow]) {
        shadow = !shadow;
    }
    Eigen::VectorXd X1(this->path.N);
    Eigen::VectorXd X2(this->path.N);
    Eigen::VectorXd X3(this->path.N);

    for (int n = 0; n < this->path.N; ++n) {
        Eigen::MRPd attitude = cArray2EigenMRPd(path.list[n]->sigma_BN);
        if (shadow) {
            attitude = attitude.shadow();
        }
        X1[n] = attitude.x();
        X2[n] = attitude.y();
        X3[n] = attitude.z();
        if (n + 1 < this->path.N) {
            shadow = nextShadow(n, shadow);
        }
    }

    this->Input = InputDataSet(X1, X2, X3);
    this->Input.setT(T);
    return nullptr;
}

/*! This method performs the BSpline interpolation/approximation and outputs an Output structure

 */
void ConstrainedAttitudeManeuver::spline()
{
    invalidateFit();
    const int numWaypoints = static_cast<int>(this->Input.X1.size());
    if (numWaypoints < 2) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: a spline requires at least two prepared waypoints. "
                           "Call pathHandle() after computing the path.");
    }
    requireInputs();
    if (this->Input.X2.size() != numWaypoints || this->Input.X3.size() != numWaypoints) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: spline coordinates must have one entry per waypoint.");
    }
    // pathHandle may have selected the shadow MRP set for either endpoint.
    const int last = numWaypoints - 1;
    const Eigen::MRPd start(this->Input.X1[0], this->Input.X2[0], this->Input.X3[0]);
    const Eigen::MRPd goal(this->Input.X1[last], this->Input.X2[last], this->Input.X3[last]);
    this->Input.setXDot_0(0.25 * start.Bmat() * cArray2EigenVector3d(this->scStateMsgBuffer.omega_BN_B));
    this->Input.setXDot_N(0.25 * goal.Bmat() * cArray2EigenVector3d(this->omega_BN_B_goal));
    if (this->BSplineType != 0 && this->BSplineType != 1) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: BSplineType has not been specified.");
    }
    try {
        if (numWaypoints == 2) {
            // Two attitudes and their endpoint rates uniquely determine a cubic in either fitting mode.
            interpolate(this->Input, 100, 3, &this->Output);
        } else if (this->BSplineType == 0) {
            interpolate(this->Input, 100, 4, &this->Output);
        } else {
            // A quartic needs at least five control points. The argument is the final control-point index.
            approximate(this->Input, 100, std::max(4, numWaypoints), 4, &this->Output);
        }
    } catch (...) {
        invalidateFit();
        throw;
    }
}

/*! This method computes the torque vector required at time step with index n

 */
void ConstrainedAttitudeManeuver::computeTorque(int n, double I[9], double L[3])
{
    const auto& output = this->Output;
    const auto numSamples = output.T.size();
    if (n < 0 || n >= numSamples) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: torque sample index %d is outside the fitted samples.", n);
    }
    if (output.X1.size() != numSamples || output.X2.size() != numSamples || output.X3.size() != numSamples ||
        output.XD1.size() != numSamples || output.XD2.size() != numSamples || output.XD3.size() != numSamples ||
        output.XDD1.size() != numSamples || output.XDD2.size() != numSamples || output.XDD3.size() != numSamples) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: fitted sample array lengths must match the time array.");
    }
    computeTorqueUnchecked(n, I, L);
}

void ConstrainedAttitudeManeuver::computeTorqueUnchecked(int n, double I[9], double L[3]) const
{
    double sigma[3], sigmaDot[3], sigmaDDot[3], omega[3], omegaDot[3], L1[3], L2[3], H[3];

    sigma[0] = this->Output.X1[n];
    sigma[1] = this->Output.X2[n];
    sigma[2] = this->Output.X3[n];
    sigmaDot[0] = this->Output.XD1[n];
    sigmaDot[1] = this->Output.XD2[n];
    sigmaDot[2] = this->Output.XD3[n];
    sigmaDDot[0] = this->Output.XDD1[n];
    sigmaDDot[1] = this->Output.XDD2[n];
    sigmaDDot[2] = this->Output.XDD3[n];
    dMRP2Omega(sigma, sigmaDot, omega);
    ddMRP2dOmega(sigma, sigmaDot, sigmaDDot, omegaDot);
    m33MultV3(RECAST3X3 I, omegaDot, L1);
    m33MultV3(RECAST3X3 I, omega, H);
    v3Cross(omega, H, L2);
    v3Add(L1, L2, L);
}

/*! This method computes the torque vector required at time step with index n

 */
double ConstrainedAttitudeManeuver::computeTorqueNorm(int n, double I [[maybe_unused]][9])
{
    double L[3];
    computeTorque(n, this->vehicleConfigMsgBuffer.ISCPntB_B, L);

    return v3Norm(L);
}

/*! This method integrates the control torque norm over maneuver time and returns the cost for the effort-based A*.

 */
double ConstrainedAttitudeManeuver::effortEvaluation()
{
    const auto numSamples = this->Output.T.size();
    if (numSamples < 2) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: effort evaluation requires at least two fitted samples.");
    }
    requireInputs();
    if (numSamples - 1 > std::numeric_limits<int>::max()) {
        bskLogger.bskError("ConstrainedAttitudeManeuver: fitted sample index exceeds the range of computeTorque().");
    }
    double effort = 0;
    double l_a, l_b;
    double L_a[3], L_b[3];
    // The first query validates all component lengths for the integration loop.
    computeTorque(0, this->vehicleConfigMsgBuffer.ISCPntB_B, L_a);

    for (int n = 0; n < numSamples - 1; n++) {
        computeTorqueUnchecked(n + 1, this->vehicleConfigMsgBuffer.ISCPntB_B, L_b);
        l_a = v3Norm(L_a);
        l_b = v3Norm(L_b);
        effort += (l_a + l_b) * (Output.T[n + 1] - Output.T[n]) / 2;

        v3Copy(L_b, L_a);
    }

    return effort;
}

/*! This method  allows to access the coordinates of a Node in NodesMap without swigging the C++ Map.
    It is designed to be used in the UnitTest primarily.

 */
double ConstrainedAttitudeManeuver::returnNodeCoord(int key[3], int nodeCoord)
{
    if (nodeCoord < 0 || nodeCoord > 2 || NodesMap[key[0]][key[1]].count(key[2]) == 0) {
        return 1000; // random large number that will cause the UnitTest comparison to fail
    } else {
        return this->NodesMap[key[0]][key[1]][key[2]].sigma_BN[nodeCoord];
    }
}

/*! This method  allows to access the state of a Node (free or not free) in NodesMap without swigging the C++ Map.
    It is designed to be used in the UnitTest primarily.

 */
bool ConstrainedAttitudeManeuver::returnNodeState(int key[3])
{
    if (NodesMap[key[0]][key[1]].count(key[2]) != 0) {
        return this->NodesMap[key[0]][key[1]][key[2]].isFree;
    } else {
        return false;
    }
}

/*! This method allows to access the coordinates of path Nodes in without swigging NodesList C++.
    It is designed to be used in the UnitTest primarily.

 */
double ConstrainedAttitudeManeuver::returnPathCoord(int index, int nodeCoord)
{
    if (!validListIndex(this->path, index) || nodeCoord < 0 || nodeCoord > 2) {
        return 1000; // random large number that will cause the UnitTest comparison to fail
    } else {
        return this->path.list[index]->sigma_BN[nodeCoord];
    }
}

/*! This helper function returns the coordinates of the 8 symmetrical points to a point in 3D cartesian space. */
void mirrorFunction(int indices[3], int mirrorIndices[8][3])
{
    mirrorIndices[0][0] = indices[0];
    mirrorIndices[0][1] = indices[1];
    mirrorIndices[0][2] = indices[2];
    mirrorIndices[1][0] = -indices[0];
    mirrorIndices[1][1] = indices[1];
    mirrorIndices[1][2] = indices[2];
    mirrorIndices[2][0] = indices[0];
    mirrorIndices[2][1] = -indices[1];
    mirrorIndices[2][2] = indices[2];
    mirrorIndices[3][0] = indices[0];
    mirrorIndices[3][1] = indices[1];
    mirrorIndices[3][2] = -indices[2];
    mirrorIndices[4][0] = -indices[0];
    mirrorIndices[4][1] = -indices[1];
    mirrorIndices[4][2] = indices[2];
    mirrorIndices[5][0] = -indices[0];
    mirrorIndices[5][1] = indices[1];
    mirrorIndices[5][2] = -indices[2];
    mirrorIndices[6][0] = indices[0];
    mirrorIndices[6][1] = -indices[1];
    mirrorIndices[6][2] = -indices[2];
    mirrorIndices[7][0] = -indices[0];
    mirrorIndices[7][1] = -indices[1];
    mirrorIndices[7][2] = -indices[2];
}

/*! For a set of indices, this helper function returns the indices of the 26 immediate neighbors. */
void neighboringNodes(int indices[3], int neighbors[26][3])
{
    neighbors[0][0] = indices[0] - 1;
    neighbors[0][1] = indices[1];
    neighbors[0][2] = indices[2];
    neighbors[1][0] = indices[0] + 1;
    neighbors[1][1] = indices[1];
    neighbors[1][2] = indices[2];
    neighbors[2][0] = indices[0];
    neighbors[2][1] = indices[1] - 1;
    neighbors[2][2] = indices[2];
    neighbors[3][0] = indices[0];
    neighbors[3][1] = indices[1] + 1;
    neighbors[3][2] = indices[2];
    neighbors[4][0] = indices[0];
    neighbors[4][1] = indices[1];
    neighbors[4][2] = indices[2] - 1;
    neighbors[5][0] = indices[0];
    neighbors[5][1] = indices[1];
    neighbors[5][2] = indices[2] + 1;
    neighbors[6][0] = indices[0] - 1;
    neighbors[6][1] = indices[1] - 1;
    neighbors[6][2] = indices[2];
    neighbors[7][0] = indices[0] + 1;
    neighbors[7][1] = indices[1] - 1;
    neighbors[7][2] = indices[2];
    neighbors[8][0] = indices[0] - 1;
    neighbors[8][1] = indices[1] + 1;
    neighbors[8][2] = indices[2];
    neighbors[9][0] = indices[0] + 1;
    neighbors[9][1] = indices[1] + 1;
    neighbors[9][2] = indices[2];
    neighbors[10][0] = indices[0] - 1;
    neighbors[10][1] = indices[1];
    neighbors[10][2] = indices[2] - 1;
    neighbors[11][0] = indices[0] + 1;
    neighbors[11][1] = indices[1];
    neighbors[11][2] = indices[2] - 1;
    neighbors[12][0] = indices[0] - 1;
    neighbors[12][1] = indices[1];
    neighbors[12][2] = indices[2] + 1;
    neighbors[13][0] = indices[0] + 1;
    neighbors[13][1] = indices[1];
    neighbors[13][2] = indices[2] + 1;
    neighbors[14][0] = indices[0];
    neighbors[14][1] = indices[1] - 1;
    neighbors[14][2] = indices[2] - 1;
    neighbors[15][0] = indices[0];
    neighbors[15][1] = indices[1] + 1;
    neighbors[15][2] = indices[2] - 1;
    neighbors[16][0] = indices[0];
    neighbors[16][1] = indices[1] - 1;
    neighbors[16][2] = indices[2] + 1;
    neighbors[17][0] = indices[0];
    neighbors[17][1] = indices[1] + 1;
    neighbors[17][2] = indices[2] + 1;
    neighbors[18][0] = indices[0] - 1;
    neighbors[18][1] = indices[1] - 1;
    neighbors[18][2] = indices[2] - 1;
    neighbors[19][0] = indices[0] + 1;
    neighbors[19][1] = indices[1] - 1;
    neighbors[19][2] = indices[2] - 1;
    neighbors[20][0] = indices[0] - 1;
    neighbors[20][1] = indices[1] + 1;
    neighbors[20][2] = indices[2] - 1;
    neighbors[21][0] = indices[0] - 1;
    neighbors[21][1] = indices[1] - 1;
    neighbors[21][2] = indices[2] + 1;
    neighbors[22][0] = indices[0] + 1;
    neighbors[22][1] = indices[1] + 1;
    neighbors[22][2] = indices[2] - 1;
    neighbors[23][0] = indices[0] + 1;
    neighbors[23][1] = indices[1] - 1;
    neighbors[23][2] = indices[2] + 1;
    neighbors[24][0] = indices[0] - 1;
    neighbors[24][1] = indices[1] + 1;
    neighbors[24][2] = indices[2] + 1;
    neighbors[25][0] = indices[0] + 1;
    neighbors[25][1] = indices[1] + 1;
    neighbors[25][2] = indices[2] + 1;
}

/*! This function implements the MRP cartesian distance between 2 nodes.  */
double distance(Node n1, Node n2)
{
    double n1n, n2n;
    double D, d[4];
    double dn[3], n1s[3], n2s[3];
    n1n = v3Norm(n1.sigma_BN);
    n2n = v3Norm(n2.sigma_BN);
    v3Subtract(n1.sigma_BN, n2.sigma_BN, dn);
    d[0] = v3Norm(dn);
    if (n2n > 1e-8) {
        MRPshadow(n2.sigma_BN, n2s);
        v3Subtract(n1.sigma_BN, n2s, dn);
        d[1] = v3Norm(dn);
    } else {
        d[1] = d[0];
    }
    if (n1n > 1e-8) {
        MRPshadow(n1.sigma_BN, n1s);
        v3Subtract(n2.sigma_BN, n1s, dn);
        d[2] = v3Norm(dn);
    } else {
        d[2] = d[0];
    }
    if (n1n > 1e-8 && n2n > 1e-8) {
        v3Subtract(n1s, n2s, dn);
        d[3] = v3Norm(dn);
    } else {
        d[3] = d[0];
    }
    D = d[0];
    for (int i = 1; i < 4; i++) {
        if (d[i] < D) {
            D = d[i];
        }
    }
    return D;
}
