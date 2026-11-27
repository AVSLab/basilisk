/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef CONSTRAINT_DYNAMIC_EFFECTOR_H
#define CONSTRAINT_DYNAMIC_EFFECTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/ConstDynEffectorMsgPayload.h"
#include "architecture/msgPayloadDefC/DeviceStatusMsgPayload.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>
#include <vector>

/*! Struct containing parent classification variables. */
struct parentID
{
    int idx;                //!< index of effector's parent within parent type (ex. hub #1 or state eff #2)
    std::string parentType; //!< type of parent attached to (hub or state effector)
};

/*! @brief constraint dynamic effector class */
class ConstraintDynamicEffector final
    : public SysModel
    , public DynamicEffector
{
  public:
    ConstraintDynamicEffector();
    ~ConstraintDynamicEffector();
    void Reset(uint64_t CurrentSimNanos) override;
    /** @brief Link the next hub endpoint.
     * @param states State manager of the attached spacecraft.
     */
    void linkInStates(DynParamManager& states) override;
    /** @brief Link the next articulated endpoint.
     * @param properties State manager containing the parent's inertial properties.
     */
    void linkInProperties(DynParamManager& properties) override;
    /** @brief Evaluate the load for the next endpoint in a direct force query.
     * @param integTime [s] Integration time.
     * @param timeStep [s] Integration step.
     */
    void computeForceTorque(double integTime, double timeStep) override;
    void UpdateState(uint64_t CurrentSimNanos) override;
    void writeOutputStateMessage(uint64_t CurrentClock);
    void computeFilteredForce(uint64_t CurrentClock);
    void computeFilteredTorque(uint64_t CurrentClock);
    void readInputMessage();

    /** setter for `r_P2P1_B1Init` initial spacecraft separation */
    void setR_P2P1_B1Init(Eigen::Vector3d r_P2P1_B1Init);
    /** setter for `r_P1B1_B1` connection point position on spacecraft 1 */
    void setR_P1B1_B1(Eigen::Vector3d r_P1B1_B1);
    /** setter for `r_P2B2_B2` connection point position on spacecraft 2 */
    void setR_P2B2_B2(Eigen::Vector3d r_P2B2_B2);
    /** setter for `sigma_B2B1_Init` initial spacecraft relative attitude */
    void setSigma_B2B1Init(Eigen::MRPd sigma_B2B1Init);
    /** setter for `alpha` gain tuning parameter */
    void setAlpha(double alpha);
    /** setter for `beta` gain tuning parameter */
    void setBeta(double beta);
    /** setter for `k_d` gain */
    void setK_d(double k_d);
    /** setter for `c_d` gain */
    void setC_d(double c_d);
    /** setter for `k_a` gain */
    void setK_a(double k_a);
    /** setter for `c_a` gain */
    void setC_a(double c_a);
    /** setter for `a,b,s,c,d,e` coefficients of low pass filter */
    void setFilter_Data(double wc, double h, double k);
    /** setter for `stateNameOfSigma` property */
    void setStateNameOfSigma(std::string value) override;
    /** setter for `stateNameOfOmega` property */
    void setStateNameOfOmega(std::string value) override;
    /** setter for `propName_inertialPosition` property */
    void setPropName_inertialPosition(std::string value) override;
    /** getter for `propName_inertialPosition` property */
    const std::vector<std::string> getPropName_inertialPosition() const { return this->propName_inertialPosition; }
    /** setter for `propName_inertialVelocity` property */
    void setPropName_inertialVelocity(std::string value) override;
    /** getter for `propName_inertialVelocity` property */
    const std::vector<std::string> getPropName_inertialVelocity() const { return this->propName_inertialVelocity; }
    /** setter for `propName_inertialAttitude` property */
    void setPropName_inertialAttitude(std::string value) override;
    /** getter for `propName_inertialAttitude` property */
    const std::vector<std::string> getPropName_inertialAttitude() const { return this->propName_inertialAttitude; }
    /** setter for `propName_inertialAngVelocity` property */
    void setPropName_inertialAngVelocity(std::string value) override;
    /** getter for `propName_inertialAngVelocity` property */
    const std::vector<std::string> getPropName_inertialAngVelocity() const
    {
        return this->propName_inertialAngVelocity;
    }

    /** getter for `r_P2P1_B1Init` initial spacecraft separation */
    Eigen::Vector3d getR_P2P1_B1Init() const { return this->r_P2P1_B1Init; };
    /** getter for `r_P1B1_B1` connection point position on spacecraft 1 */
    Eigen::Vector3d getR_P1B1_B1() const { return this->r_P1B1_B1; };
    /** getter for `r_P2B2_B2` connection point position on spacecraft 2 */
    Eigen::Vector3d getR_P2B2_B2() const { return this->r_P2B2_B2; };
    /** getter for `alpha` gain tuning parameter */
    double getAlpha() const { return this->alpha; };
    /** getter for `beta` gain tuning parameter */
    double getBeta() const { return this->beta; };
    /** getter for `k_d` gain */
    double getK_d() const { return this->k_d; };
    /** getter for `c_d` gain */
    double getC_d() const { return this->c_d; };
    /** getter for `k_a` gain */
    double getK_a() const { return this->k_a; };
    /** getter for `c_a` gain */
    double getC_a() const { return this->c_a; };

    Message<ConstDynEffectorMsgPayload>
        constraintElements; //!< output message with constraint force and torque on connected s/c
    ReadFunctor<DeviceStatusMsgPayload> effectorStatusInMsg; //!< input message to toggle effector on/off
    uint64_t effectorStatus = 1;                             //!< effector toggle on/off (1-active, 0-inactive)

  private:
#ifndef SWIG
    // Each parent registers and requests loads through the same endpoint, independent of traversal order.
    struct Endpoint final : DynamicEffector
    {
        Endpoint(ConstraintDynamicEffector& owner, std::size_t index)
            : owner(owner)
            , index(index)
        {
        }
        void linkInStates(DynParamManager& states) override;
        void linkInProperties(DynParamManager& states) override;
        void computeForceTorque(double time, double timeStep) override;
        void linkTranslation(DynParamManager& states);
        Eigen::MRPd attitude() const;
        Eigen::Vector3d angularVelocity() const;

        ConstraintDynamicEffector& owner;
        const std::size_t index;
        const Eigen::MatrixXd* position = nullptr;
        const Eigen::MatrixXd* velocity = nullptr;
        const StateData* hubPosition = nullptr;
        const StateData* hubVelocity = nullptr;
        const StateData* hubAttitude = nullptr;
        const StateData* hubRate = nullptr;
        const Eigen::MatrixXd* branchAttitude = nullptr;
        const Eigen::MatrixXd* branchRate = nullptr;
        const DynParamManager::InertialFrame* frame = nullptr;
    };
    DynamicEffector* forAttachment(bool hub) override;
    std::array<Endpoint, 2> endpoints{ { { *this, 0 }, { *this, 1 } } };
#endif
    void evaluateConstraint();
    void writeConstraintMessage(uint64_t epoch);
    std::optional<uint64_t> telemetryEpoch(uint64_t clock) const;
    bool bothLinked() const;
    void validateConfiguration(); //!< Validate the user-supplied gain configuration
    void initializeGains();       //!< Refresh derived gains while preserving explicit individual gains

    std::size_t attachmentCount = 0; //!< Number of configured parents.
    std::size_t directEndpoint = 0;  //!< Alternating endpoint for direct computeForceTorque calls.

    // Constraint length and direction
    Eigen::Vector3d r_P1B1_B1 =
        Eigen::Vector3d::Zero(); //!< [m] position vector from spacecraft 1 hub to its connection point P1
    Eigen::Vector3d r_P2B2_B2 =
        Eigen::Vector3d::Zero(); //!< [m] position vector from spacecraft 2 hub to its connection point P2
    Eigen::Vector3d r_P2P1_B1Init = Eigen::Vector3d::Zero(); //!< [m] prescribed position vector from spacecraft 1
                                                             //!< connection point to spacecraft 2 connection point

    // Constraint attitude
    Eigen::Matrix3d dcm_B2B1Init = Eigen::Matrix3d::Identity(); //!< attitude constraint violation

    // Gains for PD controller
    double alpha = 0.0;       //!< [-] Numerical tuning parameter whose square sets proportional gains in SI units
    double beta = 0.0;        //!< [-] Numerical tuning parameter doubled to set derivative gains in SI units
    double k_d = 0.0;         //!< [N/m] direction constraint proportional gain
    double c_d = 0.0;         //!< [N*s/m] direction constraint derivative gain
    double k_a = 0.0;         //!< [N*m] attitude constraint proportional gain multiplying dimensionless MRP error
    double c_a = 0.0;         //!< [N*m*s] attitude constraint derivative gain multiplying the MRP error rate
    bool k_dExplicit = false; //!< Whether setK_d supplied the direction proportional gain
    bool c_dExplicit = false; //!< Whether setC_d supplied the direction derivative gain
    bool k_aExplicit = false; //!< Whether setK_a supplied the attitude proportional gain
    bool c_aExplicit = false; //!< Whether setC_a supplied the attitude derivative gain
    double a = 0.0;           //!< coefficient in numerical low pass filter
    double b = 0.0;           //!< coefficient in numerical low pass filter
    double c = 0.0;           //!< coefficient in numerical low pass filter
    double d = 0.0;           //!< coefficient in numerical low pass filter
    double e = 0.0;           //!< coefficient in numerical low pass filter

    struct LoadSample
    {
        double magnitude = 0.0;
        double filtered = 0.0;
    };
    struct LoadFilter
    {
        std::optional<uint64_t> epoch; //!< [ns] Timestamp of the current sample.
        LoadSample current, previous, previous2;
    };
    void updateFilter(LoadFilter& filter, uint64_t epoch, double magnitude) const;
    LoadFilter forceFilter;   //!< [N] Constraint force magnitude.
    LoadFilter torque1Filter; //!< [N*m] Endpoint 1 torque magnitude.
    LoadFilter torque2Filter; //!< [N*m] Endpoint 2 torque magnitude.

    uint64_t appliedStatus = 1;                 //!< Status used by the most recent dynamics evaluation.
    std::optional<uint64_t> lastPublishedEpoch; //!< [ns] Accepted epoch of the most recent constraint message.

    // Simulation variable pointers
    std::vector<std::string> stateNameOfSigma; //!< state engine name of the parent rigid body inertial attitude
    std::vector<std::string>
        stateNameOfOmega; //!< state engine name of the parent rigid body inertial angular velocity vector
    // Parent body inertial properties
    std::vector<std::string> propName_inertialPosition;    //!< property name of inertialPosition
    std::vector<std::string> propName_inertialVelocity;    //!< property name of inertialVelocity
    std::vector<std::string> propName_inertialAttitude;    //!< property name of inertialAttitude
    std::vector<std::string> propName_inertialAngVelocity; //!< property name of inertialAngVelocity
    // Constraint violations
    Eigen::Vector3d psi_N = Eigen::Vector3d::Zero(); //!< [m] direction constraint violation in inertial frame
    Eigen::Vector3d psiPrime_N =
        Eigen::Vector3d::Zero();               //!< [m/s] direction rate constraint violation in inertial frame
    Eigen::MRPd phi = Eigen::MRPd::Identity(); //!< attitude constraint violation
    Eigen::Vector3d omega_B2B1_B2 =
        Eigen::Vector3d::Zero(); //!< [rad/s] angular velocity constraint violation in spacecraft 2 body frame

    // Force and torque quantities stored to be assigned on the alternating call of computeForceTorque
    Eigen::Vector3d Fc_N = Eigen::Vector3d::Zero(); //!< [N] force applied on each spacecraft COM in the inertial frame
    Eigen::Vector3d T_B2 = Eigen::Vector3d::Zero(); //!< [N-m] torque applied on spacecraft 2 in its body frame
    Eigen::Vector3d T_B1 = Eigen::Vector3d::Zero(); //!< [N-m] torque applied on spacecraft 1 in its body frame
};

#endif /* CONSTRAINT_DYNAMIC_EFFECTOR_H */
