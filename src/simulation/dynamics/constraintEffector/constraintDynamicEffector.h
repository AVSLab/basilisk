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

#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/avsEigenMRP.h"
#include <Eigen/Dense>
#include <vector>
#include "architecture/msgPayloadDefC/ConstDynEffectorMsgPayload.h"
#include "architecture/msgPayloadDefC/DeviceStatusMsgPayload.h"
#include "architecture/messaging/messaging.h"

/*! Struct containing parent classification variables. */
struct parentID{
    int idx;  //!< index of effector's parent within parent type (ex. hub #1 or state eff #2)
    std::string parentType;  //!< type of parent attached to (hub or state effector)
};

/*! @brief constraint dynamic effector class */
class ConstraintDynamicEffector: public SysModel, public DynamicEffector {
public:
    ConstraintDynamicEffector();
    ~ConstraintDynamicEffector();
    void Reset(uint64_t CurrentSimNanos) override;
    void linkInStates(DynParamManager& states) override;
    void linkInProperties(DynParamManager& properties) override;
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
    void setFilter_Data(double wc,double h, double k);
    /** setter for `stateNameOfPosition` property */
    void setStateNameOfPosition(std::string value) override;
    /** setter for `stateNameOfVelocity` property */
    void setStateNameOfVelocity(std::string value) override;
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
    const std::vector<std::string> getPropName_inertialAngVelocity() const { return this->propName_inertialAngVelocity; }

    /** getter for `r_P2P1_B1Init` initial spacecraft separation */
    Eigen::Vector3d getR_P2P1_B1Init() const {return this->r_P2P1_B1Init;};
    /** getter for `r_P1B1_B1` connection point position on spacecraft 1 */
    Eigen::Vector3d getR_P1B1_B1() const {return this->r_P1B1_B1;};
    /** getter for `r_P2B2_B2` connection point position on spacecraft 2 */
    Eigen::Vector3d getR_P2B2_B2() const {return this->r_P2B2_B2;};
    /** getter for `alpha` gain tuning parameter */
    double getAlpha() const {return this->alpha;};
    /** getter for `beta` gain tuning parameter */
    double getBeta() const {return this->beta;};
    /** getter for `k_d` gain */
    double getK_d() const {return this->k_d;};
    /** getter for `c_d` gain */
    double getC_d() const {return this->c_d;};
    /** getter for `k_a` gain */
    double getK_a() const {return this->k_a;};
    /** getter for `c_a` gain */
    double getC_a() const {return this->c_a;};

    Message<ConstDynEffectorMsgPayload> constraintElements; //!< output message with constraint force and torque on connected s/c
    ReadFunctor<DeviceStatusMsgPayload> effectorStatusInMsg; //!< input message to record device status
    uint64_t effectorStatus=1; //!< internal variable to toggle effector status

private:

    // Counters and flags
    int scInitCounter = 0; //!< counter to kill simulation if more than two spacecraft initialized
    int scID = 1; //!< 0,1 alternating spacecraft toggle to output appropriate force/torque
    parentID parent1, parent2;
    int hubCounter = 0;
    int effectorCounter = 0;

    // Constraint length and direction
    Eigen::Vector3d r_P1B1_B1 = Eigen::Vector3d::Zero(); //!< [m] position vector from spacecraft 1 hub to its connection point P1
    Eigen::Vector3d r_P2B2_B2 = Eigen::Vector3d::Zero(); //!< [m] position vector from spacecraft 2 hub to its connection point P2
    Eigen::Vector3d r_P2P1_B1Init = Eigen::Vector3d::Zero(); //!< [m] prescribed position vector from spacecraft 1 connection point to spacecraft 2 connection point

    // Constraint attitude
    Eigen::Matrix3d dcm_B2B1Init = Eigen::Matrix3d::Identity(); //!< attitude constraint violation

    // Gains for PD controller
    double alpha = 0.0; //!< Baumgarte stabilization gain tuning variable
    double beta = 0.0; //!< Baumgarte stabilization gain tuning variable
    double k_d = 0.0; //!< direction constraint proportional gain
    double c_d = 0.0; //!< direction constraint derivative gain
    double k_a = 0.0; //!< attitude constraint proportional gain
    double c_a = 0.0; //!< attitude constraint derivative gain
    double a = 0.0; //!< coefficient in numerical low pass filter
    double b = 0.0; //!< coefficient in numerical low pass filter
    double c = 0.0; //!< coefficient in numerical low pass filter
    double d = 0.0; //!< coefficient in numerical low pass filter
    double e = 0.0; //!< coefficient in numerical low pass filter

    double F_mag_tminus2 = 0.0; //!< Magnitude of unfiltered constraint force at t-2 time step
    double F_mag_tminus1 = 0.0; //!< Magnitude of unfiltered constraint force at t-1 time step
    double F_mag_t = 0.0; //!< Magnitude of unfiltered constraint force at t time step
    double F_filtered_mag_t = 0.0; //!< Magnitude of filtered constraint force at t time step
    double F_filtered_mag_tminus1 = 0.0; //!< Magnitude of filtered constraint force at t-1 time step
    double F_filtered_mag_tminus2 = 0.0; //!< Magnitude of filtered constraint force at t-2 time step

    double T1_mag_tminus2 = 0.0; //!< Magnitude of unfiltered constraint torque on s/c 1 at t-2 time step
    double T1_mag_tminus1 = 0.0; //!< Magnitude of unfiltered constraint torque on s/c 1 at t-1 time step
    double T1_mag_t = 0.0; //!< Magnitude of unfiltered constraint torque on s/c 1 at t time step
    double T1_filtered_mag_t = 0.0; //!< Magnitude of filtered constraint torque on s/c 1 at t time step
    double T1_filtered_mag_tminus1 = 0.0; //!< Magnitude of filtered constraint torque on s/c 1 at t-1 time step
    double T1_filtered_mag_tminus2 = 0.0; //!< Magnitude of filtered constraint torque on s/c 1 at t-2 time step

    double T2_mag_tminus2 = 0.0; //!< Magnitude of unfiltered constraint torque on s/c 2 at t-2 time step
    double T2_mag_tminus1 = 0.0; //!< Magnitude of unfiltered constraint torque on s/c 2at t-1 time step
    double T2_mag_t = 0.0; //!< Magnitude of unfiltered constraint torque on s/c 2 at t time step
    double T2_filtered_mag_t = 0.0; //!< Magnitude of filtered constraint torque on s/c 2 at t time step
    double T2_filtered_mag_tminus1 = 0.0; //!< Magnitude of filtered constraint torque on s/c 2 at t-1 time step
    double T2_filtered_mag_tminus2 = 0.0; //!< Magnitude of filtered constraint torque on s/c 2 at t-2 time step

    // Simulation variable pointers
    std::vector<std::string> stateNameOfPosition;            //!< state engine name of the parent rigid body inertial position vector
    std::vector<std::string> stateNameOfVelocity;            //!< state engine name of the parent rigid body inertial velocity vector
    std::vector<std::string> stateNameOfSigma;               //!< state engine name of the parent rigid body inertial attitude
    std::vector<std::string> stateNameOfOmega;               //!< state engine name of the parent rigid body inertial angular velocity vector
    std::vector<StateData*> hubPosition;    //!< [m] parent inertial position vector
    std::vector<StateData*> hubVelocity;    //!< [m/s] parent inertial velocity vector
    std::vector<StateData*> hubSigma;       //!< parent attitude Modified Rodrigues Parameters (MRPs)
    std::vector<StateData*> hubOmega;       //!< [rad/s] parent inertial angular velocity vector

    // Parent body inertial properties
    std::vector<std::string> propName_inertialPosition;      //!< property name of inertialPosition
    std::vector<std::string> propName_inertialVelocity;      //!< property name of inertialVelocity
    std::vector<std::string> propName_inertialAttitude;      //!< property name of inertialAttitude
    std::vector<std::string> propName_inertialAngVelocity;   //!< property name of inertialAngVelocity
    std::vector<Eigen::MatrixXd*> inertialPositionProperty;  //!< [m] position relative to inertial frame
    std::vector<Eigen::MatrixXd*> inertialVelocityProperty;  //!< [m/s] velocity relative to inertial frame
    std::vector<Eigen::MatrixXd*> inertialAttitudeProperty;  //!< attitude relative to inertial frame
    std::vector<Eigen::MatrixXd*> inertialAngVelocityProperty;  //!< [rad/s] inertial angular velocity relative to inertial frame

    // Constraint violations
    Eigen::Vector3d psi_N = Eigen::Vector3d::Zero(); //!< [m] direction constraint violation in inertial frame
    Eigen::Vector3d psiPrime_N = Eigen::Vector3d::Zero(); //!< [m/s] direction rate constraint violation in inertial frame
    Eigen::MRPd phi = Eigen::MRPd::Identity(); //!< attitude constraint violation
    Eigen::Vector3d omega_B2B1_B2 = Eigen::Vector3d::Zero(); //!< [rad/s] angular velocity constraint violation in spacecraft 2 body frame

    // Force and torque quantities stored to be assigned on the alternating call of computeForceTorque
    Eigen::Vector3d Fc_N = Eigen::Vector3d::Zero(); //!< [N] force applied on each spacecraft COM in the inertial frame
    Eigen::Vector3d T_B2 = Eigen::Vector3d::Zero(); //!< [N-m] torque applied on spacecraft 2 in its body frame
    Eigen::Vector3d T_B1 = Eigen::Vector3d::Zero(); //!< [N-m] torque applied on spacecraft 1 in its body frame
};


#endif /* CONSTRAINT_DYNAMIC_EFFECTOR_H */
