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

#ifndef GENERAL_SINGLE_BODY_STATE_EFFECTOR_H
#define GENERAL_SINGLE_BODY_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"

struct DOF {
    enum class Type { ROTATION, TRANSLATION };

    Type type;
    int index;

    Eigen::Vector3d axis_G;
    Eigen::Matrix3d dcm_G0P;
    double betaInit{};
    double betaDotInit{};
    double screwConstant{1.0};


    double beta;
    double betaDot;
    Eigen::Matrix3d dcm_GB;
};


/*! @brief General rigid body state effector class */
class GeneralSingleBodyStateEffector: public StateEffector, public SysModel {
public:
    GeneralSingleBodyStateEffector();
    ~GeneralSingleBodyStateEffector();

    void setMass(const double mass);  //!< Setter method for the effector mass
    void setIPntGc_G(const Eigen::Matrix3d IPntGc_G);  //!< Setter method for IPntGc_G
    void setR_GcG_G(const Eigen::Vector3d r_GcG_G);  //!< Setter method for r_GcG_G
    void setBetaInit(const Eigen::VectorXd betaInit);
    void setBetaDotInit(const Eigen::VectorXd betaDotInit);
    double getMass() const;  //!< Getter method for the effector mass
    const Eigen::Matrix3d getIPntGc_G() const;  //!< Getter method for IPntGc_G
    const Eigen::Vector3d getR_GcG_G() const;  //!< Getter method for r_GcG_G
    Eigen::VectorXd getBetaInit();
    Eigen::VectorXd getBetaDotInit();
    void addRotationalDOF(Eigen::Vector3d rotHat_G,
                          Eigen::Matrix3d dcm_G0P,
                          double thetaInit,
                          double thetaDotInit);
    void addTranslationalDOF(Eigen::Vector3d transHat_G,
                             Eigen::Matrix3d dcm_G0P,
                             double rhoInit,
                             double rhoDotInit);
    void addRotScrewDOF(Eigen::Vector3d rotHat_G,
                        Eigen::Matrix3d dcm_G0P,
                        double thetaInit,
                        double thetaDotInit,
                        double screwConstant);
    void addTransScrewDOF(Eigen::Vector3d transHat_G,
                          Eigen::Matrix3d dcm_G0P,
                          double rhoInit,
                          double rhoDotInit,
                          double screwConstant);
    void Reset(uint64_t currentClock) override;                      //!< Method for reset
    void writeOutputStateMessages(uint64_t currentClock) override;   //!< Method for writing the output messages
	void UpdateState(uint64_t currentSimNanos) override;             //!< Method for updating the effector states
    void registerStates(DynParamManager& statesIn) override;         //!< Method for registering the effector's states
    void linkInStates(DynParamManager& states) override;             //!< Method for giving the effector access to hub states
    void registerProperties(DynParamManager& states) override;       //!< Method for registering the prescribed motion properties
    void updateContributions(double integTime,
                             BackSubMatrices & backSubContr,
                             Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override;          //!< Method for computing the effector's back-substitution contributions
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::Vector3d sigma_BN) override;      //!< Method for effector to compute its state derivatives
    void updateEffectorMassProps(double integTime) override;         //!< Method for calculating the effector mass props and prop rates
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d & rotAngMomPntCContr_B,
                                      double & rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override;    //!< Method for computing the energy and momentum of the effector
    void computeGeneralBodyInertialStates();       //!< Method for computing the effector's states relative to the inertial frame

    Message<SCStatesMsgPayload> generalSingleBodyConfigLogOutMsg;                  //!< Output config log message for the effector's states

private:
    double mass;
    Eigen::Matrix3d IPntGc_G;
    Eigen::Vector3d r_GcG_G;
    std::vector<DOF> jointDOFList;
    int numDOF = 0;

    std::vector<double> betaInitList;
    std::vector<double> betaDotInitList;

    Eigen::MatrixXd TMat;

    static uint64_t effectorID;                                         //!< ID number of this panel

    std::string nameOfBetaState;
    std::string nameOfBetaDotState;
    std::string nameOfInertialPositionProperty;                      //!< -- identifier for the inertial position property
    std::string nameOfInertialVelocityProperty;                      //!< -- identifier for the inertial velocity property
    std::string nameOfInertialAttitudeProperty;                      //!< -- identifier for the inertial attitude property
    std::string nameOfInertialAngVelocityProperty;                   //!< -- identifier for the inertial angular velocity property

    template <typename Type>
    /** Assign the state engine parameter names */
    void assignStateParamNames(Type effector) {
        effector->setPropName_inertialPosition(this->nameOfInertialPositionProperty);
        effector->setPropName_inertialVelocity(this->nameOfInertialVelocityProperty);
        effector->setPropName_inertialAttitude(this->nameOfInertialAttitudeProperty);
        effector->setPropName_inertialAngVelocity(this->nameOfInertialAngVelocityProperty);
    };

    // States
    Eigen::VectorXd beta;
    Eigen::VectorXd betaDot;
    StateData* betaState = nullptr;
    StateData* betaDotState = nullptr;
    Eigen::MatrixXd* inertialPositionProperty = nullptr;  // Hub
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;  // Hub

    Eigen::Vector3d r_GcN_N{0.0, 0.0, 0.0};
    Eigen::Vector3d v_GcN_N{0.0, 0.0, 0.0};
    Eigen::MatrixXd* r_GN_N;
    Eigen::MatrixXd* v_GN_N;
    Eigen::MatrixXd* sigma_GN;
    Eigen::MatrixXd* omega_GN_G;

};

#endif /* GENERAL_SINGLE_BODY_STATE_EFFECTOR_H */
