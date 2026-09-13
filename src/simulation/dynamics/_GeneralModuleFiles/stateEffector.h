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

#ifndef STATE_EFFECTOR_H
#define STATE_EFFECTOR_H

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <set>
#include <vector>
#include <string>
#include "architecture/utilities/avsEigenMRP.h"
#include "dynParamManager.h"
#ifndef SWIG
#include "effectorName.h"
#endif
#include "architecture/utilities/bskLogging.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"


/*! Backsubstitution matrix structure*/
struct BackSubMatrices {
    Eigen::Matrix3d matrixA;             //!< Backsubstitution matrix A
    Eigen::Matrix3d matrixB;             //!< Backsubstitution matrix B
    Eigen::Matrix3d matrixC;             //!< Backsubstitution matrix C
    Eigen::Matrix3d matrixD;             //!< Backsubstitution matrix D
    Eigen::Vector3d vecTrans;            //!< Backsubstitution translation vector
    Eigen::Vector3d vecRot;              //!< Backsubstitution rotation vector
};

/*! @brief Abstract class that is used to implement an effector attached to the dynamicObject that has a state that
 needs to be integrated. For example: reaction wheels, flexing solar panels, fuel slosh etc */
typedef struct {
    double mEff;                           //!< [kg] Mass of the effector
    double mEffDot;                        //!< [kg/s] Time derivative of retained effector mass
    Eigen::Matrix3d IEffPntB_B;            //!< [kg m^2] Inertia of effector relative to point B in B frame components
    Eigen::Vector3d rEff_CB_B;             //!< [m] Center of mass of effector with respect to point B in B frame comp
    Eigen::Vector3d rEffPrime_CB_B;        //!< [m/s] Body-frame derivative of the retained effector CoM
    Eigen::Matrix3d IEffPrimePntB_B;       //!< [kg m^2/s] Body-frame derivative of retained effector inertia
    bool hasMassPropertyRateDynamics;      //!< True when the equations of motion use explicit rate overrides
    double mEffDotDynamics;                //!< [kg/s] Mass rate substituted into generic rate-dependent terms
    Eigen::Vector3d rEffPrime_CB_BDynamics; //!< [m/s] CoM derivative used by generic rate terms
    Eigen::Matrix3d IEffPrimePntB_BDynamics; //!< [kg m^2/s] Inertia rate substituted into generic rate-dependent terms
}EffectorMassProps;

/*! @brief state effector class */
class StateEffector {
public:
    std::string nameOfSpacecraftAttachedTo="";//!< class variable
    std::string parentSpacecraftName="";   //!< name of the spacecraft the state effector is attached to
    EffectorMassProps effProps;            //!< stateEffectors instantiation of effector mass props
    Eigen::VectorXd stateDerivContribution; //!< stateEffector contribution to another stateEffector to prevent double-counting
    Eigen::Vector3d forceOnBody_B;         //!< [N] Force that the state effector applies to the s/c
    Eigen::Vector3d torqueOnBodyPntB_B;    //!< [N] Torque that the state effector applies to the body about point B
    Eigen::Vector3d torqueOnBodyPntC_B;    //!< [N] Torque that the state effector applies to the body about point B
    Eigen::Vector3d r_BP_P;                //!< position vector of the spacecraft mody frame origin B relative to the primary spacecraft body frame P.  This is used in the SpacecraftSystem module where multiple spacecraft hubs can be a single spacecraft
    Eigen::Matrix3d dcm_BP;                //!< DCM of the spacecraft body frame B relative to primary spacecraft body frame P

    /** setter for `stateNameOfPosition` property */
    void setStateNameOfPosition(std::string value);
    /** getter for `stateNameOfPosition` property */
    const std::string getStateNameOfPosition() const {return this->stateNameOfPosition; }
    /** setter for `stateNameOfVelocity` property */
    void setStateNameOfVelocity(std::string value);
    /** getter for `stateNameOfVelocity` property */
    const std::string getStateNameOfVelocity() const { return this->stateNameOfVelocity; }
    /** setter for `stateNameOfSigma` property */
    void setStateNameOfSigma(std::string value);
    /** getter for `stateNameOfSigma` property */
    const std::string getStateNameOfSigma() const { return this->stateNameOfSigma; }
    /** setter for `stateNameOfOmega` property */
    void setStateNameOfOmega(std::string value);
    /** getter for `stateNameOfOmega` property */
    const std::string getStateNameOfOmega() const { return this->stateNameOfOmega; }
    /** setter for `propName_m_SC` property */
    void setPropName_m_SC(std::string value);
    /** getter for `propName_m_SC` property */
    const std::string getPropName_m_SC() const { return this->propName_m_SC; }
    /** setter for `propName_mDot_SC` property */
    void setPropName_mDot_SC(std::string value);
    /** getter for `propName_mDot_SC` property */
    const std::string getPropName_mDot_SC() const { return this->propName_mDot_SC; }
    /** setter for `propName_centerOfMassSC` property */
    void setPropName_centerOfMassSC(std::string value);
    /** getter for `propName_centerOfMassSC` property */
    const std::string getPropName_centerOfMassSC() const { return this->propName_centerOfMassSC; }
    /** setter for `propName_inertiaSC` property */
    void setPropName_inertiaSC(std::string value);
    /** getter for `propName_inertiaSC` property */
    const std::string getPropName_inertiaSC() const { return this->propName_inertiaSC; }
    /** setter for `propName_inertiaPrimeSC` property */
    void setPropName_inertiaPrimeSC(std::string value);
    /** getter for `propName_inertiaPrimeSC` property */
    const std::string getPropName_inertiaPrimeSC() const { return this->propName_inertiaPrimeSC; }
    /** setter for `propName_centerOfMassPrimeSC` property */
    void setPropName_centerOfMassPrimeSC(std::string value);
    /** getter for `propName_centerOfMassPrimeSC` property */
    const std::string getPropName_centerOfMassPrimeSC() const { return this->propName_centerOfMassPrimeSC; }
    /** setter for `propName_centerOfMassDotSC` property */
    void setPropName_centerOfMassDotSC(std::string value);
    /** getter for `propName_centerOfMassDotSC` property */
    const std::string getPropName_centerOfMassDotSC() const { return this->propName_centerOfMassDotSC; }
    /** setter for `propName_inertialPosition` property */
    void setPropName_inertialPosition(std::string value);
    /** getter for `propName_inertialPosition` property */
    const std::string getPropName_inertialPosition() const { return this->propName_inertialPosition; }
    /** setter for `propName_inertialVelocity` property */
    void setPropName_inertialVelocity(std::string value);
    /** getter for `propName_inertialVelocity` property */
    const std::string getPropName_inertialVelocity() const { return this->propName_inertialVelocity; }
    /** setter for `propName_vehicleGravity` property */
    void setPropName_vehicleGravity(std::string value);
    /** getter for `propName_vehicleGravity` property */
    const std::string getPropName_vehicleGravity() const { return this->propName_vehicleGravity; }
    /** setter for `propName_prescribedPosition` property */
    void setPropName_prescribedPosition(std::string value);
    /** getter for `propName_prescribedPosition` property */
    const std::string getPropName_prescribedPosition() const { return this->propName_prescribedPosition; }
    /** setter for `propName_prescribedVelocity` property */
    void setPropName_prescribedVelocity(std::string value);
    /** getter for `propName_prescribedVelocity` property */
    const std::string getPropName_prescribedVelocity() const { return this->propName_prescribedVelocity; }
    /** setter for `propName_prescribedAcceleration` property */
    void setPropName_prescribedAcceleration(std::string value);
    /** getter for `propName_prescribedAcceleration` property */
    const std::string getPropName_prescribedAcceleration() const { return this->propName_prescribedAcceleration; }
    /** setter for `propName_prescribedAttitude` property */
    void setPropName_prescribedAttitude(std::string value);
    /** getter for `propName_prescribedAttitude` property */
    const std::string getPropName_prescribedAttitude() const { return this->propName_prescribedAttitude; }
    /** setter for `propName_prescribedAngVelocity` property */
    void setPropName_prescribedAngVelocity(std::string value);
    /** getter for `propName_prescribedAngVelocity` property */
    const std::string getPropName_prescribedAngVelocity() const { return this->propName_prescribedAngVelocity; }
    /** setter for `propName_prescribedAngAcceleration` property */
    void setPropName_prescribedAngAcceleration(std::string value);
    /** getter for `propName_prescribedAngAcceleration` property */
    const std::string getPropName_prescribedAngAcceleration() const { return this->propName_prescribedAngAcceleration; }

    BSKLogger bskLogger;                   //!< BSK Logging

public:
    StateEffector();                       //!< Constructor
#ifndef SWIG
    /** @brief Copy effector data while assigning a fresh naming identity.
     * @param other Effector to copy.
     */
    StateEffector(const StateEffector& other) = default;
    /** @brief Copy effector data and renew the destination's naming identity.
     * @param other Effector to copy.
     * @return This effector.
     */
    StateEffector& operator=(const StateEffector& other) = default;
    /** @brief Transfer effector data and its naming ownership.
     * @param other Effector to move from.
     * @note Derived classes that support moving must also move this base.
     */
    StateEffector(StateEffector&& other) = default;
    /** @brief Transfer effector data and its naming ownership during assignment.
     * @param other Effector to move from.
     * @return This effector.
     */
    StateEffector& operator=(StateEffector&& other) = default;
#endif
    virtual ~StateEffector();              //!< Destructor
    virtual void updateEffectorMassProps(double integTime);  //!< Method for stateEffector to give mass contributions
    virtual void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::MRPd sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< Back-sub contributions
    virtual void addPrescribedMotionCouplingContributions(BackSubMatrices& backSubContr);  //!< Method for adding coupling contributions for state effector branching on prescribed motion
    virtual void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B);  //!< Energy and momentum calculations
    virtual void modifyStates(double integTime); //!< Modify state values after integration
    virtual void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B);  //!< Force and torque on s/c due to stateEffector
    virtual void writeOutputStateMessages(uint64_t integTimeNanos); //!< Write State Messages after integration
    virtual void registerStates(DynParamManager& states) = 0;  //!< Method for stateEffectors to register states
    virtual void registerProperties(DynParamManager& states);  //!< Method for stateEffectors to register properties
    virtual void addDynamicEffector(DynamicEffector *newDynamicEffector, int segment);  //!< Method to attach a dynamic effector
    virtual void linkInStates(DynParamManager& states) = 0;  //!< Method for stateEffectors to get other states
    virtual void linkInPrescribedMotionProperties(DynParamManager& properties);  //!< Method for stateEffectors to access prescribed motion properties
    virtual void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::MRPd sigma_BN)=0;  //!< Method for each stateEffector to calculate derivatives
    virtual void prependSpacecraftNameToStates();
    virtual void receiveMotherSpacecraftData(Eigen::Vector3d rSC_BP_P, Eigen::Matrix3d dcmSC_BP); //!< class method

#ifndef SWIG
    /** @brief Bind attached dynamic effectors after all declared states and properties exist.
     * @param manager Dynamics manager containing the resolved and registered names.
     * @note Used by the manager-local preparation path. The default has no attachments.
     */
    virtual void bindAttachedDynamicEffectors(DynParamManager& manager);

    /** @brief Collect this effector's name group for a later manager-wide resolution pass.
     * @param manager Dynamics manager that will own the states and properties.
     * @note Legacy is a no-op. ManagerLocal requires an override of
     * describeEffectorNames(). Nested effectors are visited in attachment order.
     * Repeating this call on the same manager with unchanged requests preserves identity.
     * Copying both the manager and its effectors preserves their declarations;
     * copying an effector into the same manager requires a new declaration.
     * If the previous manager was destroyed, collection can bind to a replacement.
     */
    void collectEffectorNames(DynParamManager& manager);

    /** @brief Collect all attachment trees while rejecting shared children and cycles.
     * @param manager Dynamics manager that will own the states and properties.
     * @param roots Top-level effectors in registration order.
     * @note Legacy is a no-op. Each effector may occur only once in the combined trees.
     */
    static void collectEffectorNames(DynParamManager& manager, const std::vector<StateEffector*>& roots);

    /** @brief Cancel the pending requests owned by this effector and its collected children.
     * @param manager Dynamics manager used to collect this effector's names.
     * @note Cancel before moving a pending effector away from a live manager. Resolved
     * requests cannot be cancelled. All requests are validated before any are cancelled.
     * Calling without an owned request is a no-op, including on an uncollected copy.
     * Repeated or cyclic attachments left by failed collection are visited only once.
     */
    void cancelEffectorNames(DynParamManager& manager);
#endif

protected:
#ifndef SWIG
    /** @brief Describe names using explicit override metadata from the derived effector.
     * @return Complete name group; the default empty family denotes an unsupported effector.
     * @note External effectors opt in by overriding this method, collecting before
     * resolution, and using the resolved names before registering or binding data.
     * Existing public name attributes and setters remain the derived class's responsibility.
     */
    virtual EffectorNameGroup describeEffectorNames() const { return {}; }

    /** @brief Enumerate nested state effectors for name collection.
     * @return Children in registration order; the default has no children.
     */
    virtual std::vector<StateEffector*> getNestedStateEffectors() const { return {}; }

    /** @brief Read a prepared name for assignment to a derived effector's public name field.
     * @param manager Dynamics manager used for collection and resolution.
     * @param key Local key supplied by describeEffectorNames().
     * @return Final manager-local name.
     */
    const std::string& getResolvedEffectorName(const DynParamManager& manager, const std::string& key) const;

    /** @brief Retrieve this effector's request for ownership-aware registration.
     * @return Current handle to pass to registerEffectorState() or createEffectorProperty().
     * @note Calling before collection, on an uncollected copy, or after the manager
     * lifetime ends raises BasiliskError. Recollect before registering data in a copied manager.
     */
    const EffectorNameRequest& getEffectorNameRequest() const;
#endif

    std::string stateNameOfPosition = "";                           //!< state engine name of the parent rigid body inertial position vector
    std::string stateNameOfVelocity = "";                           //!< state engine name of the parent rigid body inertial velocity vector
    std::string stateNameOfSigma = "";                              //!< state engine name of the parent rigid body inertial attitude
    std::string stateNameOfOmega = "";                              //!< state engine name of the parent rigid body inertial angular velocity vector

    std::string propName_m_SC = "";                                 //!< property name of m_SC
    std::string propName_mDot_SC = "";                              //!< property name of mDot_SC
    std::string propName_centerOfMassSC = "";                       //!< property name of centerOfMassSC
    std::string propName_inertiaSC = "";                            //!< property name of inertiaSC
    std::string propName_inertiaPrimeSC = "";                       //!< property name of inertiaPrimeSC
    std::string propName_centerOfMassPrimeSC = "";                  //!< property name of centerOfMassPrimeSC
    std::string propName_centerOfMassDotSC = "";                    //!< property name of centerOfMassDotSC
    std::string propName_inertialPosition = "";                     //!< property name of inertialPosition
    std::string propName_inertialVelocity = "";                     //!< property name of inertialVelocity
    std::string propName_vehicleGravity = "";                       //!< property name of vehicleGravity

    std::string propName_prescribedPosition = "";                   //!< property name of prescribedPosition
    std::string propName_prescribedVelocity = "";                   //!< property name of prescribedVelocity
    std::string propName_prescribedAcceleration = "";               //!< property name of prescribedAcceleration
    std::string propName_prescribedAttitude = "";                   //!< property name of prescribedAttitude
    std::string propName_prescribedAngVelocity = "";                //!< property name of prescribedAngVelocity
    std::string propName_prescribedAngAcceleration = "";            //!< property name of prescribedAngAcceleration

    Eigen::MatrixXd* prescribedPositionProperty = nullptr;         //!< [m] r_PB_B prescribed position relative to hub
    Eigen::MatrixXd* prescribedVelocityProperty = nullptr;         //!< [m/s] rPrime_PB_B prescribed velocity relative to hub
    Eigen::MatrixXd* prescribedAccelerationProperty = nullptr;     //!< [m/s^2] rPrimePrime_PB_B prescribed acceleration relative to hub
    Eigen::MatrixXd* prescribedAttitudeProperty = nullptr;         //!< sigma_PB prescribed MRP attitude relative to hub
    Eigen::MatrixXd* prescribedAngVelocityProperty = nullptr;      //!< [rad/s] omega_PB_P prescribed angular velocity relative to hub
    Eigen::MatrixXd* prescribedAngAccelerationProperty = nullptr;  //!< [rad/s^2] omegaPrime_PB_P prescribed angular acceleration relative to hub

private:
#ifndef SWIG
    EffectorNameRequest effectorNameRequest; //!< Immutable request retained across repeated preparation.
    /** @brief Collect a tree while rejecting cycles and repeated children.
     * @param manager Manager that will own all names in the tree.
     * @param visited Effectors already encountered in this traversal.
     */
    void collectEffectorNames(DynParamManager& manager, std::set<const StateEffector*>& visited);
    EffectorNameIdentity effectorNameIdentity; //!< Distinguishes copies without relying on memory addresses.
    std::weak_ptr<const EffectorNameIdentity::Token> effectorNameRequestOwner; //!< Identity used at collection.
    std::weak_ptr<const EffectorNameIdentity::Token> effectorNameManager; //!< Manager lifetime used at collection.
#endif
};


#endif /* STATE_EFFECTOR_H */
