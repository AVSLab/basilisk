/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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


#ifndef SPACECRAFT_PLUS_H
#define SPACECRAFT_PLUS_H
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "hubEffector.h"
#include "../_GeneralModuleFiles/gravityEffector.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include "_GeneralModuleFiles/sys_model.h"
#include <vector>
#include <stdint.h>
#include "spacecraftPlusMsg.h"
#include "hingedRigidBodyStateEffector.h"

/*! @brief Object that is to be used by an integrator. It's basically an interface with only one method: the F function describing a dynamic model X_dot = F(X,t)
 */
class SpacecraftPlus : public DynamicObject{
public:
    HubEffector hub;                     //!< [-] The spacecraft hub that effectors spoke off
    GravityEffector gravField;           //!< [-] Gravitational field experienced by spacecraft
    Eigen::Matrix3d matrixAContr;        //!< [-] The contribution of each stateEffetor to matrix A
    Eigen::Matrix3d matrixBContr;        //!< [-] The contribution of each stateEffetor to matrix B
    Eigen::Matrix3d matrixCContr;        //!< [-] The contribution of each stateEffetor to matrix C
    Eigen::Matrix3d matrixDContr;        //!< [-] The contribution of each stateEffetor to matrix D
    Eigen::Vector3d vecTransContr;       //!< [-] The contribution of each stateEffetor to vecTrans
    Eigen::Vector3d vecRotContr;         //!< [-] The contribution of each stateEffetor to vecRot
    Eigen::MatrixXd *m_SC;               //!< [kg] spacecrafts total mass
    Eigen::MatrixXd *ISCPntB_B;          //!< [kg m^2] Inertia of s/c about point B in B frame components
    Eigen::MatrixXd *c_B;                //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *cPrime_B;           //!< [m] Body time derivative of c_B
    Eigen::MatrixXd *ISCPntBPrime_B;     //!< [m] Body time derivative of ISCPntB_B
    Eigen::MatrixXd *sysTime;
    Eigen::MatrixXd *property_dcm_BS;    //!< [-] Dynamic property version of the structure to body
    Eigen::Vector3d totSCAngMomentum_N;  //!< [kg-m^2/s] Total angular momentum of the s/c in N frame compenents
    Eigen::MatrixXd dcm_BS;              //!< [-] Transformation from structure to body frame
    double totSCEnergy;                  //!< [J]    Total energy of the spacecraft
    double totSCAngMomentum;             //!< [kg-m^2/s] Magnitude of total angular momentum of the s/c
	double currTimeStep;
	double timePrevious;
    uint64_t simTimePrevious;            //!< [-] Previous simulation time
	uint64_t numOutMsgBuffers;           //!< [-] Number of output message buffers for I/O
    
    std::string sysTimePropertyName;     //!< [-] Name of the system time property
	std::string scStateOutMsgName;       //!< [-] Name of the state output message
    std::string struct2BdyPropertyName;  //!< [-] Name of the structure to body dynamics property
    
public:
    SpacecraftPlus();
    ~SpacecraftPlus();
    void SelfInit();                           //!< [-] Lets spacecraft plus create its own msgs
    void CrossInit();                          //!< [-] Hook to tie s/c plus back into provided msgs
    void UpdateState(uint64_t CurrentSimNanos);//!< [-] Runtime hook back into Basilisk arch
    void equationsOfMotion(double t);          //!< [-] Everyone will need to provide this EOM
    void integrateState(double t);             //!< [-] Everyone will need to integrate the state
    void computeEnergyMomentum();              //!< [-] User can implement NRG/moment check
    void initializeDynamics();                 //!< [-] Method to link all spacecraft states
    void linkInStates(DynParamManager& statesIn);
	void writeOutputMessages(uint64_t clockTime); //! [-] Method to write all of the class output messages

private:
	StateData *hubR_N;                         //!< [-] Inertial position for the hub
	StateData *hubV_N;                         //!< [-] Inertial velocity for the hub
	StateData *hubOmega_BN_B;                  //!< [-] Attitude rate of the hub
	StateData *hubSigma;                       //!< [-] sigmaBN for the hub

	int64_t scStateOutMsgID;                  //!< [-] Message ID for the outgoing spacecraft state
};

#endif /* SPACECRAFT_PLUS_H */
