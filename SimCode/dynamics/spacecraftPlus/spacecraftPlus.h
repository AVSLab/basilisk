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
#include "../_GeneralModuleFiles/dynObject2.h"
#include "_GeneralModuleFiles/sys_model.h"
#include <vector>
#include <stdint.h>

/*! @brief Object that is to be used by an integrator. It's basically an interface with only one method: the F function describing a dynamic model X_dot = F(X,t)
 */
class SpacecraftPlus : public DynObject2{
public:
    HubEffector hub;                          //! [-] The spacecraft hub that effectors spoke off
    GravityEffector gravField;                //! [-] Gravitational field experienced by spacecraft
    Eigen::Matrix3d matrixAContrSCP;           //! [-] Spacecraft plus holds the value for all matrices
    Eigen::Matrix3d matrixBContrSCP;
    Eigen::Matrix3d matrixCContrSCP;
    Eigen::Matrix3d matrixDContrSCP;
    Eigen::Vector3d vecTransContrSCP;
    Eigen::Vector3d vecRotContrSCP;
    Eigen::MatrixXd *m_SC;
    Eigen::MatrixXd *ISCPntB_B;
    Eigen::MatrixXd *cPrime_B;
    Eigen::MatrixXd *ISCPntBPrime_B;
    Eigen::MatrixXd *c_B;
    Eigen::MatrixXd *sysTime;
	double currTimeStep;
	double timePrevious;
    uint64_t simTimePrevious;                 //! [-] Previous simulation time
    
    std::string sysTimePropertyName;          //! [-] Name of the system time property
    
public:
    SpacecraftPlus();
    ~SpacecraftPlus();
    void SelfInit();                           //! [-] Lets spacecraft plus create its own msgs
    void CrossInit();                          //! [-] Hook to tie s/c plus back into provided msgs
    void UpdateState(uint64_t CurrentSimNanos);//! [-] Runtime hook back into Basilisk arch
    void equationsOfMotion(double t);         //! [-] Everyone will need to provide this EOM
    void integrateState(double t);            //! [-] Everyone will need to integrate the state
    void computeEnergyMomentum();             //! [-] User can implement NRG/moment check
    void initializeDynamics();                //! [-] Method to link all spacecraft states
};

#endif /* SPACECRAFT_PLUS_H */
