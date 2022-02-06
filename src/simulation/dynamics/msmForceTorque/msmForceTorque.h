/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef MSMFORCETORQUE_H
#define MSMFORCETORQUE_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/VoltMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdForceInertialMsgPayload.h"
#include "architecture/msgPayloadDefCpp/ChargeMsmMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <vector>
#include <Eigen/Dense>

/*! @brief This module uses the Multi-Sphere-Method (MSM) to evaluate the mutual electrostatic force and torque interactions between a series of spacecraft object.  The charging is specified through a voltage where the object is assumed to have aconstant voltaged across the surface.  The MSM model for each space object is given through a list of body-fixed sphere locations and sphere radii.  See `Multi-Sphere Method for Modeling Electrostatic Forces and Torques <http://dx.doi.org/10.1016/j.asr.2012.08.014 >`__ for more information on the MSM method.
 */
class MsmForceTorque: public SysModel {
public:
    MsmForceTorque();
    ~MsmForceTorque();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg, std::vector<double> radii, std::vector<Eigen::Vector3d> r_SB_B);

private:
    void readMessages();
    
public:
    std::vector<ReadFunctor<SCStatesMsgPayload>> scStateInMsgs; //!< vector of spacecraft state input messages
    std::vector<ReadFunctor<VoltMsgPayload>> voltInMsgs;     //!< vector of voltage input messages

    std::vector<Message<CmdTorqueBodyMsgPayload>*> eTorqueOutMsgs;      //!< vector of E-torques in body frame components
    std::vector<Message<CmdForceInertialMsgPayload>*> eForceOutMsgs;    //!< vector of E-forces in inertial frame components
    std::vector<Message<ChargeMsmMsgPayload>*> chargeMsmOutMsgs;        //!< vector of spacecraft MSM charge values

    BSKLogger bskLogger;                                        //!< -- BSK Logging

private:
    std::vector<std::vector<double>> radiiList;                 //!< vector of MSM sphere radii
    std::vector<std::vector<Eigen::Vector3d>> r_SB_BList;       //!< vector of body-fixed MSM sphere locations
    unsigned int numSat;                                        //!< number of satellites
    unsigned int numSpheres;                                    //!< numer of spheres being modeled
    std::vector<double> volt;                                   //!< [V] input voltage for each spacecrat object
    std::vector<Eigen::Vector3d> r_BN_NList;                    //!< [m] list of inertial satellite position vectors
    std::vector<Eigen::MRPd> sigma_BNList;                      //!< [m] list of satellite MRP orientations
};


#endif
