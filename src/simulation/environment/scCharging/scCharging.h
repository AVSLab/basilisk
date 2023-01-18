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


#ifndef SCCHARGINIG_H
#define SCCHARGINIG_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/VoltMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <vector>
#include <Eigen/Dense>

/*! @brief INSERT MODULE DESCRIPTION
 */
class ScCharging: public SysModel {
public:
    // Constructor And Destructor
    ScCharging();
    ~ScCharging();
    
    // Methods
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

private:
    void readMessages();
    
public:
    std::vector<ReadFunctor<SCStatesMsgPayload>> scStateInMsgs; //!< vector of spacecraft state input messages
    std::vector<ReadFunctor<VoltMsgPayload>> voltInMsgs;     //!< vector of voltage input messages

    BSKLogger bskLogger;                                        //!< -- BSK Logging

private:
    std::vector<double> volt;                                   //!< [V] input voltage for each spacecrat object
    std::vector<Eigen::Vector3d> r_BN_NList;                    //!< [m] list of inertial satellite position vectors
    std::vector<Eigen::MRPd> sigma_BNList;                      //!< [m] list of satellite MRP orientations
};


#endif
