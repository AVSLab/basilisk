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


#include "simulation/dynamics/msmForceTorque/msmForceTorque.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <iostream>
#include <cstring>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
MsmForceTorque::MsmForceTorque()
{
}

/*! Module Destructor */
MsmForceTorque::~MsmForceTorque()
{
}

/*! This method is used to reset the module and checks that required input messages are connect.
    @return void
*/
void MsmForceTorque::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    for (long unsigned int c=0; c < this->scStateInMsgs.size(); c++ ){
        if (!this->scStateInMsgs.at(c).isLinked()) {
            bskLogger.bskLog(BSK_ERROR, "MsmForceTorque.scStateInMsgs[%d] was not linked.", c);
        }
    }
    
    for (long unsigned int c=0; c < this->voltInMsgs.size(); c++) {
        if (!this->voltInMsgs.at(c).isLinked()) {
            bskLogger.bskLog(BSK_ERROR, "MsmForceTorque.voltInMsgs[%d] was not linked.", c);
        }
    }
    
    this->numSat = this->scStateInMsgs.size();
    if (this->numSat < 2) {
        bskLogger.bskLog(BSK_ERROR, "MsmForceTorque must have 2 or more spacecraft components added. You added %lu.", this->numSat);
    }
}

/*!   Subscribe to the spacecraft state message and store the corresponding MSM radii and sphere positions
 */
void MsmForceTorque::addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg
                                          , std::vector<double> radii
                                          , std::vector<Eigen::Vector3d> r_SB_B)
{
    /* add the message reader to the vector of input spacecraft state messages */
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());
    
    /* increase the vector of voltage input message readers */
    ReadFunctor<VoltageMsgPayload> inVoltMsg;
    this->voltInMsgs.push_back(inVoltMsg);
    
    /* store MSM sphere radii and location information */
    if (radii.size() != r_SB_B.size()) {
        bskLogger.bskLog(BSK_ERROR, "MsmForceTorque:addSpacecraftToModel() The vector of MSM radii and positions must have the same size, they have sizes %lu and %lu.", radii.size(), r_SB_B.size());
    }
    this->radiiList.push_back(radii);
    this->r_SB_BList.push_back(r_SB_B);
    this->volt.push_back(0);
    Eigen::Vector3d zero;
    zero << 0.0, 0.0, 0.0;
    this->r_BN_N.push_back(zero);
        
    /* create output message objects */
    Message<CmdTorqueBodyMsgPayload> *msgTorque;
    msgTorque = new Message<CmdTorqueBodyMsgPayload>;
    this->eTorqueOutMsgs.push_back(msgTorque);
    
    Message<CmdForceBodyMsgPayload> *msgForce;
    msgForce = new Message<CmdForceBodyMsgPayload>;
    this->eForceOutMsgs.push_back(msgForce);
}

/*!  Read in the input messages
 */
void MsmForceTorque::readMessages()
{
    VoltageMsgPayload voltInMsgBuffer;          //!< local copy of voltage input message buffer
    SCStatesMsgPayload scStateInMsgsBuffer;     //!< local copy of spacecraft state input message buffer
    long unsigned int c;                        //!< spacecraft loop counter
    
    for (c = 0; c < this->numSat; c++) {
        voltInMsgBuffer = this->voltInMsgs.at(c)();
        this->volt.at(c) = voltInMsgBuffer.voltage;
        
        scStateInMsgsBuffer = this->scStateInMsgs.at(c)();
        this->r_BN_N.at(c) = cArray2EigenVector3d(scStateInMsgsBuffer.r_BN_N);
    }
}

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
    @return void
*/
void MsmForceTorque::UpdateState(uint64_t CurrentSimNanos)
{
//    CmdTorqueBodyMsgPayload eTorqueOutMsgsBuffer;  //!< local copy of e-torque output message buffer
//    CmdForceBodyMsgPayload eForceOutMsgsBuffer;  //!< local copy of e-force output message buffer

    printf("HPS: num of sc: %lu\n", this->numSat);
    for (long unsigned int c = 0; c<numSat; c++) {
        for (long unsigned int i = 0; i<this->radiiList.at(c).size(); i++) {
            printf("HPS: radius[%lu, %lu] = %f\n", c, i, this->radiiList.at(c).at(i));
            std::cout << this->r_SB_BList.at(c).at(i) << std::endl;
        }
    }
    // always zero the output message buffers before assigning values
//    eTorqueOutMsgsBuffer = this->eTorqueOutMsgs.zeroMsgPayload;
//    eForceOutMsgsBuffer = this->eForceOutMsgs.zeroMsgPayload;
//
    // read in the input messages
    this->readMessages();

//    // do some math and stuff to populate the output messages
//
//    // write to the output messages
//    this->eTorqueOutMsgs.write(&eTorqueOutMsgsBuffer, this->moduleID, CurrentSimNanos);
//    this->eForceOutMsgs.write(&eForceOutMsgsBuffer, this->moduleID, CurrentSimNanos);
}

