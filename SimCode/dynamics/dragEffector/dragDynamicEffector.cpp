/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#include "dragDynamicEffector.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include "../../environment/ExponentialAtmosphere/exponentialAtmosphere.h"
#include <cstring>
#include <iostream>
#include <cmath>

DragDynamicEffector::DragDynamicEffector()
{
  this->coreParams.projectedArea = 0.0;
  this->coreParams.velocityMag = 0.0;
  this->coreParams.dragCoeff = 0.0;

  this->atmoDensInMsgName = "base_atmo_props";
  this->modelType = "cannonball";
  forceExternal_B.fill(0.0);
  torqueExternalPntB_B.fill(0.0);
  forceExternal_N.fill(0.0);
  this->dragHist.PreviousIterTime = 0;
  this->dragHist.PreviousDragForce = 0;
  this->locInertialVel.fill(0.0);
  this->dragDirection.fill(0.0);
  this->DensInMsgId = -1;
  return;
}

/*! The destructor.*/
DragDynamicEffector::~DragDynamicEffector()
{
    return;
}

/*! This method is used to clear out the current drag states and make sure
 that the overall model is ready for firing
 @return void
 */
void DragDynamicEffector::SelfInit()
{
  return;
}

/*! This method is used to connect the input command message to the drags.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void DragDynamicEffector::CrossInit()
{

    //! Begin method steps
    //! - Find the message ID associated with the InputCmds string.
    //! - Warn the user if the message is not successfully linked.
    DensInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(atmoDensInMsgName,
                                                                     sizeof(AtmoOutputData), moduleID);
}


/*! This method is here to write the output message structure into the specified
 message.  It is currently blank but we will certainly have an output message
 soon.  If it is already here, bludgeon whoever added it and didn't fix the
 comment.sizeof(dragOutputData)
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void DragDynamicEffector::WriteOutputMessages(uint64_t CurrentClock)
{
  return;
}


/*! This method is used to read the incoming command message and set the
 associated command structure for operating the drags.
 @return void
 */
bool DragDynamicEffector::ReadInputs()
{
    bool dataGood;
    //! - Zero the command buffer and read the incoming command array
    SingleMessageHeader LocalHeader;
    memset(&densityBuffer, 0x0, sizeof(AtmoOutputData));
    memset(&LocalHeader, 0x0, sizeof(LocalHeader));
    dataGood = SystemMessaging::GetInstance()->ReadMessage(DensInMsgId, &LocalHeader,
                                                          sizeof(AtmoOutputData),
                                                           reinterpret_cast<uint8_t*> (&densityBuffer), moduleID);
    return(true);

}

/*! This method is used to read the new commands vector and set the drag
 firings appropriately.  It assumes that the ReadInputs method has already been
 run successfully.  It honors all previous drag firings if they are still
 active.  Note that for unit testing purposes you can insert firings directly
 into NewThrustCmds.
 @return void
 @param currentTime The current simulation time converted to a double
 */

void DragDynamicEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject("hubSigma");
    this->hubVelocity = states.getStateObject("hubVelocity");
}

void DragDynamicEffector::ComputeDragDir(){
  this->locInertialVel = this->hubVelocity->getState();
  this->dragDirection = -(this->locInertialVel / this->locInertialVel.norm());
  this->coreParams.velocityMag = this->locInertialVel.norm();
  return;
}

void DragDynamicEffector::CannonballDrag(){
  Eigen::Vector3d SingleDragForce;
  Eigen::Vector3d SingleDragTorque;
  //! Begin method steps
  //! - Zero out the structure force/torque for the drag set
  // MassProps are missing, so setting CoM to zero momentarily

  this->forceExternal_B.setZero();
  this->forceExternal_N.setZero();
  this->torqueExternalPntB_B.setZero();

  SingleDragForce = 0.5 * this->coreParams.dragCoeff * pow(this->coreParams.velocityMag, 2.0) * this->coreParams.projectedArea * this->atmoInData.neutralDensity * this->dragDirection;
  this->forceExternal_B = SingleDragForce;
  //! - Compute the center-of-mass relative torque and aggregate into the composite body torque
  SingleDragTorque = this->coreParams.comOffset.cross(SingleDragForce);
  this->torqueExternalPntB_B = SingleDragTorque;
  return;
}

void DragDynamicEffector::PlateDrag(){
  CannonballDrag();
  return;
}



void DragDynamicEffector::computeBodyForceTorque(double integTime){
  ComputeDragDir();

  if(this->modelType == "cannonball"){
      CannonballDrag();
  }
  else if(this->modelType == "plate"){
      PlateDrag();
  }

  return;
}

/*! This method is the main cyclical call for the scheduled part of the drag
 dynamics model.  It reads the current commands array and sets the drag
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void DragDynamicEffector::UpdateState(uint64_t CurrentSimNanos)
{
    ReadInputs();
    return;

}
