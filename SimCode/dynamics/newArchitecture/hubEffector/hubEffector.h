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


#ifndef HUB_EFFECTOR_H
#define HUB_EFFECTOR_H

#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/stateData.h"

/*! @brief Abstract class that is used to implement an effector impacting a HUB body
           that does not itself maintain a state or represent a changing component of
           the body (for example: gravity, thrusters, solar radiation pressure, etc.)
 */
class HubEffector : public StateEffector {
public:
    HubEffector();
    ~HubEffector();
    void linkInStates(DynParamManager& statesIn);
    void registerStates(DynParamManager& states);
    
public:
	std::string vehicleMassStateName;              //! [-] Name of the vehicle mass state
	std::string vehiclePositionStateName;          //! [-] Name of the vehicle position state
private:
	StateData *velocityState;                          //! [-] State of the mass of the vehicle
	StateData *posState;                           //! [-] Position state of the vehicle
    StateData *sigmaState;                           //! [-] Position
    StateData *omegaState;
    
};

#endif /* GRAVITY_EFFECTOR_H */
