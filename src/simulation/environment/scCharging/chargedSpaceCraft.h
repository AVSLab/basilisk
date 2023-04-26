//
//  chargedSpaceCraft.h
//  basilisk
//
//  Created by Zack Ellis on 4/25/23.
//

#ifndef chargedSpaceCraft_h
#define chargedSpaceCraft_h

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/PlasmaFluxMsgPayload.h"
#include "architecture/msgPayloadDefC/VoltMsgPayload.h"
#include <vector>
#include <Eigen/Dense>

/*! @brief This class holds values for a charged spacecraft
 */
class chargedSpaceCraft: public SysModel {
// public functions
public:

// private functions
private:


// public variables
public:
    std::string name;                       //! < spacecraft name
    int ID;                                 //! < integer corresponding to spacecraft
    int priority;                           //! < value assigning order in which spacecraft equilibrium potential is calculated
    
    double A;                               //! < surface area of spacecraft
    double A_sunlit;                        //! < surface area of spacecraft used in photoelectric current
// private variables
private:
    
};

#endif /* chargedSpaceCraft_h */
