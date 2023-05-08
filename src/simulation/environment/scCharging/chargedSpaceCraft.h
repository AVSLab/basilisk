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
    void setID(std::string IDtype, int inputID);              //! < setter function for ID
    int getID(std::string IDtype);                            //! < getter function for ID
// private functions
private:


// public variables
public:
    std::string name;                       //! < spacecraft name
    int priority;                           //! < value assigning order in which spacecraft equilibrium potential is calculated
    
    std::string electronGunScName;          //! name of the spacecraft that emits an electron beam
    bool emitsEB;                           //! boolean that tells whether or not the craft emits an electron beam
    struct {
        double alphaEB;
        double currentEB;
        double energyEB;
    } electronGun;
    
    double A;                               //! < surface area of spacecraft
    double A_sunlit;                        //! < surface area of spacecraft used in photoelectric current

    BSKLogger bskLogger;                    //!< -- BSK Logging

// private variables
private:
    int ID;                                 //! < integer corresponding to spacecraft
    int electronGunScID;                    //! ID of the spacecraft that emits an electron beam
};

#endif /* chargedSpaceCraft_h */
