//
//  chargedSpaceCraft.cpp
//  basilisk
//
//  Created by Zack Ellis on 4/25/23.
//

#include "simulation/environment/scCharging/chargedSpaceCraft.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <vector>
#include <string>

void chargedSpaceCraft::setID(std::string IDtype, int inputID)
{
    if (IDtype == "ID") {
        ID = inputID;
    } else if (IDtype == "electronGunID") {
        electronGunScID = inputID;
    } else {
//        bskLogger.bskLog(BSK_ERROR, "chargedSpaceCraft.getID: Invalid IDtype. Must specify 'ID' or 'electronGunID'");
    }
}

int chargedSpaceCraft::getID(std::string IDtype)
{
    if (IDtype == "ID") {
        return ID;
    } else if (IDtype == "electronGunID") {
        return electronGunScID;
    } else {
//        bskLogger.bskLog(BSK_ERROR, "chargedSpaceCraft.getID: Invalid IDtype. Must specify 'ID' or 'electronGunID'");
        return NAN;
    }
}
