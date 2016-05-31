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

#ifndef SOLAR_PANELS_H
#define SOLAR_PANELS_H

#include <vector>
#include "utilities/sys_model.h"
#include "utilities/dyn_effector.h"
#include "../ADCSAlgorithms/effectorInterfaces/errorConversion/vehEffectorOut.h"

//! @brief Container for Solar Panel information for hinged solar panel dynamics*/
typedef struct {
 double massSP;             //!< kg, mass of solar panel
 double ISPPntS_S[3][3];    //!< kg-m^2, Inertia of solar panel about point S in S frame components
 double d;                  //!< m, distance from hinge point to solar panel center of mass
 double k;                  //!< N-m/rad, torsional spring constant of solar panel hinge
 double c;                  //!< N-m-s/rad, rotational damping coefficient of solar panel hinge
 double r_HB_B[3];          //!< m, vector pointing from body frame origin to Hinge location
 double HB[3][3];           //!< DCM from body frame to hinge frame
}SolarPanelConfigData;

class SolarPanels: public SysModel, public DynEffector {
public:
    SolarPanels();
    ~SolarPanels();
    
    void SelfInit();
    void CrossInit();
    //! Add a new solar panel to solar panel set
    void AddSolarPanel(SolarPanelConfigData *NewSP) {SolarPanelData.push_back(*NewSP);}

public:
    std::vector<SolarPanelConfigData> SolarPanelData;     //!< -- RW information
    
private:
};

/*! @} */

#endif
