/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef FUEL_SLOSH_H
#define FUEL_SLOSH_H



//Fuel tank models
/*! @brief This class is a class that has the defines a generic fuel slosh paricle*/
class FuelSlosh {
public:
    FuelSlosh(){return;};                  //!< -- Contructor
    virtual ~FuelSlosh(){return;};         //!< -- Destructor
    virtual void retrieveMassValue(double integTime){return;}; //!< -- retrieve current mass value of fuelSlosh particle

public:
    double fuelMass = 0.0;                 //!< [kg] mass of fuelSlosh particle
    double massToTotalTankMassRatio = 0.0; //!< -- ratio of fuelSlosh particle mass to total mass of fuel tank
    double fuelMassDot = 0.0;              //!< [kg/s] mass depletion rate of fuelSlosh particle
};


#endif /* FUEL_SLOSH_H */
