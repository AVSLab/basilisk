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

#include <stdio.h>
#include <stdlib.h>
#include "discretizeCheck.h"
#include <Eigen/Dense>
#include "utilities/avsEigenSupport.h"
#include "utilities/discretize.h"


uint64_t testDiscretize()
{
    uint64_t failures = 0;
    Discretize discretizor = Discretize(3);
    Eigen::Vector3d states;
    states << 0.1, 10.1, 11.1;
    Eigen::Vector3d LSBs;
    LSBs << 10., 10., 10.;
    discretizor.setLSB(LSBs);
    roundDirection_t roundThisWay = TO_ZERO;
    discretizor.setRoundDirection(roundThisWay);
    bool toCarryOrNotToCarry = false;
    discretizor.setCarryError(toCarryOrNotToCarry);
    Eigen::Vector3d expected;
    expected << 0, 10., 10.;
    
    states = discretizor.discretize(states);
    failures += states == expected ? 0 : 1;
    
    roundThisWay = FROM_ZERO;
    discretizor.setRoundDirection(roundThisWay);
    states << 0.1, 10.1, 11.1;
    expected << 10., 20., 20.;
    states = discretizor.discretize(states);
    failures += states == expected ? 0 : 1;
    
    roundThisWay = NEAR;
    discretizor.setRoundDirection(roundThisWay);
    states << 0.1, 10.1, 15.1;
    expected << 0, 10, 20;
    states = discretizor.discretize(states);
    failures += states == expected ? 0 : 1;
    
    discretizor = Discretize(3);
    discretizor.setLSB(LSBs);
    roundThisWay = TO_ZERO;
    discretizor.setRoundDirection(roundThisWay);
    toCarryOrNotToCarry = true;
    discretizor.setCarryError(toCarryOrNotToCarry);
    states << 0.1, 10.1, 15;
    expected << 0., 10., 10.;
    Eigen::Vector3d output;

    uint64_t numPts = 2;
    for(uint64_t i = 0; i < numPts; i++){
        output = discretizor.discretize(states);
        failures += output == expected ? 0 : 1;
        expected << 0, 10, 20;
    }
    

    return failures;
    
}




