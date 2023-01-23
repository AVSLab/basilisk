/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/messaging/messaging.h"
#include <vector>
#include <Eigen/Dense>

#define Q0  1.60217663e-19  // elementary charge [C]

/*! @brief This module computes the equilibrium electric potential of any spacecaft that are added to the module
 */
class ScCharging: public SysModel {
// public functions
public:
    // Constructor And Destructor
    ScCharging();
    ~ScCharging();

// public variables
public:
    BSKLogger bskLogger;                                        //!< -- BSK Logging

};


#endif
