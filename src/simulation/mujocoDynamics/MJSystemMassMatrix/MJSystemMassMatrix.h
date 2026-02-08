/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef MJSYSTEMMASSMATRIX_H
#define MJSYSTEMMASSMATRIX_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefCpp/MJSysMassMatrixMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/MJScene.h"

/*! @brief This is a C++ module to extract the system mass matrix from Mujoco
 */
class MJSystemMassMatrix: public SysModel {
public:
    MJSystemMassMatrix() = default; /*! This is the constructor for the module class. */
    ~MJSystemMassMatrix() = default; /*! This is the destructor for the module class. */

    void Reset(uint64_t CurrentSimNanos); /*! This method is used to reset the module and checks that the scene is setup. */
    void UpdateState(uint64_t CurrentSimNanos); /*! This method is used to extract the total spacecraft mass matrix from the MuJoCo scene. */

public:

    MJScene* scene{nullptr};  //!< pointer to the MuJoCo scene

    Message<MJSysMassMatrixMsgPayload> massMatrixOutMsg;  //!< system mass matrix C++ output msg in generalized coordinates

    BSKLogger bskLogger;              //!< BSK Logging
private:

    std::size_t nDOF{0};                              //!< number of total DOF
    std::size_t nSC{0};                               //!< number of spacecraft in the system
    std::vector<std::size_t> scStartIdx;              //!< list of the first joint index for each spacecraft
    std::vector<std::size_t> jointTypes;              //!< list of joint types in the system
};


#endif
