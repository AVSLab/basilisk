/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef VSCMG_CONFIG_ELEMENT_MESSAGE_H
#define VSCMG_CONFIG_ELEMENT_MESSAGE_H

#include <stdint.h>
#include "architecture/utilities/macroDefinitions.h"

/*! @brief VSCMG array configuration FSW msg */
typedef struct{
    double gsHat0_B[3];   //!< [-]    The initial VSCMG first axis matrix in body frame components
    double gtHat0_B[3];   //!< [-]    The initial VSCMG second axis matrix in body frame components
    double ggHat_B[3];   //!< [-]    The VSCMG third axis matrix in body frame components
    double Js;         //!< [kgm2] The first axis inertia for VSCMG
    double Jt;         //!< [kgm2] The second axis inertia for VSCMG
    double Jg;         //!< [kgm2] The third axis inertia for VSCMG
    double Iws;       //!< [kgm2] The wheel spin axis inertia for VSCMG
    double Omega0;   //!< [rad/s] The initial wheel speed for VSCMG
    double gamma0;   //!< [rad] The initial gimbal angle for VSCMG
    double gammaDot0; //!< [rad/s] The initial gimbal rate for VSCMG
}VSCMGConfigElementMsgPayload;


#endif
