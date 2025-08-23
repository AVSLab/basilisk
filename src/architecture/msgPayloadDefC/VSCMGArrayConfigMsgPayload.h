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

#ifndef VSCMG_CONFIG_MESSAGE_H
#define VSCMG_CONFIG_MESSAGE_H

#include <stdint.h>
#include "architecture/utilities/macroDefinitions.h"

/*! @brief VSCMG array configuration FSW msg */
typedef struct{
    double Gs0Matrix_B[3*MAX_EFF_CNT];   //!< [-]    The initial VSCMGs first axis matrix in body frame components
    double Gt0Matrix_B[3*MAX_EFF_CNT];   //!< [-]    The initial VSCMGs second axis matrix in body frame components
    double GgMatrix_B[3*MAX_EFF_CNT];   //!< [-]    The initial VSCMGs third axis matrix in body frame components
    double JsList[MAX_EFF_CNT];         //!< [kgm2] The first axis inertia for VSCMGs
    double JtList[MAX_EFF_CNT];         //!< [kgm2] The second axis inertia for VSCMGs
    double JgList[MAX_EFF_CNT];         //!< [kgm2] The third axis inertia for VSCMGs
    double IwsList[MAX_EFF_CNT];        //!< [kgm2] The wheel spin axis inertia for VSCMGs
    double Omega0List[MAX_EFF_CNT]; //!< [rad/s] The initial wheel speeds for VSCMGs
    double gamma0List[MAX_EFF_CNT]; //!< [rad] The initial gimbal angles for VSCMGs
    double gammaDot0List[MAX_EFF_CNT]; //!< [rad/s] The initial gimbal rates for VSCMGs
    int    numVSCMG;                       //!< [-]    The number of VSCMGs available on vehicle
}VSCMGArrayConfigMsgPayload;


#endif
