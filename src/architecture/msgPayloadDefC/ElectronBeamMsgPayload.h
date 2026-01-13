/*
 ISC License Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef ELECTRON_BEAM_MSG_H
#define ELECTRON_BEAM_MSG_H

/*! @brief Message that defines the parameters of an onboard electron beam gun. */
typedef struct {
    double energyEB;   //!< [eV] Energy of the primary electron beam
    double currentEB;  //!< [A]  Primary beam current (positive value represents electrons leaving)
    double alphaEB;    //!< [-] A number from [0,1] representing the fraction of current actually reaching the target (to simulate beam divergence)
} ElectronBeamMsgPayload;

#endif
