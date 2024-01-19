/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#include "sunPointRoll.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

const double epsilon = 1e-12;                           // module tolerance for zero

/*! Module constructor */
SunPointRoll::SunPointRoll() = default;


/*! Module destructor */
SunPointRoll::~SunPointRoll() = default;


/*! Initialize C-wrapped output messages */
void SunPointRoll::SelfInit(){
    AttRefMsg_C_init(&this->attRefOutMsgC);
}

/*! This method is used to reset the module.
 @return void
 */
void SunPointRoll::Reset(uint64_t CurrentSimNanos)
{
    if (!this->attNavInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, ".attNavInMsg wasn't connected.");
    }
    this->resetTime = CurrentSimNanos;
}


/*! This method is the main carrier for the boresight calculation routine.  If it detects
 that it needs to re-init (direction change maybe) it will re-init itself.
 Then it will compute the angles away that the boresight is from the celestial target.
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void SunPointRoll::UpdateState(uint64_t CurrentSimNanos)
{

}

