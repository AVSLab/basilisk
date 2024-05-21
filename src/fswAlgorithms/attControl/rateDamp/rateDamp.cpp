/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "rateDamp.h"


/*! Module constructor */
RateDamp::RateDamp() = default;


/*! Module destructor */
RateDamp::~RateDamp() = default;


/*! Initialize C-wrapped output messages */
void RateDamp::SelfInit(){
    CmdTorqueBodyMsg_C_init(&this->cmdTorqueOutMsgC);
}

/*! This method is used to reset the module.
 @return void
 */
void RateDamp::Reset(uint64_t CurrentSimNanos)
{
    assert(this->attNavInMsg.isLinked());
}


/*! This method is the main carrier for the computation of the control torque.
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void RateDamp::UpdateState(uint64_t CurrentSimNanos)
{

}
