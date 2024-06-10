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

#include "sunSearch.h"
#include "architecture/utilities/macroDefinitions.h"
#include <cmath>

/*! Module constructor */
SunSearch::SunSearch() = default;


/*! Module destructor */
SunSearch::~SunSearch() = default;


/*! Initialize C-wrapped output messages */
void SunSearch::SelfInit(){
    AttGuidMsg_C_init(&this->attGuidOutMsgC);
}

/*! This method is used to reset the module.
 @return void
 */
void SunSearch::Reset(uint64_t CurrentSimNanos)
{
    this->resetTime = CurrentSimNanos;
}


/*! This method is the main carrier for the computation of the guidance message
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void SunSearch::UpdateState(uint64_t CurrentSimNanos)
{

}
