/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "facetedSpacecraftModel.h"

/*! This method resets required module variables and checks the input messages to ensure they are linked.
 @param callTime [ns] Time the method is called
*/
void FacetedSpacecraftModel::Reset(uint64_t callTime) {
}


/*! Module update method.
 @param callTime [s] Time the method is called
*/
void FacetedSpacecraftModel::UpdateState(uint64_t callTime) {
    // Write module output messages
    this->writeOutputMessages(callTime);
}


/*! Method to write module output messages.
 @param callTime [s] Time the method is called
*/
void FacetedSpacecraftModel::writeOutputMessages(uint64_t callTime) {
}
