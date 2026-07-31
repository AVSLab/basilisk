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

#include "saturate.h"
#include <iostream>
#include <math.h>

/*! The constructor initialies the random number generator used for the walks*/
Saturate::Saturate()
{
    this->numStates = 0;
}

Saturate::Saturate(int64_t size) : Saturate() {
    this->numStates = size;
    this->stateBounds.resize(numStates, 2);
}
/*! The destructor is a placeholder for one that might do something*/
Saturate::~Saturate()
{
}

/*! Saturate an output vector using the configured bounds.

    @param unsaturatedStates Vector of unsaturated states.
    @return Vector of saturated states.
*/
Eigen::VectorXd Saturate::saturate(const Eigen::Ref<const Eigen::VectorXd>& unsaturatedStates)
{
    Eigen::VectorXd workingStates;
    workingStates.resize(this->numStates);
    for (int64_t i = 0; i < this->numStates; i++){
        workingStates[(int) i] = std::min(unsaturatedStates[i], this->stateBounds(i, 1));
        workingStates[(int) i] = std::max(workingStates[i], this->stateBounds(i, 0));
    }
    return workingStates;

}

/*! Set the lower and upper bounds for each state.

    @param bounds One row per state, with lower and upper bounds in the two columns.
*/
void Saturate::setBounds(const Eigen::Ref<const Eigen::MatrixX2d>& bounds) {
    this->stateBounds = bounds;
}
