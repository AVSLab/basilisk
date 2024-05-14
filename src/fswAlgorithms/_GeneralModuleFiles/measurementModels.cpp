/*
 ISC License

 Copyright (c) 2024, University of Colorado at Boulder

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

#include "measurementModels.h"

/*! Measurement model that takes three components of the state vector and normalizes them
 * @param Eigen::VectorXd state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd normalizedFirstThreeStates(Eigen::VectorXd state)
{
    assert(state.size() > 2);
    return state.head(3).normalized();
}

/*! Measurement model that takes components of the state vector
 * @param Eigen::VectorXd state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd firstThreeStates(Eigen::VectorXd state)
{
    assert(state.size() > 2);
    return state.head(3);
}

/*! Measurement model that takes last 3 components of the state vector
 * @param Eigen::VectorXd state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd lastThreeStates(Eigen::VectorXd state)
{
    assert(state.size() > 2);
    return state.tail(3);
}
