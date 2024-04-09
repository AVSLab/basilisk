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

#ifndef FILTER_MESSAGE_H
#define FILTER_MESSAGE_H

#include "fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h"

/*! @brief structure for filter-states output from a filter*/
typedef struct
//@cond DOXYGEN_IGNORE
FilterMsgPayload
//@endcond
{
    double timeTag;                             //!< [s] Current time of validity for output
    int numberOfStates;                             //!< [-] Number of scalar states in the state vector
    double covar[MAX_STATES_VECTOR*MAX_STATES_VECTOR];    //!< [-] Current covariance of the filter
    double state[MAX_STATES_VECTOR];                 //!< [-] Current estimated state of the filter
    double stateError[MAX_STATES_VECTOR];            //!< [-] Current deviation of the state from the reference state
}FilterMsgPayload;

#endif /* FILTER_MESSAGE_H */
