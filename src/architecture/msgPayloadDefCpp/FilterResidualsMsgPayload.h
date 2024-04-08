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

#ifndef FILTER_RES_MESSAGE_H
#define FILTER_RES_MESSAGE_H

#include "fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h"

//!@brief Optical navigation measurement from filter containing post and pre fits
/*! This message contains the output from the filtering process given a specific measurement
 */
typedef struct
//@cond DOXYGEN_IGNORE
FilterResidualsMsgPayload
//@endcond
{
    double timeTag;                             //!< [s] Current time of validity for output
    bool valid;                 //!< Quality of measurement if 1, invalid if 0
    int numberOfObservations;   //!< Number of observations in this message
    int sizeOfObservations;   //!< Size of observation vector in this message
    double observation[MAX_MEASUREMENT_VECTOR*MAX_MEASUREMENT_NUMBER];     //!< Measurement values processed
    double preFits[MAX_MEASUREMENT_VECTOR*MAX_MEASUREMENT_NUMBER];     //!< Measurement prefit residuals
    double postFits[MAX_MEASUREMENT_VECTOR*MAX_MEASUREMENT_NUMBER];     //!< Measurement postfit residuals
}FilterResidualsMsgPayload;

#endif /* FILTER_RES_MESSAGE_H */
