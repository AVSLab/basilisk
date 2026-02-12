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

#ifndef _FACETED_SC_MESSAGE_H
#define _FACETED_SC_MESSAGE_H

#include "architecture/utilities/macroDefinitions.h"
#include "FacetElementMsgPayload.h"



/*! @brief Message used to define an array of facet configurations */
typedef struct {
    int numFacets;  //!< [-] Number of total sc facets
    int numArticulatedFacets;  //!< [-] Number of articulated facets
    FacetElementMsgPayload facets[MAX_EFF_CNT];  //!< [-] Array of sc facets
}FacetedSCMsgPayload;


#endif
