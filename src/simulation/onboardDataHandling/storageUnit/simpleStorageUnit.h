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

#ifndef BASILISK_SIMPLESTORAGEUNIT_H
#define BASILISK_SIMPLESTORAGEUNIT_H


#include "simulation/onboardDataHandling/_GeneralModuleFiles/dataStorageUnitBase.h"
#include "architecture/utilities/macroDefinitions.h"

/*! @brief simple storage unit class */
class SimpleStorageUnit: public DataStorageUnitBase {

public:
    SimpleStorageUnit();
    ~SimpleStorageUnit();
    void setDataBuffer(int64_t data); //!< Method to add/remove data from the storage unit once

private:
    void customReset(uint64_t CurrentClock); //!< Custom Reset method
    void integrateDataStatus(double currentTime); //!< Overwrites the integrateDataStatus method to create a single partition in the storage unit ("STORED DATA")

};

#endif //BASILISK_SIMPLESTORAGEUNIT_H


