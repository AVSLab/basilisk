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

#ifndef BASILISK_SIMPLEINSTRUMENT_H
#define BASILISK_SIMPLEINSTRUMENT_H

#include "simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h"

/*! @brief simple instrument data handling class */
class SimpleInstrument: public DataNodeBase {

public:
    SimpleInstrument();
    ~SimpleInstrument();

private:
    void evaluateDataModel(DataNodeUsageMsgPayload *dataUsageMsg, double currentTime); //!< Sets the name and baud rate for the data in the output message.

};


#endif //BASILISK_SIMPLEINSTRUMENT_H
