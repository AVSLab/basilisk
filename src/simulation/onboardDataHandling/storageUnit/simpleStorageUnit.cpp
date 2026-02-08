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
#include <cstdint>


#include "simpleStorageUnit.h"
#include "architecture/utilities/bskLogging.h"


/*! The constructor creates a SimpleStorageUnit instance with zero stored data

 */
SimpleStorageUnit::SimpleStorageUnit(){
    this->storageCapacity = 0;
    this->storedDataSum = 0;
    return;
}

/*! Destructor

 */
SimpleStorageUnit::~SimpleStorageUnit(){
    return;
}

/*! Custom reset function
 @param currentClock
 */
void SimpleStorageUnit::customReset(uint64_t currentClock){
    if (this->storageCapacity <= 0) {
        bskLogger.bskLog(BSK_INFORMATION, "The storageCapacity variable must be set to a positive value.");
    }
    return;
}

/*! Overwrites the integrateDataStatus method to create a single partition in the storage unit ("STORED DATA")
 @param currentTime

 */
void SimpleStorageUnit::integrateDataStatus(double currentTime){
    this->currentTimestep = currentTime - this->previousTime;
    this->netBaud = 0;

    //! - loop over all the data nodes and add them to the single partition.
    std::vector<DataNodeUsageMsgPayload>::iterator it;
    for(it = nodeBaudMsgs.begin(); it != nodeBaudMsgs.end(); it++) {
        if (storedData.size() == 0){
            this->storedData.push_back({{'S','T','O','R','E','D',' ','D','A','T','A'}, 0});
        }
        else if ((this->storedDataSum + round(it->baudRate * this->currentTimestep) < this->storageCapacity) || (it->baudRate <= 0)){
            //! If this operation takes the sum below zero, set it to zero
            if ((this->storedData[0].dataInstanceSum + it->baudRate * this->currentTimestep) >= 0) {
                this->storedData[0].dataInstanceSum += round(it->baudRate * this->currentTimestep);
            } else {
                this->storedData[0].dataInstanceSum = 0;
            }
        }
        this->netBaud += it->baudRate;
    }

    //!- Sum all data in storedData vector
    this->storedDataSum = this->storedData[0].dataInstanceSum;

    //!- Update previousTime
    this->previousTime = currentTime;
    return;
}

/*! Adds a specific amount of data to the storedData vector once
 @param data //Data to be added to the "STORED DATA" partition

 */
void SimpleStorageUnit::setDataBuffer(int64_t data){
    std::string partitionName = "STORED DATA";
    SimpleStorageUnit::DataStorageUnitBase::setDataBuffer(partitionName, data);
}
