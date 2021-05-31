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


#include "partitionedStorageUnit.h"
#include "architecture/utilities/bskLogging.h"


/*! The constructor creates a partitionedStorageUnit instance with zero stored data
 @return void;
 */
PartitionedStorageUnit::PartitionedStorageUnit(){
    this->storageCapacity = -1.0;
    this->storedDataSum = 0.0;
    return;
}

/*! Destructor.
 @return void
 */
PartitionedStorageUnit::~PartitionedStorageUnit(){
    return;
}

/*! Custom reset function.
 @param currentClock
 @return void
 */
void PartitionedStorageUnit::customReset(uint64_t currentClock){
    if (this->storageCapacity <= 0.0) {
        bskLogger.bskLog(BSK_INFORMATION, "The storageCapacity variable must be set to a positive value.");
    }
    return;
}

/*! Adds a partition to the storageUnit
 @param dataName
 @return void
 */
void PartitionedStorageUnit::addPartition(std::string dataName){
    dataInstance tmpDataInstance;
    strncpy(tmpDataInstance.dataInstanceName, dataName.c_str(), sizeof(tmpDataInstance.dataInstanceName));
    tmpDataInstance.dataInstanceSum = 0.0;
    this->storedData.push_back(tmpDataInstance);
    return;
}