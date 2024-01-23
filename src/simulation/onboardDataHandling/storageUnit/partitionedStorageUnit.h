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

#ifndef BASILISK_PARTITIONEDSTORAGEUNIT_H
#define BASILISK_PARTITIONEDSTORAGEUNIT_H

#include "simulation/onboardDataHandling/_GeneralModuleFiles/dataStorageUnitBase.h"
#include "architecture/utilities/macroDefinitions.h"

/*! @brief partioned storage unit class */
class PartitionedStorageUnit: public DataStorageUnitBase {

public:
    PartitionedStorageUnit();
    ~PartitionedStorageUnit();
    void addPartition(std::string dataName);
    void setDataBuffer(std::vector<std::string> partitionNames, std::vector<long long int> data); //!< Adds/removes the data from the partitionNames partitions

private:
    void customReset(uint64_t CurrentClock) override;


};

#endif //BASILISK_PARTITIONEDSTORAGEUNIT_H
