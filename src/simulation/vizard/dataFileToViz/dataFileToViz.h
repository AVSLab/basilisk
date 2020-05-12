/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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


#ifndef VIZ_DATAFILETOVIZ_H
#define VIZ_DATAFILETOVIZ_H

#include <vector>
#include <iostream>
#include <fstream>
#include "_GeneralModuleFiles/sys_model.h"
#include "utilities/bskLogging.h"


/*! Defines a data structure for the spacecraft state messages and ID's.
*/
class DataFileToViz : public SysModel {
public:
    DataFileToViz();
    ~DataFileToViz();
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    std::string dataFileName;                   //!< Name of the simulation data file
    int numSatellites = 1;                      //!< number of satellites being read in, default is 1
    std::vector<std::string> scStateOutMsgNames;//!< vector of spacecraft state messages
    std::string delimiter;                      //!< delimiter string that separates data on a line
    double convertPosToMeters;                  //!< conversion factor to meters

    BSKLogger bskLogger;                        //!< [-] BSK Logging object
    uint64_t OutputBufferCount = 2;             //!< number of output buffers for messaging system


private:
    std::vector<int64_t>  scStateOutMsgIds;     //!< vector of module output message IDs
    std::ifstream *fileHandle;                  //!< file handle to the simulation data input file
};

#endif /* VIZ_DATAFILETOVIZ_H */
