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
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "simulation/vizard/_GeneralModuleFiles/vizStructures.h"
#include <Eigen/Dense>



/*! Defines a data structure for the spacecraft state messages and ID's.
*/
class DataFileToViz : public SysModel {
public:
    DataFileToViz();
    ~DataFileToViz();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void appendThrPos(double pos_B[3]);
    void appendThrDir(double dir_B[3]);
    void appendThrForceMax(double);
    void appendThrClusterMap(std::vector <ThrClusterMap> thrMsgData, std::vector<int> numThrPerCluster);
    void appendRwPos(double pos_B[3]);
    void appendRwDir(double dir_B[3]);
    void appendOmegaMax(double);
    void appendUMax(double);
    void setNumOfSatellites(int numSat);
    void appendNumOfRWs(int numRW);


private:
    void pullVector(std::istringstream *iss, double *);
    void pullVector4(std::istringstream *iss, double *);
    double pullScalar(std::istringstream *iss);

public:
    std::string dataFileName;                   //!< Name of the simulation data file

    std::vector<Message<SCStatesMsgPayload>*> scStateOutMsgs;//!< vector of spacecraft state messages
    std::string delimiter;                      //!< delimiter string that separates data on a line
    double convertPosToMeters;                  //!< conversion factor to meters
    bool headerLine;                            //!< [bool] flag to mark first line as a header
    int attitudeType;                           //!< 0 - MRP, 1 - EP or quaternions (q0, q1, q2, q3), 2 - (3-2-1) Euler angles

    std::vector <std::vector <ThrClusterMap>> thrMsgDataSC;  //!< (Optional) vector of sets of thruster cluster mapping info
    std::vector <std::vector <Message<THROutputMsgPayload>*>> thrScOutMsgs;  //!< (Optional) vector of spacecraft thruster output message vectors
    std::vector <std::vector <Message<RWConfigLogMsgPayload>*>> rwScOutMsgs; //!< (Optional) vector of sets of RW msg names, each entry is per SC

    BSKLogger bskLogger;                        //!< [-] BSK Logging object


private:
    std::vector<std::vector<int>> numThrPerCluster;  //!< vector containing list of numbers of thruster per cluster per spacecraft
    std::ifstream fileHandle;                  //!< file handle to the simulation data input file
    std::vector <Eigen::Vector3d> thrPosList;   //!< [m] vector of thrust positions
    std::vector <Eigen::Vector3d> thrDirList;   //!< [-] vector of thrust unit direction vectors in B-frame components
    std::vector <double> thrForceMaxList;       //!< [-] vector of thrust maximum force values
    std::vector <Eigen::Vector3d> rwPosList;    //!< [m] vector of RW positions
    std::vector <Eigen::Vector3d> rwDirList;    //!< [-] vector of RW sprin axis unit direction vectors in B-frame components
    std::vector <double> rwOmegaMaxList;        //!< [r/s] vector of RW maximum spin rate values
    std::vector <double> rwUMaxList;            //!< [N] vector of RW maximum motor torque values values
    int numRW = 0;                              //!< -- number of RWs across all spacecraft
    int numThr = 0;                             //!< -- number of Thrusters across all spacecraft
};

#endif /* VIZ_DATAFILETOVIZ_H */
