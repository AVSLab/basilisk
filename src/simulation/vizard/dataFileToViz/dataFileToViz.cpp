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


#include "dataFileToViz.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <sstream>
#include <string>

/*! DataFileToViz Constructor
 */
DataFileToViz::DataFileToViz()
{
    this->dataFileName = "";
    this->delimiter = " ";
    this->convertPosToMeters = 1000.;       /* convert km to meters */
    this->headerLine = true;
    this->attitudeType = 0;

    return;
}

/*! DataFileToViz Destructor
 */
DataFileToViz::~DataFileToViz()
{
    /* close the data file if it is open */
    if(this->fileHandle.is_open()) {
        this->fileHandle.close();
        bskLogger.bskLog(BSK_INFORMATION, "DataFileToViz:\nclosed the file: %s.", this->dataFileName.c_str());
    }

    for (long unsigned int c=0; c<this->scStateOutMsgs.size(); c++) {
        delete this->scStateOutMsgs.at(c);
    }

    return;
}


/*! A Reset method to put the module back into a clean state
 @param CurrentSimNanos The current sim time in nanoseconds
 */
void DataFileToViz::Reset(uint64_t CurrentSimNanos)
{
    if (this->dataFileName.length() == 0) {
        bskLogger.bskLog(BSK_ERROR, "DataFileToViz: dataFileName must be an non-empty string.");
    }
    if (this->scStateOutMsgs.size() < 1) {
        bskLogger.bskLog(BSK_ERROR, "DataFileToViz: spacecraft list must have at least one element in it.");
    }

    /* check thruster states */
    if (this->thrMsgDataSC.size() > 0) {

        if (this->scStateOutMsgs.size() != this->thrMsgDataSC.size()) {
            bskLogger.bskLog(BSK_ERROR, "DataFileToViz: you set appendThrClusterMap() %d times, but set number of spacecraft to %d", (int) this->thrMsgDataSC.size(), (int) this->scStateOutMsgs.size());
        }

        /* check vector dimensions */
        if (this->numThr != (int) this->thrPosList.size()) {
            bskLogger.bskLog(BSK_ERROR, "DataFileToViz: thrPosList must the same size as the number of thrusters.");
        }

        if (this->numThr != (int) this->thrDirList.size()) {
            bskLogger.bskLog(BSK_ERROR, "DataFileToViz: thrDirList must the same size as the number of thrusters.");
        }

        if (this->numThr != (int) this->thrForceMaxList.size()) {
            bskLogger.bskLog(BSK_ERROR, "DataFileToViz: thrForceMaxList must the same size as the number of thrusters.");
        }
    }

    /* check RW states */
    if (this->rwScOutMsgs.size() > 0) {

        if (this->scStateOutMsgs.size() != this->rwScOutMsgs.size()) {
            bskLogger.bskLog(BSK_ERROR, "DataFileToViz: you set appendRwMsg %d times, but set number of spacecraft to %d", (int) this->rwScOutMsgs.size(), (int) this->scStateOutMsgs.size());
        }

        /* check vector dimensions */
        if (this->numRW != (int) this->rwPosList.size()) {
            bskLogger.bskLog(BSK_ERROR, "DataFileToViz: rwPosList must the same size as the total number of RWs.");
        }

        if (this->numRW != (int) this->rwDirList.size()) {
            bskLogger.bskLog(BSK_ERROR, "DataFileToViz: rwDirList must the same size as the total number of RWs.");
        }

        if (this->numRW != (int) this->rwOmegaMaxList.size()) {
            bskLogger.bskLog(BSK_ERROR, "DataFileToViz: rwOmegaMaxList must the same size as the total number of RWs.");
        }

        if (this->numRW != (int) this->rwUMaxList.size()) {
            bskLogger.bskLog(BSK_ERROR, "DataFileToViz: rwUMaxList must the same size as the total number of RWs.");
        }
    }

    /* close the data file if it is open */
    if(this->fileHandle.is_open()) {
        this->fileHandle.close();
        bskLogger.bskLog(BSK_INFORMATION, "DataFileToViz:\nclosed the file: %s.", this->dataFileName.c_str());
    }

    /* open the data file*/
    this->fileHandle.open(this->dataFileName);
    if (this->fileHandle.fail()) {
        bskLogger.bskLog(BSK_ERROR, "DataFileToViz: was not able to load the file %s.", this->dataFileName.c_str());
    }
    if (this->headerLine) {
        std::string line;
        getline(this->fileHandle, line);
    }

    bskLogger.bskLog(BSK_INFORMATION, "DataFileToViz:\nloaded the file: %s.", this->dataFileName.c_str());


    return;
}

/*!
 set the number of satellites being read in
 */
void DataFileToViz::setNumOfSatellites(int numSat)
{
    for (int i=0; i<numSat; i++) {
        /* create output message */
        Message<SCStatesMsgPayload> *msg;
        msg = new Message<SCStatesMsgPayload>;
        this->scStateOutMsgs.push_back(msg);
    }
}

/*!
 Add a thruster 3d position vector to the list of thruster locations
 */
void DataFileToViz::appendThrPos(double pos_B[3])
{
    this->thrPosList.push_back(cArray2EigenVector3d(pos_B));
}

/*!
 Add a thruster 3d unit direction vector to the list.  The input vectors gets normalized before being added to the list.
 */
void DataFileToViz::appendThrDir(double dir_B[3])
{
    v3Normalize(dir_B, dir_B);
    this->thrDirList.push_back(cArray2EigenVector3d(dir_B));
}

/*!
 Add a thruster maximum force value to the list of thrusters.
 */
void DataFileToViz::appendThrForceMax(double forceMax)
{
    this->thrForceMaxList.push_back(forceMax);
}

/*!
 Add a thruster cluster map for each spacecraft
*/
void DataFileToViz::appendThrClusterMap(std::vector <ThrClusterMap> thrMsgData, std::vector<int> numThrPerCluster)
{
    this->thrMsgDataSC.push_back(thrMsgData);

    std::vector <Message<THROutputMsgPayload> *> vecMsgs;
    // loop over the number of thruster clusters for this spacecraft
    if (thrMsgData.size()>0) {
        this->numThrPerCluster.push_back(numThrPerCluster);
        for (long unsigned int thrClusterCount=0; thrClusterCount<thrMsgData.size(); thrClusterCount++) {
            // loop over the number of thrusters in this cluster and create an output message
            for (int i=0; i<numThrPerCluster[thrClusterCount]; i++) {
                /* create output message */
                Message<THROutputMsgPayload> *msg;
                msg = new Message<THROutputMsgPayload>;
                vecMsgs.push_back(msg);
            }
            this->numThr += numThrPerCluster[thrClusterCount];
        }
    }

    // add vector of thruster output messages
    this->thrScOutMsgs.push_back(vecMsgs);

}

/*!
 Add a RW output msg list for each spacecraft
*/
void DataFileToViz::appendNumOfRWs(int numRW)
{
    std::vector <Message<RWConfigLogMsgPayload>*> vecMsgs;
    for (int i=0; i<numRW; i++) {
        /* create output message */
        Message<RWConfigLogMsgPayload> *msg;
        msg = new Message<RWConfigLogMsgPayload>;
        vecMsgs.push_back(msg);
    }
    // update total number of RWs
    this->numRW += numRW;

    // add vector to RW output messages
    this->rwScOutMsgs.push_back(vecMsgs);
}


/*!
 Add a RW maximum motor torque value to the list
 */
void DataFileToViz::appendUMax(double uMax)
{
    this->rwUMaxList.push_back(uMax);
}

/*!
 Add a RW wheel rate value to the list
 */
void DataFileToViz::appendOmegaMax(double OmegaMax)
{
    this->rwOmegaMaxList.push_back(OmegaMax);
}

/*!
 Add a thruster 3d position vector to the list of thruster locations
 */
void DataFileToViz::appendRwPos(double pos_B[3])
{
    this->rwPosList.push_back(cArray2EigenVector3d(pos_B));
}

/*!
 Add a RW spin axis unit direction vector to the list.  The input vectors gets normalized before being added to the list.
 */
void DataFileToViz::appendRwDir(double dir_B[3])
{
    v3Normalize(dir_B, dir_B);
    this->rwDirList.push_back(cArray2EigenVector3d(dir_B));
}


/*! Update this module at the task rate
 @param CurrentSimNanos The current sim time
 */
void DataFileToViz::UpdateState(uint64_t CurrentSimNanos)
{
    /* ensure that a file was opened */
    if (this->fileHandle.is_open()) {
        /* read in next line*/
        std::string line;
        if (getline(this->fileHandle, line)) {

            std::istringstream iss(line);

            /* pull time, this is not used in the BSK msg */
            pullScalar(&iss);

            // create all the state output messages for each spacecraft
            for (long unsigned int scCounter=0; scCounter<this->scStateOutMsgs.size(); scCounter++) {
                SCStatesMsgPayload scMsg;

                /* zero output message */
                scMsg = this->scStateOutMsgs.at(scCounter)->zeroMsgPayload;

                /* get inertial position */
                pullVector(&iss, scMsg.r_CN_N);
                v3Scale(this->convertPosToMeters, scMsg.r_CN_N, scMsg.r_CN_N);
                v3Copy(scMsg.r_CN_N, scMsg.r_BN_N);

                /* get inertial velocity */
                pullVector(&iss, scMsg.v_CN_N);
                v3Scale(this->convertPosToMeters, scMsg.v_CN_N, scMsg.v_CN_N);
                v3Copy(scMsg.v_CN_N, scMsg.v_BN_N);

                /* get inertial attitude and rates */
                double att[4];
                if (this->attitudeType != 1) {
                    /* 3D attitude coordinate set */
                    pullVector(&iss, att);
                } else {
                    /* 4D attitude coordinate set */
                    pullVector4(&iss, att);
                }
                switch (this->attitudeType) {
                    case 0:
                        /* MRPs */
                        v3Copy(att, scMsg.sigma_BN);
                        break;
                    case 1:
                        /* quaternions (q0, q1, q2, q3) */
                        EP2MRP(att, scMsg.sigma_BN);
                        break;
                    case 2:
                        /* (3-2-1) Euler Angles */
                        Euler3212MRP(att, scMsg.sigma_BN);
                        break;
                    default:
                        bskLogger.bskLog(BSK_ERROR, "DataFileToViz: unknown attitudeType encountered: %d", this->attitudeType);
                        break;
                }
                pullVector(&iss, scMsg.omega_BN_B);

                /* write spacecraft state message */
                this->scStateOutMsgs.at(scCounter)->write(&scMsg, this->moduleID, CurrentSimNanos);

                /* check if thruster states are provided */
                if (this->thrMsgDataSC.size() > 0) {
                    if (this->thrMsgDataSC[scCounter].size() > 0) {
                        int thrCounter = 0;
                        std::vector<ThrClusterMap>::iterator thrSet;
                        int thrClusterCounter = 0;
                        for (thrSet = this->thrMsgDataSC[scCounter].begin(); thrSet!=this->thrMsgDataSC[scCounter].end(); thrSet++) {
                            for (uint32_t idx = 0; idx< (uint32_t)this->numThrPerCluster[scCounter][thrClusterCounter]; idx++) {
                                THROutputMsgPayload thrMsg;
                                thrMsg = this->thrScOutMsgs[scCounter].at(thrCounter)->zeroMsgPayload;

                                /* fill out the thruster state message */
                                thrMsg.maxThrust = this->thrForceMaxList[thrCounter];
                                thrMsg.thrustForce = pullScalar(&iss);
                                eigenVector3d2CArray(this->thrPosList[thrCounter], thrMsg.thrusterLocation);
                                eigenVector3d2CArray(this->thrDirList[thrCounter], thrMsg.thrusterDirection);

                                this->thrScOutMsgs[scCounter].at(thrCounter)->write(&thrMsg, this->moduleID, CurrentSimNanos);
                                thrCounter++;
                            }
                            thrClusterCounter++;
                        }
                    }
                }

                /* check if RW states are provided */
                if (this->rwScOutMsgs.size() > 0) {
                    if (this->rwScOutMsgs[scCounter].size() > 0) {
                        for (long unsigned int rwCounter = 0; rwCounter < this->rwScOutMsgs[scCounter].size(); rwCounter++) {

                            RWConfigLogMsgPayload rwOutMsg;
                            rwOutMsg = this->rwScOutMsgs[scCounter].at(rwCounter)->zeroMsgPayload;

                            /* create RW message */
                            rwOutMsg.Omega = pullScalar(&iss);
                            rwOutMsg.Omega_max = this->rwOmegaMaxList[rwCounter];
                            rwOutMsg.u_current = pullScalar(&iss);
                            rwOutMsg.u_max = this->rwUMaxList[rwCounter];
                            eigenVector3d2CArray(this->rwPosList[rwCounter], rwOutMsg.rWB_B);
                            eigenVector3d2CArray(this->rwDirList[rwCounter], rwOutMsg.gsHat_B);

                            this->rwScOutMsgs[scCounter].at(rwCounter)->write(&rwOutMsg, this->moduleID, CurrentSimNanos);

                        }
                    }
                }
            }
        } else {
            bskLogger.bskLog(BSK_INFORMATION, "DataFileToViz: reached end of file.");
        }
    }

    return;
}

/*! pull a 3-d set of double values from the input stream
 */
void DataFileToViz::pullVector(std::istringstream *iss, double vec[3]) {
    double x,y,z;
    x = pullScalar(iss);
    y = pullScalar(iss);
    z = pullScalar(iss);
    v3Set(x, y, z, vec);
}

/*! pull a 4-d set of double values from the input stream
 */
void DataFileToViz::pullVector4(std::istringstream *iss, double *vec) {
    double q0, q1, q2, q3;
    q0 = pullScalar(iss);
    q1 = pullScalar(iss);
    q2 = pullScalar(iss);
    q3 = pullScalar(iss);
    v4Set(q0, q1, q2, q3, vec);
}


/*! pull a double from the input stream
*/
double DataFileToViz::pullScalar(std::istringstream *iss) {
    const char delimiterString = *this->delimiter.c_str();
    std::string item;

    getline(*iss, item, delimiterString);

    return stod(item);
}
