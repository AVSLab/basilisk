/*
 ISC License

 Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef ALBEDO_H
#define ALBEDO_H

// General
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <inttypes.h>
#include <vector>
#include <string>
#include "architecture/_GeneralModuleFiles/sys_model.h"
// Utilities
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/geodeticConversion.h"
#include "architecture/utilities/linearAlgebra.h"
// Sim Messages
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/AlbedoMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/macroDefinitions.h"

/*! albedo instrument configuration class */
typedef class Config {
public:
    Config();
    ~Config();
    double fov;              //!< [rad] instrument's field of view half angle
    Eigen::Vector3d nHat_B;  //!< [-] unit normal of the instrument (spacecraft body)
    Eigen::Vector3d r_IB_B;  //!< [m] instrument's misalignment wrt spacecraft's body frame
} instConfig_t;

/*! @brief albedo class */
class Albedo : public SysModel {
public:
    Albedo();
    ~Albedo();

    void UpdateState(uint64_t CurrentSimNanos);               //!< @brief updates the state
    void Reset(uint64_t CurrentSimNanos);                     //!< @brief resets the module

    void addInstrumentConfig(instConfig_t configMsg);  //!< @brief adds instrument configuration (overloaded function)
    void addInstrumentConfig(double fov, Eigen::Vector3d nHat_B, Eigen::Vector3d r_IB_B);  //!< @brief adds instrument configuration (overloaded function)
    void addPlanetandAlbedoAverageModel(Message<SpicePlanetStateMsgPayload> *msg); //!< @brief This method adds planet msg and albedo average model name (overloaded function)
    void addPlanetandAlbedoAverageModel(Message<SpicePlanetStateMsgPayload> *msg, double ALB_avg, int numLat, int numLon);  //!< @brief This method adds planet name and albedo average model name (overloaded function)
    void addPlanetandAlbedoDataModel(Message<SpicePlanetStateMsgPayload> *msg, std::string dataPath, std::string fileName); //!< @brief This method adds planet name and albedo data model
    double getAlbedoAverage(std::string planetSpiceName);     //!< @brief gets the average albedo value of the specified planet

private:
    void readMessages();                                      //!< reads the inpt messages
    void writeMessages(uint64_t CurrentSimNanos);             //!< writes the outpus messages
    void getPlanetRadius(std::string planetSpiceName);        //!< gets the planet's radius
    void evaluateAlbedoModel(int idx);                        //!< evaluates the ALB model
    void computeAlbedo(int idx, int instIdx, SpicePlanetStateMsgPayload planetMsg, bool AlbArray, double outData[]); //!< computes the albedo at instrument's location
    double computeEclipseAtdA(double Rplanet, Eigen::Vector3d r_dAP_N, Eigen::Vector3d r_SP_N); //!< computes the shadow factor at dA

public:
    std::vector<Message<AlbedoMsgPayload>*> albOutMsgs;         //!< vector of output messages for albedo data
    ReadFunctor<SpicePlanetStateMsgPayload> sunPositionInMsg;   //!< input message name for sun data
    ReadFunctor<SCStatesMsgPayload> spacecraftStateInMsg;   //!< input message name for spacecraft data
    std::vector<ReadFunctor<SpicePlanetStateMsgPayload>> planetInMsgs; //!< vector of planet data input data

    BSKLogger bskLogger;                        //!< BSK Logging    
    int defaultNumLat;                                 //!< [-] default number of latitude grid points
    int defaultNumLon;                                 //!< [-] default number of longitude grid points
    Eigen::Vector3d nHat_B_default;             //!< [-] default value for unit normal of the instrument (spacecraft body)
    double fov_default;                         //!< [rad] default value for instrument's field of view half angle
    bool eclipseCase;                           //!< consider eclipse at dA, if true
    double shadowFactorAtdA;                    //!< [-] shadow factor at incremental area
    double altitudeRateLimit;                   //!< [-] rate limit of the instrument's altitude to the planet's radius for albedo calculations

private:
    std::vector<std::string> dataPaths;           //!< string with the path to the ALB coefficient folder
    std::vector<std::string> fileNames;           //!< file names containing the ALB coefficients
    std::vector<std::string> modelNames;          //!< albedo model names
    std::vector<double> ALB_avgs;                 //!< [-] albedo average value vector for each planet defined
    std::vector<double> REQ_planets, RP_planets;  //!< [m] equatorial and polar radius of the planets
    Eigen::MatrixXd albLon, albLat;          //!< sunlit area seen by the instroment fov
    double scLon, scLat;                     //!< [deg, deg] spaceccraft footprint
    std::vector<int> numLats, numLons;       //!< [-] vector of latitude and longitude number
    double albedoAtInstrument;               //!< [-] total albedo at instrument location
    double albedoAtInstrumentMax;            //!< [-] max total albedo at instrument location
    double SfluxAtInstrument;                //!< [W/m2] solar flux at instrument's position
    double AfluxAtInstrumentMax;             //!< [W/m2] max albedo flux at instrument's position
    double AfluxAtInstrument;                //!< [W/m2] albedo flux at instrument's position
    std::vector < Eigen::Vector3d > r_IB_Bs; //!< [m] instrument's misalignment vector wrt spacecraft's body frame
    std::vector < Eigen::Vector3d > nHat_Bs; //!< [-] unit normal vector of the instrument (spacecraft body)
    std::vector < double > fovs;             //!< [rad] vector containing instrument's field of view half angle
    std::vector < Eigen::Vector4d > albOutData;     //!< the vector that keeps the albedo output data
    std::map < int, std::vector < double > > gdlat; //!< [rad] geodetic latitude
    std::map < int, std::vector < double > > gdlon; //!< [rad] geodetic longitude
    std::map < int, double > latDiff;               //!< [rad] latitude difference between grid points
    std::map < int, double > lonDiff;               //!< [rad] longitude difference between grid points
    std::map < int, std::vector < std::vector < double > > > ALB; //!< [-] ALB coefficients
    bool readFile;                          //!< defines if there is a need for reading an albedo model file or not
    std::vector<bool> albArray;             //!< defines if the albedo data is formatted as array or not
    Eigen::Vector3d r_PN_N;                 //!< [m] planet position (inertial)
    Eigen::Vector3d r_SN_N;                 //!< [m] sun position (inertial)
    Eigen::MRPd sigma_BN;                   //!< [-] Current spaceraft MRPs (inertial)
    Eigen::Vector3d r_BN_N;                 //!< [m] s/c position (inertial)
    Eigen::Vector3d nHat_N;                 //!< [-] Unit normal vector of the instrument (inertial)
    Eigen::Vector3d rHat_PI_N;              //!< [-] direction vector from instrument to planet

    std::vector<SpicePlanetStateMsgPayload> planetMsgData; //!< vector of incoming planet message states
    SpicePlanetStateMsgPayload sunMsgData;      //!< sun message data
    SCStatesMsgPayload scStatesMsgData;     //!< spacecraft message data
};

#endif /* ALBEDO_BASE_H */
