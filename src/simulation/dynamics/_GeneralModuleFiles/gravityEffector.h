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


#ifndef GRAVITY_DYN_EFFECTOR_H
#define GRAVITY_DYN_EFFECTOR_H

#include "dynamicEffector.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/system_messaging.h"
#include <vector>
#include <Eigen/Dense>
#include "simMessages/spicePlanetStateSimMsg.h"

class SphericalHarmonics
{
public:
    double maxDeg;        //! [-] Maximum degree of the spherical harmonics
    double radEquator;    //! [-] Reference radius for the planet
    double muBody;        //! [-] Gravitation parameter for the planet
    
    std::vector<std::vector<double>> cBar;  //! [-] C coefficient set
    std::vector<std::vector<double>> sBar;  //! [-] S coefficient set
    std::vector<std::vector<double>> aBar;  //! [-] Normalized 'derived' Assoc. Legendre
    std::vector<std::vector<double>> n1;    //! [-] What am I
    std::vector<std::vector<double>> n2;    //! [-] What am I
    std::vector<std::vector<double>> nQuot1;//! [-] What am I
    std::vector<std::vector<double>> nQuot2;//! [-] What am I
    
public:

    SphericalHarmonics();
    ~SphericalHarmonics();
    bool initializeParameters();            //! [-] configure all spher-harm based on inputs
    double getK(const unsigned int degree);
    Eigen::Vector3d computeField(const Eigen::Vector3d pos_Pfix, unsigned int degree,
                                                     bool include_zero_degree);
    bool harmReady();
    
};

//!@brief Container for gravitational body data
/*! This class is designed to hold all of the information for a gravity
 body.  The nominal use-case has it initialized at the python level and
 attached to dynamics using the AddGravityBody method.

 The module
 [PDF Description](Basilisk-GravityEffector-20170712.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 */
class GravBodyData
{
public:
    // Default constructor
    GravBodyData();
    ~GravBodyData();
    
    void initBody(uint64_t moduleID); //!<        Method to initialize the gravity body
    Eigen::Vector3d computeGravityInertial(Eigen::Vector3d r_I, uint64_t simTimeNanos);
    double computePotentialEnergy(Eigen::Vector3d r_I);
    void loadEphemeris(uint64_t moduleID); //!< Command to load the ephemeris data
    
public:
    bool isCentralBody;             //!<          Flag indicating that object is center
    bool isDisplayBody;             //!<          Flag indicating that body is display
    bool useSphericalHarmParams;    //!<          Flag indicating to use spherical harmonics perturbations
    
    double mu;                      //!< [m3/s^2] central body gravitational param
    double ephemTime;               //!< [s]      Ephemeris time for the body in question
    double ephIntTime;              //!< [s]      Integration time associated with the ephem data
    double radEquator;              //!< [m]      Equatorial radius for the body
    SpicePlanetStateSimMsg localPlanet;//!< [-]      Class storage of ephemeris info from scheduled portion
    SingleMessageHeader localHeader;//!  [-]      Header information for ephemeris storage
    std::string bodyInMsgName;      //!<          Gravitational body name
    std::string outputMsgName;      //!<          Ephemeris information relative to display frame
    std::string planetEphemName;    //!<          Ephemeris name for the planet
    int64_t outputMsgID;            //!<          ID for output message data
    int64_t bodyMsgID;              //!<          ID for ephemeris data message
    SphericalHarmonics spherHarm;   //!<          Object that computes the spherical harmonics gravity field
    
};

/*! @brief Abstract class that is used to implement an effector impacting a GRAVITY body
           that does not itself maintain a state or represent a changing component of
           the body (for example: gravity, thrusters, solar radiation pressure, etc.)
 */
class GravityEffector : public SysModel {
public:
    GravityEffector();
    ~GravityEffector();
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void linkInStates(DynParamManager& statesIn);
    void registerProperties(DynParamManager& statesIn);
    void computeGravityField(Eigen::Vector3d r_cF_N, Eigen::Vector3d rDot_cF_N);
    void updateInertialPosAndVel(Eigen::Vector3d r_BF_N, Eigen::Vector3d rDot_BF_N);
    void updateEnergyContributions(Eigen::Vector3d r_CN_N, double & orbPotEnergyContr);  //!< -- Orbital Potential Energy Contributions
    void setGravBodies(std::vector<GravBodyData *> gravBodies);
    void addGravBody(GravBodyData* gravBody);
    void prependSpacecraftNameToStates();
    
private:
    Eigen::Vector3d getEulerSteppedGravBodyPosition(GravBodyData *bodyData);
    void writeOutputMessages(uint64_t currentSimNanos);
    
public:
    std::string vehicleGravityPropName;            //! [-] Name of the vehicle mass state
    std::string systemTimeCorrPropName;            //! [-] Name of the correlation between times
    std::vector<GravBodyData*> gravBodies;         //! [-] Vector of bodies we feel gravity from
    GravBodyData* centralBody;         //!<  Central body
    std::string inertialPositionPropName;           //! [-] Name of the inertial position property
    std::string inertialVelocityPropName;           //! [-] Name of the inertial velocity property
    std::string nameOfSpacecraftAttachedTo;         //! [-] Name of the s/c this gravity model is attached to
    
private:
    Eigen::MatrixXd *gravProperty;                  //! [-] g_N property for output
    Eigen::MatrixXd *timeCorr;                      //! [-] Time correlation property
    int64_t centralBodyOutMsgId;                //! [-] Id for the central body spice data output message
    std::string centralBodyOutMsgName;              //! [-] Unique name for the central body spice data output message
    Eigen::MatrixXd *inertialPositionProperty;             //! [m] r_N inertial position relative to system spice zeroBase/refBase coordinate frame, property for output.
    Eigen::MatrixXd *inertialVelocityProperty;             //! [m/s] v_N inertial velocity relative to system spice zeroBase/refBase coordinate frame, property for output.

};

#endif /* GRAVITY_EFFECTOR_H */
