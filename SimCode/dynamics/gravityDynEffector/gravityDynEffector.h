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


#ifndef GRAVITY_DYN_EFFECTOR_H
#define GRAVITY_DYN_EFFECTOR_H

#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "_GeneralModuleFiles/sys_model.h"
#include <vector>
#include <Eigen/Dense>

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
 */
class GravBodyData
{
public:
    bool isCentralBody;             //!<          Flag indicating that object is center
    bool isDisplayBody;             //!<          Flag indicating that body is display
    bool useSphericalHarmParams;    //!<          Flag indicating to use spherical harmonics perturbations
    Eigen::Vector3d posFromEphem;   //!< [m]      Position vector from central to body
    Eigen::Vector3d velFromEphem;   //!< [m/s]    Velocity vector from central body
    Eigen::Matrix3d j20002Pfix;     //!<          Transformation matrix from J2000 to planet-fixed
    Eigen::Matrix3d j20002Pfix_dot; //!<          Derivative of the transformation matrix from J2000 to planet-fixed
    Eigen::Vector3d posRelDisplay;  //!< [m]      Position of planet relative to display frame
    Eigen::Vector3d velRelDisplay;  //!< [m]      Velocity of planet relative to display frame
    double mu;                      //!< [m3/s^2] central body gravitational param
    double ephemTime;               //!< [s]      Ephemeris time for the body in question
    double ephIntTime;              //!< [s]      Integration time associated with the ephem data
    double radEquator;              //!< [m]      Equatorial radius for the body
    uint64_t ephemTimeSimNanos;     //!< [ns]     Simulation nanoseconds associated with Ephemeris time
    std::string bodyMsgName;        //!<          Gravitational body name
    std::string outputMsgName;      //!<          Ephemeris information relative to display frame
    std::string planetEphemName;    //!<          Ephemeris name for the planet
    int64_t outputMsgID;            //!<          ID for output message data
    int64_t bodyMsgID;              //!<          ID for ephemeris data message
    SphericalHarmonics spherHarm;   //!<          Object that computes the spherical harmonics gravity field
    
    
    // Default constructor
    GravBodyData();
    ~GravBodyData();
    
    void initBody(uint64_t moduleID); //!<        Method to initialize the gravity body
    Eigen::Vector3d computeGravityInertial(Eigen::Vector3d r_I, uint64_t simTimeNanos);
    
    // Copy constructor
    //GravBodyData(const GravBodyData& gravBody);
    
    // Overloaded operators
    //GravBodyData& operator=(const GravBodyData& gravBody);
    
};

/*! @brief Abstract class that is used to implement an effector impacting a GRAVITY body
           that does not itself maintain a state or represent a changing component of
           the body (for example: gravity, thrusters, solar radiation pressure, etc.)
 */
class GravityDynEffector : public DynamicEffector, public SysModel {
public:
    GravityDynEffector();
    ~GravityDynEffector();
    void linkInStates(const DynParamManager& statesIn);
    void updateDerivativeSums();
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);

public:
	std::string vehicleMassStateName;              //! [-] Name of the vehicle mass state
	std::string vehiclePositionStateName;          //! [-] Name of the vehicle position state
    std::vector<GravBodyData*> gravBodies;         //! [-] Vector of bodies we feel gravity from
    GravBodyData* centralBody;         //!<  Central body
private:
	StateData *massState;                          //! [-] State of the mass of the vehicle
	StateData *posState;                           //! [-] Position state of the vehicle

};

#endif /* GRAVITY_EFFECTOR_H */
