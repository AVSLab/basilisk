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
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include <vector>
#include <Eigen/Dense>
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

/*! @brief poyhedral class */
class Polyhedral
{
public:
    unsigned int nVertex;         //!< [-] Number of vertexes
    unsigned int nFacet;          //!< [-] Number of facets
    
    double volPoly;               //!< [-] Volume of the polyhedral
    double muBody;                //!< [-] Gravitation parameter for the planet
    
    Eigen::MatrixXd xyzVertex;    //!< [m] Position of vertex
    Eigen::MatrixXd orderFacet;   //!< [-] Vertexes of a facet

    Eigen::MatrixXd normalFacet;  //!< [-] Normal of a facet

    BSKLogger bskLogger;          //!< -- BSK Logging

public:

    Polyhedral();
    ~Polyhedral();
    bool initializeParameters();            //!< [-] configure polyhedral based on inputs
    Eigen::Vector3d computeField(const Eigen::Vector3d pos_Pfix);
    bool polyReady();                       //!< class variable
};

/*! @brief spherical harmonics class */
class SphericalHarmonics
{
public:
    unsigned int maxDeg;  //!< [-] Maximum degree of the spherical harmonics
    double radEquator;    //!< [-] Reference radius for the planet
    double muBody;        //!< [-] Gravitation parameter for the planet
    
    std::vector<std::vector<double>> cBar;  //!< [-] C coefficient set
    std::vector<std::vector<double>> sBar;  //!< [-] S coefficient set
    std::vector<std::vector<double>> aBar;  //!< [-] Normalized 'derived' Assoc. Legendre
    std::vector<std::vector<double>> n1;    //!< [-] What am I
    std::vector<std::vector<double>> n2;    //!< [-] What am I
    std::vector<std::vector<double>> nQuot1;//!< [-] What am I
    std::vector<std::vector<double>> nQuot2;//!< [-] What am I

    BSKLogger bskLogger;                      //!< -- BSK Logging

public:

    SphericalHarmonics();
    ~SphericalHarmonics();
    bool initializeParameters();            //!< [-] configure all spher-harm based on inputs
    double getK(const unsigned int degree); //!< class method
    Eigen::Vector3d computeField(const Eigen::Vector3d pos_Pfix, unsigned int degree,
                                                     bool include_zero_degree);
    bool harmReady();                       //!< class variable
    
};

//!@brief Container for gravitational body data
/*! This class is designed to hold all of the information for a gravity
 body.  The nominal use-case has it initialized at the python level and
 attached to dynamics using the AddGravityBody method.

 */
class GravBodyData
{
public:
    // Default constructor
    GravBodyData();
    ~GravBodyData();

    void initBody(int64_t moduleID); //!<        Method to initialize the gravity body
    Eigen::Vector3d computeGravityInertial(Eigen::Vector3d r_I, uint64_t simTimeNanos);
    double computePotentialEnergy(Eigen::Vector3d r_I);
    void loadEphemeris();           //!< Command to load the ephemeris data
    void registerProperties(DynParamManager& statesIn);  //!< class method
    ReadFunctor<SpicePlanetStateMsgPayload> planetBodyInMsg;       //!< planet spice ephemeris input message

public:
    bool isCentralBody=0;           //!<          Flag indicating that object is center
    bool usePolyhedral=0;           //!<          Flag indicating to use polyhedral model
    bool useSphericalHarmParams=0;  //!<          Flag indicating to use spherical harmonics perturbations

    double mu=0;                    //!< [m3/s^2] central body gravitational param
    double ephemTime;               //!< [s]      Ephemeris time for the body in question
    double ephIntTime;              //!< [s]      Integration time associated with the ephem data
    double radEquator=0;            //!< [m]      Equatorial radius for the body
    double radiusRatio=0;           //!< []       ratio of polar over equatorial radius
    SpicePlanetStateMsgPayload localPlanet = {};  //!< [-]   Class storage of ephemeris info from scheduled portion
    uint64_t timeWritten = 0;       //!< [ns]     time the input planet state message was written
    std::string planetName="";      //!<          Gravitational body name, this is used as the Spice name if spiceInterface is used
    std::string displayName="";     //!<          this is the name that is displayed in Vizard.  If not set, Vizard shows planetName
    std::string modelDictionaryKey = ""; //!<     "" will result in using the current default for the celestial body's given name, otherwise key will be matched if possible to available model in internal model dictionary

    Polyhedral poly;                //!<          Object that computes the polyhedral gravity field
    SphericalHarmonics spherHarm;   //!<          Object that computes the spherical harmonics gravity field
    BSKLogger bskLogger;            //!< -- BSK Logging
    Eigen::MatrixXd *r_PN_N;        //!< [m]      (state engine property) planet inertial position vector
    Eigen::MatrixXd *v_PN_N;        //!< [m/s]    (state engine property) planet inertial velocity vector
    Eigen::MatrixXd *muPlanet;      //!< [m/s]    (state engine property) planet inertial velocity vector
    Eigen::MatrixXd *J20002Pfix;    //!< [m/s]    (state engine property) planet attitude [PN]
    Eigen::MatrixXd *J20002Pfix_dot;//!< [m/s]    (state engine property) planet attitude rate [PN_dot]

};


/*! @brief gravity effector class */
class GravityEffector : public SysModel {
public:
    GravityEffector();
    ~GravityEffector();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void linkInStates(DynParamManager& statesIn); //!< class method
    void registerProperties(DynParamManager& statesIn);
    void computeGravityField(Eigen::Vector3d r_cF_N, Eigen::Vector3d rDot_cF_N);
    void updateInertialPosAndVel(Eigen::Vector3d r_BF_N, Eigen::Vector3d rDot_BF_N);
    void updateEnergyContributions(Eigen::Vector3d r_CN_N, double & orbPotEnergyContr);  //!< -- Orbital Potential Energy Contributions
    void setGravBodies(std::vector<GravBodyData *> gravBodies); //!< class method
    void addGravBody(GravBodyData* gravBody); //!< class method
    void prependSpacecraftNameToStates(); //!< class method
    
private:
    Eigen::Vector3d getEulerSteppedGravBodyPosition(GravBodyData *bodyData); //!< class method
    void writeOutputMessages(uint64_t currentSimNanos); //!< class method

public:
    std::string vehicleGravityPropName;            //!< [-] Name of the vehicle mass state
    std::string systemTimeCorrPropName;            //!< [-] Name of the correlation between times
    std::vector<GravBodyData*> gravBodies;         //!< [-] Vector of bodies we feel gravity from
    GravBodyData* centralBody;         //!<  Central body
    std::string inertialPositionPropName;           //!< [-] Name of the inertial position property
    std::string inertialVelocityPropName;           //!< [-] Name of the inertial velocity property
    std::string nameOfSpacecraftAttachedTo;         //!< [-] Name of the s/c this gravity model is attached to
    BSKLogger bskLogger;                      //!< -- BSK Logging
    Message<SpicePlanetStateMsgPayload> centralBodyOutMsg;  //!< central planet body state output message

private:
    Eigen::MatrixXd *gravProperty;                  //!< [-] g_N property for output
    Eigen::MatrixXd *timeCorr;                      //!< [-] Time correlation property
    Eigen::MatrixXd *inertialPositionProperty;      //!< [m] r_N inertial position relative to system spice zeroBase/refBase coordinate frame, property for output.
    Eigen::MatrixXd *inertialVelocityProperty;      //!< [m/s] v_N inertial velocity relative to system spice zeroBase/refBase coordinate frame, property for output.

};

#endif /* GRAVITY_EFFECTOR_H */
