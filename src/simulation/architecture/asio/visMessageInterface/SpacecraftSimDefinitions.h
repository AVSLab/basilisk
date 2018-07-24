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
/*
* SpacecraftSimDefinitions.hpp
*
*/

#ifndef SPACECRAFT_SIM_DEFINITIONS_H
#define SPACECRAFT_SIM_DEFINITIONS_H

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <stdio.h>
extern "C" {
#include "spacecraftDefinitions.h"
}

class RWSim
{
public:
    ComponentState_t state;
    double           u_current;                      /*!< Nm, commanded torque */
    double           u_max;                   /*!< Nm, Max torque allowed */
    double           u_min;                   /*!< Nm, Max torque allowed */
    double           Omega;                  /*!< spin rate relative to body (rad/sec) */
    double           theta;
    double           Omega_max;
    double           Js;                     /*!< spin inertia (kgm^s) */
    double           rWB_S[3];               /*!< m, position vector of the RW relative to the spacecraft structure frame */
    double           gsHat_S[3];             /*!< spin axis in body frame */
    double           w2Hat0_B[3];            /*!< initial RW Wheel transfer axis */
    double           w3Hat0_B[3];            /*!< initial RW Wheel 3rd axis */
    double           wheelAngle;             /*!< rad, angle of the RW wheel */
    double           staticImbalance;
    double           dynamicImbalance;
    double           motorTemp1;             /*!< temperature of RW */
    double           motorTemp2;             /*!< temperature of RW */
    double           power;                  /*!< power usage of RW */
//    TempSpeedState_t speedState;             /*!< if spin rate is > 6000 rpm */
//    TempSpeedState_t tempState;              /*!< if temperature is > 80 C */
    unsigned char    input[RW_CMD_SIZE + 1];
    size_t           inputLength;
    unsigned char    output[RW_TELEM_SIZE + RW_CMD_SIZE + 1];
    size_t           outputLength;
    unsigned char    resetCounter;           /*!< flag that indicates if the fault logic has commanded a device reset */
    
    double frictionTimeConstant; /*!< time constant for RW friction */
    double highSpeed;            /*!< rad/s, max speed before speed limiting occurs */
    double maxTemp;              /*!< C, max temp before temperature limiting occurs */
    
    RWSim();
    ~RWSim();
    
private:
    friend class boost::serialization::access;
    template<typename Archive>
    void serialize(Archive &a, const unsigned int version) {
        a &BOOST_SERIALIZATION_NVP(state);
        a &BOOST_SERIALIZATION_NVP(u_current);
        a &BOOST_SERIALIZATION_NVP(u_max);
        a &BOOST_SERIALIZATION_NVP(u_min);
        a &BOOST_SERIALIZATION_NVP(Omega);
        a &BOOST_SERIALIZATION_NVP(Omega_max);
        a &BOOST_SERIALIZATION_NVP(Js);
        a &BOOST_SERIALIZATION_NVP(gsHat_S);
        a &BOOST_SERIALIZATION_NVP(w2Hat0_B);
        a &BOOST_SERIALIZATION_NVP(w3Hat0_B);
        a &BOOST_SERIALIZATION_NVP(wheelAngle);
        a &BOOST_SERIALIZATION_NVP(staticImbalance);
        a &BOOST_SERIALIZATION_NVP(dynamicImbalance);
        a &BOOST_SERIALIZATION_NVP(rWB_S);
        a &BOOST_SERIALIZATION_NVP(motorTemp1);
        a &BOOST_SERIALIZATION_NVP(motorTemp2);
        a &BOOST_SERIALIZATION_NVP(power);
        a &boost::serialization::make_array(input, numElems(input));
        a &BOOST_SERIALIZATION_NVP(inputLength);
        a &boost::serialization::make_array(output, numElems(output));
        a &BOOST_SERIALIZATION_NVP(outputLength);
        a &BOOST_SERIALIZATION_NVP(resetCounter);
        a &BOOST_SERIALIZATION_NVP(frictionTimeConstant);
        a &BOOST_SERIALIZATION_NVP(highSpeed);
        a &BOOST_SERIALIZATION_NVP(maxTemp);
    }
};


class Thruster
{
public:
    double              r_B[3];                 /*!< position vector to origin of thruster frame in body frame */
    double              gt_B[3];                /*!< nominal thrust axis in body frame */
    double              maxThrust;              /*!< maximum thrust capability (N) */
    double              minThrust;              /*!< minimum thrust capability (N) */
    double              timeStart;              /*!< Time to begin thrust */
    double              duration;               /*!< length of burn */
    CommandedState_t    thrustState;            /*!< thruster ON/OFF state */
    double              pulseTime;              /*!< thruster ON time per control period */
    bool                isValveOpen;            /*!< Flag for whether the thruster valve is open or closed */
    int                 pairId;                 /*!< Thruster pair id */
    double              tau_B[3];               /*!< Body axis about which thruster torque acts */
    double              level;                  /*!< thruster duty cycle percentage */
    
private:
    unsigned char       input[TH_CMD_SIZE + TH_CMD_CODE_SIZE];
    size_t              inputLength;
    unsigned char       output[TH_TELEM_SIZE];
    size_t              outputLength;
    
public:
    Thruster();
    Thruster(double r_B[3], double gt_B[3]);
    ~Thruster();
    
private:
    
    friend class boost::serialization::access;
    template<typename Archive>
    void serialize(Archive &a, const unsigned int version) {
        a &BOOST_SERIALIZATION_NVP(r_B);
        a &BOOST_SERIALIZATION_NVP(gt_B);
        a &BOOST_SERIALIZATION_NVP(maxThrust);
        a &BOOST_SERIALIZATION_NVP(minThrust);
        a &BOOST_SERIALIZATION_NVP(timeStart);
        a &BOOST_SERIALIZATION_NVP(duration);
        a &BOOST_SERIALIZATION_NVP(thrustState);
        a &BOOST_SERIALIZATION_NVP(pulseTime);
        a &BOOST_SERIALIZATION_NVP(isValveOpen);
        a &BOOST_SERIALIZATION_NVP(pairId);
        a &BOOST_SERIALIZATION_NVP(tau_B);
        a &BOOST_SERIALIZATION_NVP(level);
        a &BOOST_SERIALIZATION_NVP(input);
        a &BOOST_SERIALIZATION_NVP(inputLength);
        a &BOOST_SERIALIZATION_NVP(output);
        a &BOOST_SERIALIZATION_NVP(outputLength);
    }
};

typedef enum {
    CSSFAULT_OFF,           /*!< CSS measurement is set to 0 for all future time */
    CSSFAULT_STUCK_CURRENT, /*!< CSS measurement is set to current value for all future time */
    CSSFAULT_STUCK_MAX,     /*!< CSS measurement is set to maximum value for all future time */
    CSSFAULT_STUCK_RAND,    /*!< CSS measurement is set to randomly selected value for all future time */
    CSSFAULT_STUCK,         /*!< CSS measurement is set to percent value for all future time */
    CSSFAULT_RAND,          /*!< CSS measurement returns uniformly distributed random values between 0 and max */
    MAX_CSSFAULT
} CSSFaultState_t;

class CoarseSunSensor {
public:
    ComponentState_t    state;
    CSSFaultState_t     faultState;             /*!< Specification used if state is set to COMPONENT_FAULT */
    double              stuckPercent;           /*!< percent of full value the CSS will remain stuck at if a fault is triggered */
    double              theta;                  /*!< rad, css azimuth angle, measured positive from the body +x axis around the +z axis */
    double              phi;                    /*!< rad, css elevation angle, measured positive toward the body +z axis from the x-y plane */
    double              B2P321Angles[3];
    double              PB[3][3];               /*!< DCM from platform frame P to body frame B */
    double              nHatB[3];               /*!< css unit direction vector in body frame components */
    double              horizonPlane[3];        /*!< unit direction vector defining horizon cut off plane of CSS */
    double              directValue;            /*!< direct solar irradiance measurement */
    double              albedoValue;            /*!< albedo irradiance measurement */
    double              scaleFactor;            /*!< scale factor applied to sensor (common + individual multipliers) */
    double              sensedValue;            /*!< total measurement including perturbations */
    unsigned char       output[CSS_TELEM_SIZE];
    size_t              outputLength;
    double              fov;                    /*!< rad, field of view half angle */
    double              maxVoltage;             /*!< max voltage measurable by CSS, used in discretization */
    double              r_B[3];                 /*!< position vector to origin of css frame in body frame */
    
    CoarseSunSensor();
    ~CoarseSunSensor();
    
private:
    //    double wrapTo2PI(double angle);
    //    double wrapToHalfPI(double angle);
    
    friend class boost::serialization::access;
    template<typename Archive>
    void serialize(Archive &a, const unsigned int version) {
        a &BOOST_SERIALIZATION_NVP(state);
        a &BOOST_SERIALIZATION_NVP(faultState);
        a &BOOST_SERIALIZATION_NVP(stuckPercent);
        a &BOOST_SERIALIZATION_NVP(theta);
        a &BOOST_SERIALIZATION_NVP(phi);
        a &BOOST_SERIALIZATION_NVP(PB);
        a &BOOST_SERIALIZATION_NVP(nHatB);
        a &BOOST_SERIALIZATION_NVP(horizonPlane);
        a &BOOST_SERIALIZATION_NVP(directValue);
        a &BOOST_SERIALIZATION_NVP(albedoValue);
        a &BOOST_SERIALIZATION_NVP(scaleFactor);
        a &BOOST_SERIALIZATION_NVP(sensedValue);
        a &boost::serialization::make_array(output, numElems(output));
        a &BOOST_SERIALIZATION_NVP(outputLength);
        a &BOOST_SERIALIZATION_NVP(fov);
        a &BOOST_SERIALIZATION_NVP(maxVoltage);
        a &BOOST_SERIALIZATION_NVP(r_B);
    }
};

class TAM;

/* Integration Method */
typedef enum {
    INTEGRATION_RUNGE_KUTTA_4,
    INTEGRATION_EULER,
    MAX_INTEGRATION_METHOD
} IntegrationMethod_t;

/* Simulation behaviors */
typedef enum {
    CODE_STANDALONE,
    CODE_REALITY,
    CODE_REALITY_ON_DEMAND,
    CODE_FSW,
    MAX_CODE_BEHAVIOR
}
CodeBehavior_t;

/* MC print functions */
typedef enum {
    MCPRINT_CSSEST,                             /* printMCStatesCssEst */
    MAX_MCPRINT
} MCPrintFunction_t;


//! The SPICE time output structure outputs time information to the rest of the system
class SpiceTime
{
public:
    double J2000Current;        //!< s Current J2000 elapsed time
    double julianDateCurrent;   //!< s Current JulianDate
    double GPSSeconds;          //!< s Current GPS seconds
    uint16_t GPSWeek;           //!< -- Current GPS week value
    uint64_t GPSRollovers;      //!< -- Count on the number of GPS rollovers
    
    SpiceTime();
    ~SpiceTime();
    
private:
    friend class boost::serialization::access;
    template<typename Archive>
    void serialize(Archive &a, const unsigned int version) {
        a &BOOST_SERIALIZATION_NVP(J2000Current);
        a &BOOST_SERIALIZATION_NVP(julianDateCurrent);
        a &BOOST_SERIALIZATION_NVP(GPSSeconds);
        a &BOOST_SERIALIZATION_NVP(GPSWeek);
        a &BOOST_SERIALIZATION_NVP(GPSRollovers);
    }
};


/* Initial conditions */
class InitialConditions
{
public:
    double      SMA;           /* km */
    double      eccentricity;
    double      inclination;   /* rad */
    double      ascendingNode; /* hrs since midnight */
    double      argPeriapse;   /* rad */
    double      trueAnomaly;   /* rad */
    double      ET0;           /* seconds past J2000, spice Ephemeris Time (ET) */
    double      JD0;           /* Julian date */
    std::string simulationDateStr;
    double      M0;            /* initial mean anomaly */
    double      gamma0;        /* rad, earth's rotation angle from J2000 at start of mission */

    InitialConditions();
    ~InitialConditions();

private:
    friend class boost::serialization::access;
    template<typename Archive>
    void serialize(Archive &a, const unsigned int version) {
        a &BOOST_SERIALIZATION_NVP(SMA);
        a &BOOST_SERIALIZATION_NVP(eccentricity);
        a &BOOST_SERIALIZATION_NVP(inclination);
        a &BOOST_SERIALIZATION_NVP(ascendingNode);
        a &BOOST_SERIALIZATION_NVP(argPeriapse);
        a &BOOST_SERIALIZATION_NVP(trueAnomaly);
        a &BOOST_SERIALIZATION_NVP(ET0);
        a &BOOST_SERIALIZATION_NVP(JD0);
        a &BOOST_SERIALIZATION_NVP(simulationDateStr);
        a &BOOST_SERIALIZATION_NVP(M0);
        a &BOOST_SERIALIZATION_NVP(gamma0);
    }
};

/* Control parameters */
class ControllerSim
{
public:
    double sigma_BR[3];                 /*!< MRP attitude tracking error */
    double omega_BR[3];                 /*!< attitude rate tracking error */
    double sigma_BR_SP[3];              /*!< Sun-pointing attitude error */
    double sigma_BR_NP[3];              /*!< Hill-Pointing attitude error */
    double sigma_BR_NS[3];              /*!< Hill-Spin-Pointing attitude error */
    double sigma_BR_VP[3];              /*!< Velocity-Pointing attitude error */
    double sigma_BR_VS[3];              /*!< Velocity-Spin attitude error */
    double sigma_BR_EP[3];              /*!< Earth pointing attitud error */
    double sigma_BR_IP[3];              /*!< Inertial pointing attitude errors */
    double sigma_BR_IS[3];              /*!< Inertial spin attitude errors */
    double omega_BR_SP[3];              /*!< Sun-pointing rate error */
    double omega_BR_EP[3];              /*!< Earth-pointing rate error */
    double omega_BR_NP[3];              /*!< Nadir-Pointing rate error */
    double omega_BR_NS[3];              /*!< Rate error vector for nadir spin control mode */
    double omega_BR_VP[3];              /*!< Velocity-Pointing rate error */
    double omega_BR_VS[3];              /*!< Velocity-Spin rate error */
    double omega_BR_T[3];               /*!< Thrusting rate error */
    double omega_BR_IP[3];              /*!< Inertial pointing rate errors */
    double omega_BR_IS[3];              /*!< Inertial spin rate errors */
    double omega_BR_RD[3];              /*!< inertial rate damping errors */
    char   deadbandStatus;              /*!< flag indicating the deadband status */
    double v1[3], v2[3];

    ControllerSim();
    ~ControllerSim();

private:
    friend class boost::serialization::access;
    template<typename Archive>
    void serialize(Archive &a, const unsigned int version) {
        a &BOOST_SERIALIZATION_NVP(sigma_BR);
        a &BOOST_SERIALIZATION_NVP(omega_BR);
        a &BOOST_SERIALIZATION_NVP(sigma_BR_SP);
        a &BOOST_SERIALIZATION_NVP(sigma_BR_NP);
        a &BOOST_SERIALIZATION_NVP(sigma_BR_NS);
        a &BOOST_SERIALIZATION_NVP(sigma_BR_VP);
        a &BOOST_SERIALIZATION_NVP(sigma_BR_VS);
        a &BOOST_SERIALIZATION_NVP(sigma_BR_EP);
        a &BOOST_SERIALIZATION_NVP(sigma_BR_IP);
        a &BOOST_SERIALIZATION_NVP(sigma_BR_IS);
        a &BOOST_SERIALIZATION_NVP(omega_BR_SP);
        a &BOOST_SERIALIZATION_NVP(omega_BR_EP);
        a &BOOST_SERIALIZATION_NVP(omega_BR_NP);
        a &BOOST_SERIALIZATION_NVP(omega_BR_NS);
        a &BOOST_SERIALIZATION_NVP(omega_BR_VP);
        a &BOOST_SERIALIZATION_NVP(omega_BR_VS);
        a &BOOST_SERIALIZATION_NVP(omega_BR_T);
        a &BOOST_SERIALIZATION_NVP(omega_BR_IP);
        a &BOOST_SERIALIZATION_NVP(omega_BR_IS);
        a &BOOST_SERIALIZATION_NVP(omega_BR_RD);
        a &BOOST_SERIALIZATION_NVP(deadbandStatus);
    }
};

/* Rate gyro parameters */
class IRUSim
{
public:
    ComponentState_t state;
    double           omega[3];               /*!< rad/s, Simulated IRU rate measurement */
    double           BI[3][3];               /*!< DCM to convert from IRU frame to body frame */
    unsigned char    output[IRU_TELEM_SIZE];
    size_t           outputLength;
    unsigned int     crcError;               /*!< # of CRC errors in a row to trigger fault */
    double           bias[3];                /*!< rad/s, actual iru bias applied, in the gyro frame */
    int              id;                     /*!< iru device identifier */
    unsigned char    resetCounter;           /*!< flag that indicates if the fault logic has commanded a device reset */

    IRUSim();
    ~IRUSim();

private:
    friend class boost::serialization::access;
    template<typename Archive>
    void serialize(Archive &a, const unsigned int version) {
        a &BOOST_SERIALIZATION_NVP(state);
        a &BOOST_SERIALIZATION_NVP(omega);
        a &BOOST_SERIALIZATION_NVP(BI);
        a &boost::serialization::make_array(output, numElems(output));
        a &BOOST_SERIALIZATION_NVP(outputLength);
        a &BOOST_SERIALIZATION_NVP(crcError);
        a &BOOST_SERIALIZATION_NVP(bias);
        a &BOOST_SERIALIZATION_NVP(id);
        a &BOOST_SERIALIZATION_NVP(resetCounter);
    }
};

/* POD parameters */
class PODSim
{
public:
    ComponentState_t state;
    double           gpsPosDataHz;           /*!< Hz */
    double           startUpTime;            /*!< time it takes for POD to get valid R,V solution */
    unsigned char    output[POD_TELEM_SIZE];
    size_t           outputLength;
    unsigned char    resetCounter;           /*!< flag that indicates if the fault logic has commanded a device reset */

    PODSim();
    ~PODSim();

private:
    friend class boost::serialization::access;
    template<typename Archive>
    void serialize(Archive &a, const unsigned int version) {
        a &BOOST_SERIALIZATION_NVP(state);
        a &BOOST_SERIALIZATION_NVP(gpsPosDataHz);
        a &BOOST_SERIALIZATION_NVP(startUpTime);
        a &boost::serialization::make_array(output, numElems(output));
        a &BOOST_SERIALIZATION_NVP(outputLength);
        a &BOOST_SERIALIZATION_NVP(resetCounter);
    }
};

/* Over Temperature/Speed State */
typedef enum {
    TEMP_SPEED_NOMINAL,
    TEMP_SPEED_OVER_RATING,
    MAX_TEMP_SPEED_STATE
} TempSpeedState_t;

/* Star Tracker parameters */
class STSim
{
public:
    ComponentState_t    state;
    double              BS[3][3];               /*!< DCM to convert from ST frame to body-frame */
    double              r_B_head1[3];           /*!< Position vector to origin of head 1 of the ST in body-frame */
    double              r_B_head2[3];           /*!< Position vector to origin of head 2 of the ST in body-frame */
    double              gs_head1[3];            /*!< Pointing vector of head 1 of the ST in body-frame */
    double              gs_head2[3];            /*!< Position vector of head 2 of the ST in body-frame */
    unsigned short int  tempDIPU;               /*!< Internal temperature of the DIPU */
    unsigned short int  tempOptHead;            /*!< Internal temperature of the Optical Head */
    double              power;                  /*!< power usage of RW */
    int                 isGroundModeActive;     /*!< flag to specify if ground test mode is active */
    int                 isIdleModeActive;       /*!< flag to specify if idle mode is active */
    int                 attitudeError;          /*!< flag to specify if an error has occurred and no star field is recognized */
    unsigned short int  attitudeDet;            /*!< Counter for number of good attitude determinations */
    unsigned int        sysTime;                /*!< ST system time since it has been ON */
    unsigned int        lastValidTime;          /*!< Last valid attitude recognition time */
    unsigned char       input[ST_ID_SIZE + ST_CMD_SIZE];
    size_t              inputLength;
    unsigned char       output[ST_ID_SIZE + ST_MAX_TELEM_SIZE];
    size_t              outputLength;
    unsigned int        crcError;               /*!< Induced crcErrors, to cause faults */
    unsigned char       resetCounter;           /*!< flag that indicates if the fault logic has commanded a device reset */

    STSim();
    ~STSim();

private:
    friend class boost::serialization::access;
    template<typename Archive>
    void serialize(Archive &a, const unsigned int version) {
        a &BOOST_SERIALIZATION_NVP(state);
        a &BOOST_SERIALIZATION_NVP(BS);
        a &BOOST_SERIALIZATION_NVP(r_B_head1);
        a &BOOST_SERIALIZATION_NVP(r_B_head2);
        a &BOOST_SERIALIZATION_NVP(gs_head1);
        a &BOOST_SERIALIZATION_NVP(gs_head2);
        a &BOOST_SERIALIZATION_NVP(tempDIPU);
        a &BOOST_SERIALIZATION_NVP(tempOptHead);
        a &BOOST_SERIALIZATION_NVP(power);
        a &BOOST_SERIALIZATION_NVP(isGroundModeActive);
        a &BOOST_SERIALIZATION_NVP(isIdleModeActive);
        a &BOOST_SERIALIZATION_NVP(attitudeError);
        a &BOOST_SERIALIZATION_NVP(attitudeDet);
        a &BOOST_SERIALIZATION_NVP(sysTime);
        a &BOOST_SERIALIZATION_NVP(lastValidTime);
        a &boost::serialization::make_array(input, numElems(input));
        a &BOOST_SERIALIZATION_NVP(inputLength);
        a &boost::serialization::make_array(output, numElems(output));
        a &BOOST_SERIALIZATION_NVP(outputLength);
        a &BOOST_SERIALIZATION_NVP(crcError);
        a &BOOST_SERIALIZATION_NVP(resetCounter);
    }
};

/* Torque rod parameters */
class TRSim
{
public:
    ComponentState_t state;
    
    double           r_B[3];             /*!< Position vector to origin of TR pyramide in body-frame */
    double           u;                  /*!< commanded current */
    double           dipoleAxis[3];      /*!< magnetic torque rod di-pole axis */
    double           a;                  /*!< Current function parameter a */
    double           b;                  /*!< Current function parameter b */
    double           current;            /*!< Current */
    unsigned char    input[TR_CMD_SIZE];
    size_t           inputLength;
    unsigned char    flags;              /*!< One byte flag parameter */
    double           maxVoltage;         /*!< Maximum voltage */

    TRSim();
    ~TRSim();

private:
    friend class boost::serialization::access;
    template<typename Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar &BOOST_SERIALIZATION_NVP(state);
        ar &BOOST_SERIALIZATION_NVP(u);
        ar &BOOST_SERIALIZATION_NVP(dipoleAxis);
        ar &BOOST_SERIALIZATION_NVP(a);
        ar &BOOST_SERIALIZATION_NVP(b);
        ar &BOOST_SERIALIZATION_NVP(current);
        ar &boost::serialization::make_array(input, numElems(input));
        ar &BOOST_SERIALIZATION_NVP(inputLength);
        ar &BOOST_SERIALIZATION_NVP(flags);
        ar &BOOST_SERIALIZATION_NVP(maxVoltage);
    }
};

template<typename Archive>
void serialize(Archive &a, classicElements &oe, const unsigned int version)
{
    a &BOOST_SERIALIZATION_NVP(oe.a);
    a &BOOST_SERIALIZATION_NVP(oe.e);
    a &BOOST_SERIALIZATION_NVP(oe.i);
    a &BOOST_SERIALIZATION_NVP(oe.Omega);
    a &BOOST_SERIALIZATION_NVP(oe.omega);
    a &BOOST_SERIALIZATION_NVP(oe.f);
}

/* Structure for holding a DCM */
class DCM_t
{
public:
    double DCM[3][3];

    DCM_t();
    ~DCM_t();
private:
    friend class boost::serialization::access;
    template<typename Archive>
    void serialize(Archive &a, const unsigned int version) {
        a &BOOST_SERIALIZATION_NVP(DCM);
    }
};


class SpacecraftSim
{
public:
    double               time;
    long                 timeStamp;                      /*!< number of runs to perform */
    unsigned int         rerunCaseNum;                   /*!< if non-zero will attempt to rerun requested mcrun from previous MC */
    double               maxSimTime;                     /*!< sec */
    
    CelestialObject_t    celestialObject;                /*!< celestial object about which the simulation is setup */
    double               mu;                             /*!< celestial object gravity constant mu */
    double               req;                            /*!< celestial object equatorial radius (km) */
    double               polarRate;                      /*!< celestial object polar rotation rate (rad/s) */
    
    
    char                 useRealTimeSync;                /*!< flag for syncing simulation time to real time */
    double               realTimeSpeedUpFactor;          /*!< indicates how much faster than real-time the simulation should run.
                                                          Does not enforce strick real-time syncing unless useRealTimeSync is
                                                          turned on.  A negative value lets the simulation run as quickly as
                                                          it can.  */
    
    char                 useOpenGLVisualization;         /*!< if 1, stream the simulation data to the OpenGL visualization program */
    double               openglLastSync;                 /*!< time last OpenGL communication string was sent */
    
    char                 useGravityGradientTorque;       /*!< if 1, add the gravity gradient torque vector */
    char                 useReactionWheelJitter;
    char                 useGravityPerturbModel;         /*!< if 1, Add J2 through J6 gravity accelerations to EOMs */
    char                 useAtmosDragModel;              /*!< if 1, Add atmospheric drag accelerations to EOMs */
    char                 useSrpModel;                    /*!< if 1, Add solar radiation pressure accelerations to EOMs */
    AlbedoModel_t        albedoModel;                    /*!< flag for determining which albedo model to use as truth */
    EarthMagFieldModel_t earthMagFieldModel;             /*!< flag for determining which Earth magnetic field model to use as truth */
    
    CommandedState_t     ctrlState;                      /*!< defaulted to CMD_ON, turns off control if CMD_OFF */
    char                 ctrlUsingTrueSunDirection;
    char                 ctrlUsingTrueAttitude;
    char                 ctrlUsingTrueRates;
    
    /* DV Thrusting parameters */
    double               DVThrusterManeuverStartTime;    /*!< time to engage the primary DV thrusters */
    double               DVThrusterManeuverDuration;     /*!< time to disengage the primary DV thrusters */
    
    /* parameters for calculating power */
    double rwPowerMin; /*!< Watts just to be turn on, maintain zero speed */
    double rwPowerMax; /*!< Watts to maintain max speed */
    double trPowerMax;
    
    InitialConditions ics;
    SpiceTime spiceTime;
    
    /*-----------------------------------------------------------------------*/
    /* Spacecraft parameters */
    ADCSState_t adcsState;
    //    ControlMethod_t controlMethod;
    //    CDHState_t  cdhState;
    //    EPSState_t  epsState;
    
    /* Sub components */
    //    ControllerSim   control;
    std::vector<Thruster> acsThrusters;  /*!< Array of attitude control thruster objects */
    std::vector<Thruster> dvThrusters;  /*!< Array of attitude control thruster objects */
    CoarseSunSensor css[NUM_CSS];
//    TAM             tam;
    IRUSim          iru;
    PODSim          pod;
    std::vector<RWSim> reactionWheels;
    STSim           st;
    TRSim           tr[NUM_TR];
    
    /* Physical properties of the spacecraft */
    double I[3][3];    /*!< spacecraft inertia tensor */
    double Iinv[3][3]; /*!< spacecraft inertia tensor m33Inverse */
    
    double A_drag;  /*!< spacecraft area subject to drag */
    double A_srp;   /*!< spacecraft area subject to solar radiation pressure */
    double mass;    /*!< spacecraft mass */
    double Cd;      /*!< spacecraft drag coefficient */
    double cHatSolarpanelSim_B[3];  /*!< solar cell normal vector in body frame components */
    double cHatEarthAntennaSim_B[3];/*!< earch communication antenna heading vector */
    
    
    /* Current state of the spacecraft */
    classicElements oe;    /*!< classic orbital element set */
    double          gamma; /*!< current sidereal time of the Planet */
    double          n;     /*!< mean orbit rate */
    double          T0; /*epoch of perigee anomaly*/
    
    double r_N[3]; /*!< orbital position vector in inertial frame */
    double v_N[3]; /*!< orbital velocity vector in inertial frame */
    
    double sigma[3]; /*!< MRP attitude set */
    double omega[3]; /*!< rad/sec, spacecraft body rates */
    
    double B_N[3]; /*!< magnetic field vector in inertial frame components */
    double B_B[3]; /*!< magnetic field vector in body frame components */
    
    double r_CS[3];     /*!< unit vector from celestial object to Sun in celestial object inertial frame */
    double sHatN[3];    /*!< sun direction vector in inertial frame components */
    double sHatB[3];    /*!< sun direction vector in body frame components */
    double sunAngle;    /*!< sun pointing angle */
    
    double earthHeadingSim_N[3]; /*!< unit direction vector pointing from spacecraft back to Earth in inertial frame components */
    double earthHeadingSim_B[3]; /*!< unit direction vector pointing from spacecraft back to Earth in body frame components */
    
public:
    SpacecraftSim();
    ~SpacecraftSim();
    
//    static void save(const SpacecraftSim &scSim, const char *filename);
//    static void load(SpacecraftSim &scSim, const char *filename);
//    void initializeSpacecraftSimInputs(Spacecraft_t *sc);
//    void initializeSpacecraftSimDefaults(Spacecraft_t *sc);
//    void setAllThrustValvesClosed();
//    
//    void integrate(double t, double dt);
//    
//    void computeTorqueRodTorque(TRSim *tr, double B_B[3], double Tm[3]);
//    
//    void computeAttitudeErrors(Spacecraft_t *sc);
//    void computeMagneticField(int init, double t);
//    void computeDipoleFromCurrent(int i, double current);
//    void computeSunVector(double t);
//    void computeEarthVector(double t, Spacecraft_t *sc);
//    
//    void simulateRateGyro();
//    void simulateCoarseSunSensors(double t);
//    void simulateStarTracker(double *quatPerturb);
//    void simulatePOD(double t);
//    void mtbReply(int i);
    
private:    
    friend class boost::serialization::access;
    
    template<typename Archive>
    void serialize(Archive &a, const unsigned int version) {
        a &BOOST_SERIALIZATION_NVP(time);
        a &BOOST_SERIALIZATION_NVP(spiceTime);
        a &BOOST_SERIALIZATION_NVP(timeStamp);
        a &BOOST_SERIALIZATION_NVP(rerunCaseNum);
        a &BOOST_SERIALIZATION_NVP(maxSimTime);
        a &BOOST_SERIALIZATION_NVP(celestialObject);
        a &BOOST_SERIALIZATION_NVP(mu);
        a &BOOST_SERIALIZATION_NVP(req);
        a &BOOST_SERIALIZATION_NVP(polarRate);
        
        a &BOOST_SERIALIZATION_NVP(useRealTimeSync);
        a &BOOST_SERIALIZATION_NVP(realTimeSpeedUpFactor);
        
        a &BOOST_SERIALIZATION_NVP(useOpenGLVisualization);
        a &BOOST_SERIALIZATION_NVP(openglLastSync);
        
        a &BOOST_SERIALIZATION_NVP(useGravityGradientTorque);
        a &BOOST_SERIALIZATION_NVP(useReactionWheelJitter);
        a &BOOST_SERIALIZATION_NVP(useGravityPerturbModel);
        a &BOOST_SERIALIZATION_NVP(useAtmosDragModel);
        a &BOOST_SERIALIZATION_NVP(useSrpModel);
        a &BOOST_SERIALIZATION_NVP(albedoModel);
        
        a &BOOST_SERIALIZATION_NVP(ctrlState);
        a &BOOST_SERIALIZATION_NVP(ctrlUsingTrueSunDirection);
        a &BOOST_SERIALIZATION_NVP(ctrlUsingTrueAttitude);
        a &BOOST_SERIALIZATION_NVP(ctrlUsingTrueRates);
        
        a &BOOST_SERIALIZATION_NVP(DVThrusterManeuverStartTime);
        a &BOOST_SERIALIZATION_NVP(DVThrusterManeuverDuration);
        
        a &BOOST_SERIALIZATION_NVP(rwPowerMin);
        a &BOOST_SERIALIZATION_NVP(rwPowerMax);
        a &BOOST_SERIALIZATION_NVP(trPowerMax);
        
        a &BOOST_SERIALIZATION_NVP(ics);
        
        a &BOOST_SERIALIZATION_NVP(adcsState);
        
        a &BOOST_SERIALIZATION_NVP(acsThrusters);
        a &BOOST_SERIALIZATION_NVP(dvThrusters);
        a &BOOST_SERIALIZATION_NVP(css);
        a &BOOST_SERIALIZATION_NVP(iru);
        a &BOOST_SERIALIZATION_NVP(pod);
        a &BOOST_SERIALIZATION_NVP(reactionWheels);
        a &BOOST_SERIALIZATION_NVP(st);
        a &BOOST_SERIALIZATION_NVP(tr);
        
        a &BOOST_SERIALIZATION_NVP(I);
        a &BOOST_SERIALIZATION_NVP(Iinv);
        
        a &BOOST_SERIALIZATION_NVP(A_drag);
        a &BOOST_SERIALIZATION_NVP(A_srp);
        a &BOOST_SERIALIZATION_NVP(mass);
        a &BOOST_SERIALIZATION_NVP(Cd);
        a &BOOST_SERIALIZATION_NVP(cHatSolarpanelSim_B);
        a &BOOST_SERIALIZATION_NVP(cHatEarthAntennaSim_B);
        
        a &BOOST_SERIALIZATION_NVP(oe);
        a &BOOST_SERIALIZATION_NVP(gamma);
        a &BOOST_SERIALIZATION_NVP(n);
        
        a &BOOST_SERIALIZATION_NVP(r_N);
        a &BOOST_SERIALIZATION_NVP(v_N);
        
        a &BOOST_SERIALIZATION_NVP(sigma);
        a &BOOST_SERIALIZATION_NVP(omega);
        
        a &BOOST_SERIALIZATION_NVP(B_N);
        a &BOOST_SERIALIZATION_NVP(B_B);
        
        a &BOOST_SERIALIZATION_NVP(r_CS);
        a &BOOST_SERIALIZATION_NVP(sHatN);
        a &BOOST_SERIALIZATION_NVP(sHatB);
        a &BOOST_SERIALIZATION_NVP(sunAngle);
        
        a &BOOST_SERIALIZATION_NVP(earthHeadingSim_N);
        a &BOOST_SERIALIZATION_NVP(earthHeadingSim_B);        
    }
};

#endif
