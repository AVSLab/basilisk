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
* spacecraftDefinitions.h
*
*/

#ifndef _SPACECRAFT_DEFINITIONS_H_
#define _SPACECRAFT_DEFINITIONS_H_

#include <string.h>
#include "utilities/orbitalMotion.h"

/* TODO: The following definitions need to be removed before flight */

/* Set the following CSS_CONFIG to values of either 0 (CSS_CUBE6_CONFIG), 1 (CSS_CICERO_CONFIG) or 2 CSS_EMM_CONFIG */
#define CSS_CONFIG              2

/* Do not manually set the below CSS_*_CONFIG preprocessor macros. Use the above CSS_CONFIG macro */
#if CSS_CONFIG == 0
#define CSS_CUBE6_CONFIG        1
#define CSS_CICERO_CONFIG       0
#define CSS_EMM_CONFIG          0
#elif CSS_CONFIG == 1
#define CSS_CUBE6_CONFIG        0
#define CSS_CICERO_CONFIG       1
#define CSS_EMM_CONFIG          0
#else
#define CSS_CUBE6_CONFIG        0
#define CSS_CICERO_CONFIG       0
#define CSS_EMM_CONFIG          1
#endif

#define CSS_USE_DOUBLES         1           /* Uses continuous doubles for measurements instead of scaling to uints and truncating */
#define IRU_USE_DOUBLES         1           /* Uses continuous doubles for measurements instead of truncating to floats */
#define POD_USE_DOUBLES         1           /* Uses continuous doubles for POD measurements instead of truncating to floats */
#define EST_INT_RK4             1           /* Use RK4 integration (vs Euler integration) in estimators */
#define PRINT_COMP_STATES       0           /* Flag to determine if component states should be printed to the output file*/
#define CALC_CSSEST_TIMING      0           /* Flag to determine if estimation and propagation times should be averaged and displayed for sun-direction filter */

/* TODO: These values should become parameters/look-up table values */
#define NUM_RW                  4           /* number of reaction wheels */
#define NUM_TR                  3           /* number of magnetic torque rods */
#define NUM_ACS_THRUSTER        8           /* number of attitude control thrusters */
#define NUM_DV_THRUSTER         6           /* number of delta V thrusters */

#if CSS_CUBE6_CONFIG
#define NUM_CSS                 6
#else
#define NUM_CSS                 8           /* number of coarse sun sensors */
#endif
/* The following define telemetry/command packet sizes in bytes */
#if CSS_USE_DOUBLES
#define CSS_TELEM_SIZE          8
#else
#define CSS_TELEM_SIZE          2
#endif
#define TH_CMD_CODE_SIZE        1
#define TH_CMD_SIZE             4
#define TH_TELEM_SIZE           32
#define TAM_CMD_SIZE            1
#define TAM_TELEM_SIZE          12
#if IRU_USE_DOUBLES
#define IRU_TELEM_SIZE          30
#else
#define IRU_TELEM_SIZE          18
#endif
#define POD_TELEM_SIZE          290
#if POD_USE_DOUBLES
#define POD_ADCS_TELEM_SIZE     68
#else
#define POD_ADCS_TELEM_SIZE     44
#endif
#define RW_CMD_CODE_SIZE        1
#define RW_CMD_SIZE             4
#define RW_TELEM_SIZE           32
#define ST_ID_SIZE              1
#define ST_CMD_SIZE             9
#define ST_TELEM_SIZE           42
#define ST_MAX_TELEM_SIZE       256         /* Max = raw telemtry return, 256 byte max data ("Read Star Data") */
#define TR_CMD_SIZE             4

#define NUM_CSSEST_STATES       3               /* Number of states used by CSS estimator */
#define NUM_CSSEST_ETA          6               /* Number of process noise parameters used by CSS estimator */
#define NUM_CSSEST_B_V          4 * NUM_CSS     /* Number of measurement bias parameters used by CSS estimator */
#define NUM_CSSEST_B_N          6               /* Number of dynamic bias parameters used by CSS estimator */

#define NUM_CSSCAL_STATES       6 + 3 * NUM_CSS /* Number of states used by CSS calibration filter */
#define NUM_CSSCAL_ETA          6 + NUM_CSS     /* Number of process noise parameters used by CSS calibration filter */
#define NUM_CSSCAL1_B_V         6               /* Number of measurement bias parameters used by CSS calibration filter ECKF1 */
#define NUM_CSSCAL2_B_V         6 + NUM_CSS     /* Number of measurement bias parameters used by CSS calibration filter ECKF2 */
#define NUM_CSSCAL_NISN         100             /* Number of NIS values to use for running average */

#define numElems(x) (sizeof(x) / sizeof(x[0]))

/* Albedo models */
typedef enum {
    ALBEDO_NONE,
    ALBEDO_TOMS_1p25x1,
    ALBEDO_TOMS_5x5,
    ALBEDO_TOMS_10x10,
    MAX_ALBEDO_MODEL
} AlbedoModel_t;

/* Earth magnetic field models */
typedef enum {
    MAGFIELD_WMM,
    MAGFIELD_TILTDIPOLE,
    MAX_MAGFIELD_MODEL
} EarthMagFieldModel_t;

/* ADCS Control Mode */
typedef enum {
    ADCS_SUN_POINTING,
    ADCS_HILL_POINTING,
    ADCS_HILL_SPIN,
    ADCS_VELOCITY_POINTING,
    ADCS_VELOCITY_SPIN,
    ADCS_THRUSTING,
    ADCS_EARTH_POINTING,
    ADCS_INERTIAL3D_POINTING,
    ADCS_INERTIAL3D_SPIN,
    ADCS_RATE_DAMPING,
    MAX_ADCS_STATE
} ADCSState_t;

/* ADCS Control Method */
typedef enum {
    CONTROL_MRP_FEEDBACK,
    CONTROL_MRP_STEERING,
    CONTROL_PRV_STEERING,
    MAX_CONTROL_METHOD
} ControlMethod_t;

/* EPS State */
typedef enum {
    EPS_NOMINAL,
    EPS_MARGINAL,
    EPS_CRITICAL,
    MAX_EPS_STATE
} EPSState_t;

/* Command and Data Handling State */
typedef enum {
    CDH_NOMINAL,
    CDH_SAFE,
    MAX_CDH_STATE
} CDHState_t;

/* Commanded Component State */
typedef enum {
    CMD_OFF,
    CMD_ON,
    MAX_CMD_STATE
} CommandedState_t;

/* Deadband states */
typedef enum {
    DEADBAND_OFF,
    DEADBAND_ON,
    DEADBAND_NO_SIGNAL,
    MAX_DEADBAND
} DeadbandStates_t;

/* Component state */
typedef enum {
    COMPONENT_OFF,
    COMPONENT_START,
    COMPONENT_ON,
    COMPONENT_FAULT,
    COMPONENT_TEST,
    MAX_COMPONENT_STATE
} ComponentState_t;

/* Nadir pointing states */
typedef struct OrbitFrameStates_t {
    unsigned int        i_r;                        /* body axis index that lines up with i_r */
    unsigned int        i_theta;                    /* body axis index that lines up with i_theta */
    unsigned int        i_h;                        /* body axis index that lines up with i_h */
    int                 i_rSign;                    /* sign of the i_r axis alignment */
    int                 i_thetaSign;                /* sign of the i_theta axis alignment */
    int                 i_hSign;                    /* sign of the i_h axis alignment */
    int                 o_spin;                     /* orbit frame axis about which to spin */
    unsigned int        b_spin;                     /* principal body frame axis about which to spin */
    double              omega_spin;                 /* desired spin rate */
} OrbitFrameStates_t;

/* Controller structure */
typedef struct Controller_t {
    double              tPrev;
    double              minModeSwitchTime;          /* sec, minimum switching time between control mode changes */
    double              minTumbleRate;                  /* minimum attitude rate where external forces are used to detumble */
    double              maxTRDipoleMoment;          /* Am^2, maximum torque rod dipole moment */
    double              integralLimit;              /* integration limit to avoid integral wind-up */
    double              sunAngleDeadbandInner;      /* rad, coarse sun pointing deadband (inner) */
    double              sunAngleDeadbandOuter;      /* rad, coarse sun pointing deadband (outer) */
    double              sunRateDeadbandInner;       /*!< rad/s, rate regulation deadband (inner) for sun pointing mode */
    double              sunRateDeadbandOuter;       /*!< rad/s, rate regulation deadband (outer) for sun pointing mode */
    double              earthAngleDeadbandInner;    /*!< rad, earth pointing inner deadband */
    double              earthAngleDeadbandOuter;    /*!< rad, earth pointing outer deadband */
    char                deadbandStatus;             /*!< flag, indicating if the deadband is on*/
    double              earthRateDeadbandInner;     /*!< rad/s, earth pointing inner deadband */
    double              earthRateDeadbandOuter;     /*!< rad/s, earth pointing outer deadband */
    double              earthSunDeadband;           /*!< rad, deadband angle between earth and sun to consider solar panel
                                                         pointing in Earth Pointing mode */
    double              K;                          /* attitude feedback gain */
    double              K1;                         /* attitude steering law gain */
    double              K3;                         /* attitude steering law gain */
    double              omega_max;                  /* attitude steering law gain */
    double              P;                          /* attitude rate feedback gain */
    double              K_nom;                      /* nominal attitude feedback gain */
    double              P_nom;                      /* nominal attitude rate feedback gain */
    double              K_thrust;                   /* thrust attitude feedback gain */
    double              P_thrust;                   /* thrust attitude rate feedback gain */
    double              Ki;                         /* integral feedback gain */
    double              C;                          /* momentum dumping feedback gain */
    double              z[3];                       /* integral feedback term */
    int                 isMomentumDumpingWithTB;    /* flag to trigger momentum dumping with the torque bars */
    int                 isMomentumBalancing;        /* flag to trigger RW momentum with the RW nullspace */
    unsigned char       B_degaussCounter;           /* counter for determining when to degauss tr prior to taking magnetic field measurement */
    double              sigma_BR[3];                /* MRP attitude tracking error */
    double              omega_BR[3];                /* attitude rate tracking error */
    double              int_sigma[3];               /* integral of k*sigma */
    double              domega0[3];                 /* initial body rate error */
    double              phi_spin;                   /* spin angle in the orbit frame aligned spin mode */
    int                 isUsingThrusters;           /* flag indicating if Attiude control is done with thrusters, rather than RWs */
    int                 considerSunHeadinginEarthPointing;  /*!< flag indicating if the sun direciton should be considered in doing
                                                             HGA Earth communication */
    double              RNinertialPointing[3][3];   /* DCM from inertial to static reference orientation for ADCS_INERTIAL3D_POINTING,
                                                       this is the initial inertial attitude for the ADCS_INERTIAL3D_SPIN mode */
    double              sigma_RN[3];                /* time varying MRP from inertial to reference orientationfor ADCS_INERTIAL3D_SPIN */
    double              omega_rN[3];                /* reference frame angular rate vector in inertial frame components for ADCS_INERTIAL3D_SPIN mode */
    double              Lr[3];                      /* required control torque of the attiude control law implemented */
} Controller_t;

/* Thruster commands */
typedef enum {
    TH_GET_TELEM,
    TH_SET_THRUST,
    TH_SET_THRUST_GET_TELEM,
    TH_MAX_CMD
} THCmd_t;

/* Thrust structure */
typedef struct Thrust_t {
    CommandedState_t    cmdState;                   /* This is flag to FSW to close valve if ADCS detects an issue */
    unsigned char       telemetry[TH_CMD_CODE_SIZE + TH_TELEM_SIZE];
    size_t              telemLength;
    unsigned char       command[TH_CMD_CODE_SIZE + TH_CMD_SIZE];
    size_t              commandLength;
    double              timeStart;                  /* Time to begin thrust */
    double              timeDuration;               /* Time to fire thruster as an integer multiple of min pulse duty cycle */
    double              pulseRemainder;             /* Unimplemented thrust */
    double              preThrustWindow;            /* Time to harden gains before thrust begins */
    unsigned char       flags;                      /* Flags for thrust mode */
    double              r_B[3];                     /* Position vector in body frame */
    double              gt_B[3];                    /* Thrust direction vector in body frame */
    double              tau_B[3];                   /* Body axis about which thruster torque acts */
    double              maxThrust;                  /* Maximum thrust capability */
    int                 pairId;                     /* Thruster pair id */
    double              pulseTime;                  /*!< Desired high cycle time of pulse */
    double              pulseTimeResolution;        /*!< Pulse increment */
    double              pulseTimeMin;               /*!< Minimum pulse command */
    int                 pulseTickCount;             /*!< Counts number of pulse iterations */
    int                 pulseTickMax;               /*!< Number of pulse iterations for thrust cycle */
    int                 newThrustCommand;        /*!< Indicates new thruster command to be executed */
    CommandedState_t    thrustState;                /*!< Thruster ON/OFF state */
    double              thrustForce;                /*!< thruster force applied */
    double              level;                      /*!< thruster duty cycle percentage */
    int                 numPulses;                  /*!< number of discrete pulses implemented */
} Thrust_t;

/* Coarse sun sensor structure */
typedef struct CSS_t {
    ComponentState_t    state;
    unsigned char       telemetry[CSS_TELEM_SIZE];
    size_t              telemLength;
    double              theta;                      /* rad, css azimuth angle, measured positive from the body +x axis around the +z axis */
    double              phi;                        /* rad, css elevation angle, measured positive toward the body +z axis from the x-y plane */
    double              PB[3][3];                   /* Body to platform DCM */
    double              horizonPlane[3];            /* unit direction vector defining horizon cut off plane of CSS */
    double              sensedValue;                /* sensor value that has been optionally filtered */
    double              rawSensedValue;             /* sensor reported value */
    unsigned char       flags;                      /* One byte flag parameter. Bit 0 => first read = 0, all other reads = 1. */
    unsigned char       badReadTelem;               /* Number of bad telemetry strings received. Will increment/decrement for every good/bad read obtained. Min value = 0 */
    unsigned char       maxBadReadTelem;
    double              fov;                        /* rad, CSS field of view (half-angle) */
    double              maxVoltage;                 /* max voltage measurable by CSS, used in discretization */
    double              minDiodeValue;              /* threshold value below which it is assumed CSS is not getting input */
} CSS_t;

/* Three axis magnetometer structure */
typedef struct TAM_t {
    ComponentState_t    state;
    CommandedState_t    cmdState;
    signed short int    temperature;                /* Temperature from TAM readings */
    unsigned char       telemetry[TAM_TELEM_SIZE];
    size_t              telemLength;
    unsigned char       command[TAM_CMD_SIZE];
    size_t              commandLength;
    double              B[3];                       /* Measured magnetic field in the 3 axes */
    double              coefficients[2];            /* Coefficients for count->nT computation */
    short               counts[3];                  /* Counts in x, y, z directions given by TAM */
    double              BT[3][3];                   /* DCM to rotate from TAM frame into body frame */
    unsigned char       flags;                      /* One byte flag parameter. Bit 0 => first read = 0, all other reads = 1. */
    unsigned char       resetCounter;               /* reset counter, nominally 0, >=1 if a reset has occurred */
    unsigned char       badReadTelem;               /* Number of bad telemetry strings received. Will increment/decrement for every good/bad read obtained. Min value = 0 */
    /* The following can be separated to separate structure if more than one TAM is used */
    unsigned char       maxReset;                   /* Maximum number of resets allowed for a given device before it is permanently turned off */
    unsigned char       maxBadReadTelem;
    double              minTemp;                    /* minimum temp expected on TAM before declaring error */
    double              maxTemp;                    /* maximum temp expected on TAM before declaring error */
} TAM_t;

/* Inertial reference unit structure */
typedef struct IRU_t {
    ComponentState_t    state;
    CommandedState_t    cmdState;
    unsigned char       telemetry[IRU_TELEM_SIZE];
    size_t              telemLength;
    double              omega[3];                   /* rad/s, body rates as measured by rate gyro (in gyro frame) that have optionally been filtered */
    double              rawOmega[3];                /* IRU frame rates directly from rate gyro (in gyro frame) */
    double              BI[3][3];                   /* DCM to convert from IRU frame to body frame */
    unsigned char       flags;                      /* One byte flag parameter. Bit 0 => first read = 0, all other reads = 1. */
    unsigned char       resetCounter;               /* reset counter, nominally 0, >=1 if a reset has occurred */
    unsigned char       badReadTelem;               /* Number of bad telemetry strings received. Will increment/decrement for every good/bad read obtained. Min value = 0 */
    /* The following can be separated to separate structure if more than one IRU is used */
    unsigned char       maxReset;                   /* Maximum number of resets allowed for a given device before it is permanently turned off */
    unsigned char       maxBadReadTelem;
    int                 id;                         /* iru device identifier. TODO: Do we get this with IRU or do we set it? */
    double              maxRate;                    /* body rate beyond which an error is thrown */
} IRU_t;

/* Precise orbit determination unit structure */
typedef struct POD_t {
    ComponentState_t    state;
    CommandedState_t    cmdState;
    unsigned char       telemetry[POD_TELEM_SIZE];
    size_t              telemLength;
    double              R[3];                       /* Radius Vector */
    double              V[3];                       /* Velocity Vector */
    double              t;                          /* time at which measurement was taken */
    double              gpsTime[2];                 /* current GPS time: GPS week, second of week */
    unsigned char       flags;                      /* One byte flag parameter. Bit 0 => first read = 0, all other reads = 1. */
    unsigned char       resetCounter;               /* reset counter, nominally 0, >=1 if a reset has occurred */
    unsigned char       badReadTelem;               /* Number of bad telemetry strings received. Will increment/decrement for every good/bad read obtained. Min value = 0 */
    /* The following can be separated to separate structure if more than one IRU is used */
    unsigned char       maxReset;                   /* Maximum number of resets allowed for a given device before it is permanently turned off */
    unsigned char       maxBadReadTelem;
    double              maxChiSquared;
    double              minRadius;                  /* km, minimum expected radius */
    double              maxRadius;                  /* km, maximum expected radius */
    double              minOrbitalVelocity;         /* km/s, minimum expected orbital velocity */
    double              maxOrbitalVelocity;         /* km/s, maximum expected orbital velocity */
} POD_t;

/* Reaction wheel commands */
/* TODO: These will need to be updated before flight to include all possible commands */
typedef enum {
    RW_GET_TELEM,
    RW_SET_TORQUE,
    RW_SET_MAX_TORQUE,
    RW_SET_TORQUE_GET_TELEM,
    RW_MAX_CMD
} RWCmd_t;

/* Reaction wheel structure */
typedef struct RW_t {
    ComponentState_t    state;
    CommandedState_t    cmdState;
    unsigned char       telemetry[RW_CMD_CODE_SIZE + RW_TELEM_SIZE];
    size_t              telemLength;
    unsigned char       command[RW_CMD_CODE_SIZE + RW_CMD_SIZE];
    size_t              commandLength;
    unsigned char       groundCmd[RW_CMD_CODE_SIZE + RW_CMD_SIZE];
    size_t              groundCmdLength;
    unsigned char       groundTelem[RW_CMD_CODE_SIZE + RW_CMD_SIZE + RW_TELEM_SIZE];
    size_t              groundTelemLength;
    double              u;                          /* commanded torque */
    double              sign;                       /* sign (+ or -) of command */
    double              maxU;                       /* Max torque allowed */
    double              Omega;                      /* spin rate relative to body (rad/sec) */
    double              OmegaBias;                  /* spin bias (rad/sec) */
    double              Js;                         /* spin inertia (kgm^s) */
    double              gs[3];                      /* spin axis in body frame */
    unsigned char       flags;                      /* One byte flag parameter. Bit 0 => first read = 0, all other reads = 1. Bit 1,2,3 => MAX, Half, Zero torque modes */
    unsigned char       resetCounter;               /* reset counter, nominally 0, >=1 if a reset has occurred */
    unsigned char       badReadTelem;               /* Number of bad telemetry strings received. Will increment/decrement for every good/bad read obtained. Min value = 0 */
    unsigned char       badReadCmd;                 /* Number of bad command strings received. Will increment/decrement for every good/bad read obtained. Min value = 0 */
    
    unsigned char       maxReset;                   /* Maximum number of resets allowed for a given device before it is permanently turned off */
    unsigned char       maxBadReadTelem;
    unsigned char       maxBadReadCmd;
    double              maxSpeed;                   /* rad/s, maximum possible speed */
    double              highSpeed;                  /* rad/s, max speed before speed limiting occurs */
    double              epsilon;                    /* error possibly present in float round-off */
    double              maxTemp;                    /* C, max temp before temperature limiting occurs */
    double              maxTorque;                  /* Nm, max torque accepted */
    double              nominalTorque;              /* Nm, nominal start up torque */
    double              minTorque;                  /* Nm, minimum "set max torque" command value allowed */
    
} RW_t;

/* Star tracker mode */
typedef enum {
    ST_IDLE,
    ST_FIRST_AQUISITION,
    ST_TRACKING,
    ST_GROUND_OPS,
    MAX_STAR_TRACKER_MODE
} STMode_t;

/* Star tracker commands */
/* TODO: These will need to be updated before flight to include all possible commands */
typedef enum {
    ST_GET_ATTITUDE,
    ST_SET_IDLE_MODE_ON,
    ST_SET_IDLE_MODE_OFF,
    ST_SET_GROUND_TEST_ON,
    ST_SET_GROUND_TEST_OFF,
    ST_MAX_CMD
} STCmd_t;

/* Star tracker structure */
typedef struct ST_t {
    ComponentState_t    state;
    CommandedState_t    cmdState;
    unsigned char       telemetry[ST_ID_SIZE + ST_TELEM_SIZE];
    size_t              telemLength;
    unsigned char       command[ST_ID_SIZE + ST_CMD_SIZE];
    size_t              commandLength;
    unsigned char       groundCmd[ST_ID_SIZE + ST_CMD_SIZE];
    size_t              groundCmdLength;
    unsigned char       groundTelem[ST_ID_SIZE + ST_MAX_TELEM_SIZE];
    size_t              groundTelemLength;
    unsigned int        time;                       /* Internal system time just before starting data transmission. Started w/ ST power ON. Multiple of system time period 1/28800 sec */
    STMode_t            mode;                       /* internal mode of star tracker */
    double              BS[3][3];                   /* DCM to convert from ST frame to body-frame */
    //double              r_B[3];
    double              qSNraw[4];                  /* Quaternion describing rotation from inertial to ST frame */
    double              qSN[4];                     /* Quaternion describing rotation from inertial to ST frame that has been optionally filtered */
    double              qBN[4];                     /* Quaternion describing rotation from inertial to body frame that has been optionally filtered */
    unsigned char       flags;                      /* One byte flag parameter. Bit 0 => first read = 0, all other reads = 1. */
    unsigned char       resetCounter;               /* reset counter, nominally 0, >=1 if a reset has occurred */
    unsigned char       badReadTelem;               /* Number of bad telemetry strings received. Will increment/decrement for every good/bad read obtained. Min value = 0 */
    unsigned char       badReadCmd;                 /* Number of bad command strings received. Will increment/decrement for every good/bad read obtained. Min value = 0 */
    /* The following can be separated to separate structure if more than one ST is used */
    unsigned char       maxReset;                   /* Maximum number of resets allowed for a given device before it is permanently turned off */
    unsigned char       maxBadReadTelem;
    double              epsilon;                    /* error possibly present in float round-off */
    unsigned char       id;                         /* st device identifier. TODO: Do we get this with ST or do we set it? */
} ST_t;

/* Torque rod structure */
typedef struct TR_t {
    ComponentState_t    state;
    double              u;                          /* commanded current */
    unsigned char       command[TR_CMD_SIZE];       /* bytes sent to EPS for MTB command */
    size_t              commandLength;              /* size of command given to CDH, should be 4 */
    double              dipoleAxis[3];              /* magnetic torque rod di-pole axis */
    double              a;                          /* Current function parameter a */
    double              b;                          /* Current function parameter b */
    double              current;                    /* Current */
    char                hexCurrent[2];
    double              sign;                       /* Sign (+ or -) of command */
    unsigned char       flags;                      /* flag0 = isDegaussing, flag1 = isSafeToRead */
    double              degaussTime;
    char                degaussStarted;
    double              maxVoltage;
} TR_t;

/* Flag for determining which mode of the estimator we want to run */
typedef enum {
    EST_PROPAGATE,
    EST_UPDATE,
    MAX_EST_MODE
} EstMode_t;

/* State estimator structure */
typedef struct StateEstimator_t {
    CommandedState_t    cmdState;           /* Commanded state of estimator: it is either ON or OFF */
    char                isEstimating;       /* Determines if estimator is ON and has enough information to estimate */
    char                useOmegaDot;        /* Flag for determining if omegaDot is held zero during integration */
    double              tPrev;              /* sec, last time state estimator was called */
    double              omegaPrev[3];       /* rad/sec, last recorded angular velocity in gyro frame, use to integrate forward to current time */
    double              sigma[3];           /* MRP of spacecraft wrt inertial */
    double              omegaBias[3];       /* rad/s, estimated rate gyro bias, in gyro frame */
    double              P[6][6];            /* state estimator covariance matrix */
    double              Q[6][6];            /* process noise covariance matrix */
    double              R[3][3];            /* measurement noise covariance matrix */
    int                 init;               /* flag for determining if estimator needs to be initialized */
} StateEstimator_t;

/* Orbit estimator structure, used with POD */
typedef struct OrbitPropagator_t {
    CommandedState_t    cmdState;           /* Commanded state of estimator: it is either ON or OFF */
    char                isEstimating;       /* Determines if estimator is ON and has enough information to estimate */
    double              rEpoch[3];          /* km, position at last epoch */
    double              vEpoch[3];          /* km/s, velocity at last epoch */
    double              tEpoch;             /* sec, time of last epoch */
} OrbitPropagator_t;

/* CSS estimation modes */
typedef enum {
    CSSEST_LSMN,                            /* least squares minimum norm method */
    CSSEST_WLSMN,                           /* weighted least squares minimum norm method */
    CSSEST_EKF_1,                           /* extended kalman filter method (processes only measurements that align with current sun direction estimate) */
    MAX_CSSEST_MODE
} CSSEstMode_t;

/* CSS estimator structure */
typedef struct CSSEstimator_t {
    CommandedState_t    cmdState;           /* Commanded state of estimator: it is either ON or OFF */
    char                isEstimating;       /* Determines if estimator is ON and has enough information to estimate */
    unsigned char       init;               /* flag for determining if estimator needs to be initialized */
    CSSEstMode_t        mode;               /* flag for determining which estimator to use */
    double              minObsInSun;        /* minimum CSS measurement required to assume in view of Sun */
    char                shouldDelayCtrl;    /* flag to delay control damping rates until at least one sensor sees the Sun */
    char                delayCtrl;          /* flag to delay control damping rates until at least one sensor sees the Sun */
    char                useOmegaDot;        /* Flag for determining if omegaDot is held zero during integration */
    double              CBias[NUM_CSS];     /* W, calibration coefficient bias for each CSS */
    double              thetaBias[NUM_CSS]; /* rad, CSS misalignment azimuth bias, measured positive from the body +x axis around the +z axis */
    double              phiBias[NUM_CSS];   /* rad, CSS misalignment elevation bias, measured positve toward the body +z axis from the x-y plane */
    double              tPrev;              /* sec, previous time CSS estimator called */
    double              sunAngle;           /* rad, angle between current sun estimate and solar array normal */
    double              sunAngleStd;        /* rad, uncertainty in sun angle estimate */
    double              d[3];               /* sun direction estimate in body frame components multiplied by common calibration coefficient*/
    double              dPrev[3];           /* last sun direction estimate in body frame components */
    double              dPrevTime;          /* sec, time of last sun direction estimate in body frame components */
    double              omega[3];           /* rad/s, spacecraft body rates as estimated by CSS filter, in body frame */
    double              omegaPrev[3];       /* rad/sec, last recorded angular velocity in gyro frame, use to integrate forward to current time */
    double              rwCtrlPrev[NUM_RW]; /* previous commanded reaction whel torque */
    double              omegaMax;           /* rad/s, ground setable value for cutoff for cssEst.omega */
    double              computedCssValue[NUM_CSS]; /* computed CSS value */
    /* The following variables are used exclusively by the EKF/ECKF estimators */
    double              nisLimit;           /* upper bound of chi-square test for normalized innovation squared values, in MATLAB: confidenceInterval = 99.7; alpha = 1-confidenceInterval/100; chi2inv(1-alpha/2,1) */
    double              P0[NUM_CSSEST_STATES][NUM_CSSEST_STATES];   /* initial state covariance matrix */
    double              P[NUM_CSSEST_STATES][NUM_CSSEST_STATES];    /* state covariance matrix */
    double              sunDirRateSpecDens[3];                      /* rad/s^1/2, spectral density of sun-direction rate process noise */
    double              iruNoiseSpecDens[3];                        /* rad/s^1/2, spectral density of iru noise */
    double              Q[NUM_CSSEST_ETA][NUM_CSSEST_ETA];          /* state process noise covariance matrix */
    double              cssNoiseStd[NUM_CSS];                       /* measurement noise uncertainty */
    double              cssNis[NUM_CSS];    /* CSS normalized innovation squared */
} CSSEstimator_t;

/* Star tracker estimator structure */
typedef struct STRateEstimator_t {
    CommandedState_t    cmdState;           /* Commanded state of estimator: it is either ON or OFF */
    char                isEstimating;       /* Determines if estimator is ON and has enough information to estimate */
    double              omega[3];           /* rad/s, spacecraft body rates as estimated using ST measurements, in body frame */
    double              qPrev[4];           /* previous measured attitude quaternion */
    double              tPrev;              /* sec, time of previous quaternion measurement */
} STRateEstimator_t;

/* Expected state of spacecraft for state change */
typedef struct ExpectedState_t {
    ComponentState_t    rw[NUM_RW];         /* Expected state of device */
    ComponentState_t    iru;                /* Expected state of device */
    ComponentState_t    pod;                /* Expected state of device */
    ComponentState_t    tam;                /* Expected state of device */
    ComponentState_t    tr[NUM_TR];         /* Expected state of device */
    ComponentState_t    css[NUM_CSS];       /* Expected state of device */
    ComponentState_t    st;                 /* Expected state of device */
    EPSState_t          eps;                /* Power system state */
    CDHState_t          cdh;                /* CDH system state */
    CommandedState_t    stateEstimator;     /* If estimator is ON or OFF */
    CommandedState_t    orbitPropagator;    /* If estimator is ON or OFF */
    CommandedState_t    cssEstimator;       /* If estimator is ON or OFF */
    CommandedState_t    stEstimator;        /* If estimator is ON or OFF */
} ExpectedState_t;

/* Boolean Definition */
typedef enum {
    BOOL_FALSE = 0,
    BOOL_TRUE
} boolean_t;

/* Low pass filter flag enumeration */
typedef enum {
    FILTER_TR_1_U,
    FILTER_TR_2_U,
    FILTER_TR_3_U,
    FILTER_RW_1_U,
    FILTER_RW_2_U,
    FILTER_RW_3_U,
    FILTER_RW_4_U,
    FILTER_CSSEST_OMEGA_1,
    FILTER_CSSEST_OMEGA_2,
    FILTER_CSSEST_OMEGA_3,
    FILTER_IRU_OMEGA_1,
    FILTER_IRU_OMEGA_2,
    FILTER_IRU_OMEGA_3,
    FILTER_CSS_1,
    FILTER_CSS_2,
    FILTER_CSS_3,
    FILTER_CSS_4,
    FILTER_CSS_5,
    FILTER_CSS_6,
    FILTER_CSS_7,
    FILTER_CSS_8,
    FILTER_LR_1,
    FILTER_LR_2,
    FILTER_LR_3,
    MAX_FILTER_PARAMETER
} FilterParameter_t;

/* Spacecraft structure
* State of the spacecraft as seen by the spacecraft
*/
typedef struct Spacecraft_t {
    ADCSState_t          adcsState;             /* attitude control mode. Only changed by FSW */
    ADCSState_t          adcsStatePrior;        /* attitude control mode from the prior control cycle.  */
    ControlMethod_t      controlMethod;         /* attitude control method */
    EPSState_t           epsState;              /* power state of eps. Only changed by FSW */
    CDHState_t           cdhState;              /* command and data handling mode. Only changed by FSW */
    ADCSState_t          stateRequest;          /* ADCS's requested new state. FSW will change adcsState to this value */


    /* Sub components */
    Controller_t         control;
    Thrust_t             acsThrusters[NUM_ACS_THRUSTER];
    Thrust_t             dvThrusters[NUM_DV_THRUSTER];
    CSS_t                css[NUM_CSS];
    TAM_t                tam;
    IRU_t                iru;
    POD_t                pod; 
    RW_t                 rw[NUM_RW];
    int                  numAvailableRWs;       /* number of RW that have a component state ON */
    ST_t                 st;
    TR_t                 tr[NUM_TR];
    int                  numAvailableTRs;       /* number of TR that have a component state ON */

    StateEstimator_t     stateEst;
    OrbitPropagator_t    orbitProp;
    CSSEstimator_t       cssEst;
    STRateEstimator_t    stEst;
    OrbitFrameStates_t   orbitFrameStates;
    double               earthHeading_N[3];     /*!< Earth heading vector in inertial frame components */

//    EarthMagFieldModel_t earthMagFieldModel;    /* flag for determining which Earth magnetic field model to use */
    double               JD0;                   /* Julian date at beginning of mission */

    double               lowPassFilterHz;
    unsigned long        lowPassFilterFlags;    /* bit flags determining what is passed through a low pass filter */
    /* see FilterParameter_t for list of bit flag indices */

    /* Physical properties of the spacecraft */
    double               I[3][3];               /* kg*m^2, spacecraft inertia tensor */
    double               Iinv[3][3];            /* spacecraft inertia tensor m33Inverse */
    double               cHatSolarpanel_B[3];   /* solar cell normal vector in body frame components */
    double               cHatEarthAntenna_B[3]; /* earth communication antenna heading in body frame components */
    double               CB[3][3];              /* DCM from body (primary) to communication frame */
    double               TB[3][3];              /* DCM from body (primary) to thruster frame */
    double               thrustGroups[6][5];    /* thrusters grouped according to the axis about which they can produce a torque */
    /* Current state of the spacecraft */
    double               r_N[3];                /* km, orbital position vector in inertial frame */
    double               v_N[3];                /* km/s, orbital velocity vector in inertial frame */
    classicElements      oe;                    /* classical orbit element set */

    double               sigma[3];              /* MRP attitude set */
    double               omega[3];              /* rad/sec, spacecraft body rates in the body frame */

    double               B_B[3];                /* tesla, estimate of magnetic field vector in body frame components */
    double               B_tPrev;               /* tesla, time at which previous estimate of magnetic field was made */

    double               earthGamma;            /* rad, earth's current rotation angle from J2000 */
    double               earthGamma0;           /* rad, earth's rotation angle from J2000 at start of mission */

    double               mu;                    /* gravity constant of the primary celestial body */

    boolean_t            gsInRange;             /* TRUE if in range of ground station */

    ExpectedState_t      expectedNadir;         /* Expected state of spacecraft when changing to Nadir state */
    ExpectedState_t      expectedThrusting;     /* Expected state of spacecraft when changing to Thrust state */

#if CALC_CSSEST_TIMING
    FILE                *cssEstPropagateTimeFile;
    FILE                *cssEstEstimateTimeFile;
#endif

} Spacecraft_t;

/* Functions to set flags */
#define isBitFlagSet(flag, index) (((flag) & (1 << (index))) != 0)
#define setBitFlag(flag, index) ((flag) |= (1 << (index)))
#define clearBitFlag(flag, index) ((flag) &= ~(1 << (index)))

/* Unions for communicating with hardware */

typedef union uchar2ushort_t {
    unsigned char       char_value[2];      /* two 8 bit unsigned char */
    unsigned short int  value;              /* resulting 16 bit unsigned int */
} uchar2ushort_t;

typedef union uchar2short_t {
    unsigned char       char_value[2];      /* two 8 bit unsigned integers */
    signed short int    value;              /* resulting 16 bit signed int */
} uchar2short_t;

typedef union char2short_t {
    char                char_value[2];      /* two 8 bit signed integers */
    signed short int    value;              /* resulting 16 bit signed int */
} char2short_t;

typedef union uchar2uint_t {
    unsigned char       char_value[4];      /* four 8 bit unsigned char */
    unsigned int        value;              /* resulting 32 bit unsigned integers */
} uchar2uint_t;

typedef union uchar2float_t {
    unsigned char       char_value[4];      /* four 8 bit unsigned integers */
    float               value;              /* resulting 32 bit signed float */
} uchar2float_t;

typedef union uchar2ulong_t {
    unsigned char       char_value[4];      /* four 8 bit unsigned integers */
    unsigned long       value;              /* resulting 32 bit unsigned long */
} uchar2ulong_t;

typedef union uchar2double_t {
    unsigned char       char_value[8];      /* eight 8 bit unsigned integers */
    double              value;              /* resulting 64 bit signed double */
} uchar2double_t;

#endif
