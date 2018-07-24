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
* SpacecraftSimDefinitions.cpp
*
*/

#include "SpacecraftSimDefinitions.h"
#include <cmath>
#include "enumConversions.h"
extern "C" {
#include "utilities/linearAlgebra.h"
#include "utilities/rigidBodyKinematics.h"
#include "utilities/astroConstants.h"
}

typedef enum {
    DEVICE_RW1,
    DEVICE_RW2,
    DEVICE_RW3,
    DEVICE_RW4,
    DEVICE_ST,
    DEVICE_IRU,
    DEVICE_TAM,
    DEVICE_POD,
    DEVICE_CDH,
    MAX_SERVER_DEVICE
} ServerDevice_t;

template<> const enumMap<CelestialObject_t> enumStrings<CelestialObject_t>::data = {
    {CELESTIAL_MERCURY, "CELESTIAL_MERCURY"},
    {CELESTIAL_VENUS, "CELESTIAL_VENUS"},
    {CELESTIAL_EARTH, "CELESTIAL_EARTH"},
    {CELESTIAL_MOON, "CELESTIAL_MOON"},
    {CELESTIAL_MARS, "CELESTIAL_MARS"},
    {CELESTIAL_PHOBOS, "CELESTIAL_PHOBOS"},
    {CELESTIAL_DEIMOS, "CELESTIAL_DEIMOS"},
    {CELESTIAL_JUPITER, "CELESTIAL_JUPITER"},
    {CELESTIAL_SATURN, "CELESTIAL_SATURN"},
    {CELESTIAL_URANUS, "CELESTIAL_URANUS"},
    {CELESTIAL_NEPTUNE, "CELESTIAL_NEPTUNE"},
    {CELESTIAL_PLUTO, "CELESTIAL_PLUTO"},
    {CELESTIAL_SUN, "CELESTIAL_SUN"},
    {MAX_CELESTIAL, "MAX_CELESTIAL"}
};

template<> const enumMap<CodeBehavior_t> enumStrings<CodeBehavior_t>::data = {
    {CODE_STANDALONE, "CODE_STANDALONE"},
    {CODE_REALITY, "CODE_REALITY"},
    {CODE_REALITY_ON_DEMAND, "CODE_REALITY_ON_DEMAND"},
    {CODE_FSW, "CODE_FSW"},
    {MAX_CODE_BEHAVIOR, "MAX_CODE_BEHAVIOR"}
};

template<> const enumMap<ComponentState_t> enumStrings<ComponentState_t>::data = {
    {COMPONENT_OFF, "COMPONENT_OFF"},
    {COMPONENT_START, "COMPONENT_START"},
    {COMPONENT_ON, "COMPONENT_ON"},
    {COMPONENT_FAULT, "COMPONENT_FAULT"},
    {COMPONENT_TEST, "COMPONENT_TEST"},
    {MAX_COMPONENT_STATE, "MAX_COMPONENT_STATE"}
};

template<> const enumMap<MCPrintFunction_t> enumStrings<MCPrintFunction_t>::data = {
    {MCPRINT_CSSEST, "MCPRINT_CSSEST"},
    {MAX_MCPRINT, "MAX_MCPRINT"}
};

template<> const enumMap<IntegrationMethod_t> enumStrings<IntegrationMethod_t>::data = {
    {INTEGRATION_RUNGE_KUTTA_4, "INTEGRATION_RUNGE_KUTTA_4"},
    {INTEGRATION_EULER, "INTEGRATION_EULER"},
    {MAX_INTEGRATION_METHOD, "MAX_INTEGRATION_METHOD"}
};

template<> const enumMap<AlbedoModel_t> enumStrings<AlbedoModel_t>::data = {
    {ALBEDO_NONE, "ALBEDO_NONE"},
    {ALBEDO_TOMS_1p25x1, "ALBEDO_TOMS_1p25x1"},
    {ALBEDO_TOMS_5x5, "ALBEDO_TOMS_5x5"},
    {ALBEDO_TOMS_10x10, "ALBEDO_TOMS_10x10"},
    {MAX_ALBEDO_MODEL, "MAX_ALBEDO_MODEL"}
};

template<> const enumMap<EarthMagFieldModel_t> enumStrings<EarthMagFieldModel_t>::data = {
    {MAGFIELD_WMM, "MAGFIELD_WMM"},
    {MAGFIELD_TILTDIPOLE, "MAGFIELD_TILTDIPOLE"},
    {MAX_MAGFIELD_MODEL, "MAX_MAGFIELD_MODEL"}
};

template<> const enumMap<ADCSState_t> enumStrings<ADCSState_t>::data = {
    {ADCS_SUN_POINTING, "ADCS_SUN_POINTING"},
    {ADCS_HILL_POINTING, "ADCS_HILL_POINTING"},
    {ADCS_HILL_SPIN, "ADCS_HILL_SPIN"},
    {ADCS_VELOCITY_POINTING, "ADCS_VELOCITY_POINTING"},
    {ADCS_VELOCITY_SPIN, "ADCS_VELOCITY_SPIN"},
    {ADCS_THRUSTING, "ADCS_THRUSTING"},
    {ADCS_EARTH_POINTING, "ADCS_EARTH_POINTING"},
    {ADCS_INERTIAL3D_POINTING, "ADCS_INERTIAL3D_POINTING"},
    {ADCS_INERTIAL3D_SPIN, "ADCS_INERTIAL3D_SPIN"},
    {ADCS_RATE_DAMPING, "ADCS_RATE_DAMPING"},
    {MAX_ADCS_STATE, "MAX_ADCS_STATE"}
};

template<> const enumMap<CommandedState_t> enumStrings<CommandedState_t>::data = {
    {CMD_OFF, "CMD_OFF"},
    {CMD_ON, "CMD_ON"},
    {MAX_CMD_STATE, "MAX_CMD_STATE"}
};

RWSim::RWSim()
{
    this->state = COMPONENT_OFF;
    this->u_current = 0.0;
    this->u_max = 0.0;
    this->Omega = 0.0;
    this->Omega_max = 0.0;
    this->Js = 0.0;
    v3SetZero(this->gsHat_S);
    this->motorTemp1 = 0.0;
    this->motorTemp2 = 0.0;
    this->power = 0.0;
    for(int i = 0; i < numElems(this->input); i++) {
        this->input[i] = '\0';
    }
    this->inputLength = 0;
    for(int i = 0; i < numElems(this->output); i++) {
        this->output[i] = '\0';
    }
    this->outputLength = 0;
    this->resetCounter = 0;
}

RWSim::~RWSim()
{
    
}

Thruster::Thruster()
{
    v3SetZero(this->r_B);
    v3SetZero(this->gt_B);
    this->maxThrust = 3.5; // Newtons
    this->minThrust = 0.0;
    this->timeStart = 0.0;
    this->duration = 0.0;
    this->thrustState = CMD_OFF;
    this->pulseTime = 0.0;
    this->isValveOpen = false;
    this->pairId = 0;
    v3SetZero(this->tau_B);
    
    for(int i = 0; i < numElems(this->input); i++) {
        this->input[i] = '\0';
    }
    this->inputLength = 0;
    for(int i = 0; i < numElems(this->output); i++) {
        this->output[i] = '\0';
    }
    this->outputLength = 0;
};

Thruster::Thruster(double r_B[3], double gt_B[3]) : Thruster()
{
    // @TODO this code could be refactored to use a std::array rather than C array.
    // This would remove the need to call v3Set. - Not a big issue really - PAT.
    v3Set(r_B[0], r_B[1], r_B[3], this->r_B);
    v3Set(gt_B[0], gt_B[1], gt_B[3], this->gt_B);
    
    //  double tempAxis;
    //  v3Cross(r_B, gt_B, &tempAxis);
    //  v3Normalize(&tempAxis, &this->torqueAxis);
    return;
}

Thruster::~Thruster()
{
    
};

SpacecraftSim::SpacecraftSim(void)
{
    this->time = 0.0;
    this->timeStamp = 0.0;
    this->rerunCaseNum = 0;
    this->maxSimTime = 100.0 * 60.0;
    
    this->celestialObject = CELESTIAL_EARTH;
    this->mu = MU_EARTH;
    this->req = REQ_EARTH;
    this->polarRate = OMEGA_EARTH;
    
    this->useRealTimeSync = 0;
    this->realTimeSpeedUpFactor = 1.0;
    this->useOpenGLVisualization = 0;
    this->openglLastSync = 0.0;
    
    this->useGravityGradientTorque = 0;
    this->useReactionWheelJitter = 0;
    this->useGravityPerturbModel = 0;
    this->useAtmosDragModel = 0;
    this->useSrpModel = 0;
    this->albedoModel = ALBEDO_NONE;
    
    this->ctrlState = CMD_ON;
    this->ctrlUsingTrueSunDirection = 0;
    this->ctrlUsingTrueAttitude = 0;
    this->ctrlUsingTrueRates = 0;
    
    this->DVThrusterManeuverStartTime = -1.0;
    this->DVThrusterManeuverDuration = -1.0;
    
    /* parameters for calculating power */
    this->rwPowerMin = 1.0;
    this->rwPowerMax = 3.0;
    this->trPowerMax = 1.0;
    
    this->adcsState = MAX_ADCS_STATE;
//    this->cdhState = MAX_CDH_STATE;
//    this->epsState = MAX_EPS_STATE;
    
    m33SetZero(this->I);
    m33SetZero(this->Iinv);
    
    this->A_drag = 0.0;
    this->A_srp = 0.0;
    this->mass = 0.0;
    this->Cd = 0.0;
    v3Set(0, 0, 1, this->cHatSolarpanelSim_B);
    v3Set(0,cos((90-22.2)*D2R),sin((90-22.2)*D2R), this->cHatEarthAntennaSim_B);
    v3Normalize(this->cHatEarthAntennaSim_B, this->cHatEarthAntennaSim_B);
    
    this->oe.a = 0.0;
    this->oe.e = 1.0;
    this->oe.i = 0.0;
    this->oe.Omega = 0.0;
    this->oe.omega = 0.0;
    this->oe.f = 0.0;
    this->gamma = 0.0;
    this->n = 0.0;
    this->T0= 0.0;
    
    v3SetZero(this->r_N);
    v3SetZero(this->v_N);
    
    v3SetZero(this->sigma);
    v3SetZero(this->omega);
    
    v3SetZero(this->B_N);
    v3SetZero(this->B_B);
    
    v3SetZero(this->r_CS);
    v3SetZero(this->sHatN);
    v3SetZero(this->sHatB);
    this->sunAngle = 0.0;
    
    v3SetZero(this->earthHeadingSim_N);
    v3SetZero(this->earthHeadingSim_B);
}

SpacecraftSim::~SpacecraftSim(void)
{
    
}

//template<> const enumMap<CSSFaultState_t> enumStrings<CSSFaultState_t>::data = {
//    {CSSFAULT_OFF, "CSSFAULT_OFF"},
//    {CSSFAULT_STUCK_CURRENT, "CSSFAULT_STUCK_CURRENT"},
//    {CSSFAULT_STUCK_MAX, "CSSFAULT_STUCK_MAX"},
//    {CSSFAULT_STUCK_RAND, "CSSFAULT_STUCK_RAND"},
//    {CSSFAULT_STUCK, "CSSFAULT_STUCK"},
//    {CSSFAULT_RAND, "CSSFAULT_RAND"},
//    {MAX_CSSFAULT, "MAX_CSSFAULT"}
//};

CoarseSunSensor::CoarseSunSensor()
{
    this->state = COMPONENT_OFF;
    this->faultState = MAX_CSSFAULT;
    this->stuckPercent = 0.0;
    v3SetZero(this->nHatB);
    v3SetZero(this->horizonPlane);
    this->directValue = 0.0;
    this->albedoValue = 0.0;
    this->scaleFactor = 0.0;
    this->sensedValue = 0.0;
    for(int i = 0; i < numElems(this->output); i++) {
        this->output[i] = '\0';
    }
    this->outputLength  = 0;
    this->fov           = 60.0 * D2R;
    this->maxVoltage    = 0.0;
    this->phi           = 45.0 * D2R;
    this->theta         = 0.0;
    v3SetZero(this->B2P321Angles);
    v3SetZero(this->r_B);
}

CoarseSunSensor::~CoarseSunSensor()
{
}

SpiceTime::SpiceTime()
{
    this->J2000Current = 0;
    this->julianDateCurrent = 0;
    this->GPSSeconds = 0;
    this->GPSWeek = 0;
    this->GPSRollovers = 0;
}

SpiceTime::~SpiceTime(){}

InitialConditions::InitialConditions(void)
{
    this->SMA = 6850.0; /* km*/
    this->eccentricity = 0.0;
    this->inclination = 98.0; /* deg */
    this->ascendingNode = 13.5; /* deg */
    this->argPeriapse = 0.0; /* deg */
    this->trueAnomaly = 22.0; /* deg */
    this->simulationDateStr = "2015 June 1, 00:00";
    this->ET0 = 486388867.184914590000000;
    this->JD0 = 2457174.50000000;
    this->M0 = 0.0;
    this->gamma0 = 0.0;
}

InitialConditions::~InitialConditions(void)
{

}

ControllerSim::ControllerSim(void)
{
    v3SetZero(this->sigma_BR);
    v3SetZero(this->omega_BR);
    v3SetZero(this->sigma_BR_SP);
    v3SetZero(this->sigma_BR_NP);
    v3SetZero(this->sigma_BR_NS);
    v3SetZero(this->sigma_BR_VP);
    v3SetZero(this->sigma_BR_IP);
    v3SetZero(this->omega_BR_SP);
    v3SetZero(this->omega_BR_NP);
    v3SetZero(this->omega_BR_NS);
    v3SetZero(this->omega_BR_VP);
    v3SetZero(this->omega_BR_T);
    v3SetZero(this->omega_BR_IP);
    v3SetZero(this->omega_BR_RD);
}

ControllerSim::~ControllerSim(void)
{

}

IRUSim::IRUSim()
{
    this->state = COMPONENT_OFF;
    v3SetZero(this->omega);
    m33SetIdentity(BI);
    for(int i = 0; i < numElems(this->output); i++) {
        this->output[i] = '\0';
    }
    this->outputLength = 0;
    this->crcError = 0;
    v3SetZero(this->bias);
    this->id = 0;
    this->resetCounter = 0;
}

IRUSim::~IRUSim()
{

}

PODSim::PODSim()
{
    this->state = COMPONENT_OFF;
    this->gpsPosDataHz = 1.0;
    this->startUpTime = 180.0;
    for(int i = 0; i < numElems(this->output); i++) {
        this->output[i] = '\0';
    }
    this->outputLength = 0;
    this->resetCounter = 0;
}

PODSim::~PODSim()
{

}

//template<> const enumMap<TempSpeedState_t> enumStrings<TempSpeedState_t>::data = {
//    {TEMP_SPEED_NOMINAL, "TEMP_SPEED_NOMINAL"},
//    {TEMP_SPEED_OVER_RATING, "TEMP_SPEED_OVER_RATING"},
//    {MAX_TEMP_SPEED_STATE, "MAX_TEMP_SPEED_STATE"}
//};


STSim::STSim()
{
    this->state = COMPONENT_OFF;
    m33SetIdentity(this->BS);
    v3SetZero(this->r_B_head1);
    v3SetZero(this->r_B_head2);
    v3SetZero(this->gs_head1);
    v3SetZero(this->gs_head2);
    this->tempDIPU = 0;
    this->tempOptHead = 0;
    this->power = 0.0;
    this->isGroundModeActive = 0;
    this->isIdleModeActive = 0;
    this->attitudeError = 0;
    this->attitudeDet = 0;
    this->sysTime = 0;
    this->lastValidTime = 0;
    for(int i = 0; i < numElems(this->input); i++) {
        this->input[i] = '\0';
    }
    this->inputLength = 0;
    for(int i = 0; i < numElems(this->output); i++) {
        this->output[i] = '\0';
    }
    this->outputLength = 0;
    this->crcError = 0;
    this->resetCounter = 0;
}

STSim::~STSim()
{

}

TRSim::TRSim()
{
    this->state = COMPONENT_OFF;
    v3SetZero(this->r_B);
    this->u = 0.0;
    v3SetZero(this->dipoleAxis);
    this->a = 0.0;
    this->b = 0.0;
    this->current = 0.0;
    this->inputLength = 0;
    this->flags = 0;
}

TRSim::~TRSim()
{

}

DCM_t::DCM_t()
{
    m33SetIdentity(this->DCM);
}

DCM_t::~DCM_t()
{

}
