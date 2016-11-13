
#ifndef RADIATION_PRESSURE_H
#define RADIATION_PRESSURE_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "environment/spice/spice_planet_state.h"
#include <array>

/*! \addtogroup SimModelGroup
 * @{
 */
    
//! @brief Thruster dynamics class used to provide thruster effects on body
/*! This class is used to ...*/
class RadiationPressure: public SysModel, public DynamicEffector{
public:
    RadiationPressure();
    ~RadiationPressure();

    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void linkInStates(DynParamManager& statesIn);
    void writeOutputMessages(uint64_t currentClock);
    void readInputMessages();
    void computeBodyForceTorque(uint64_t currentTime);
    void setUseCannonballModel(bool use);
    
private:
    void computeCannonballModel(Eigen::Vector3d rSunB_B);
    void computeLookupModel(Eigen::Vector3d rSunB_B);

public:
    double  area;                    //!< m^2 Body surface area
    double  coefficientReflection;   //!< -- Factor grouping surface optical properties
    std::string sunEphmInMsgName;   //!< -- Message name for the sun state
    std::string stateInMsgName;     //!< -- Message name for the S/C state
    std::vector<std::vector<double>> lookupForce_B;     //!< -- Force on S/C at 1 AU from sun
    std::vector<std::vector<double>> lookupTorque_B;    //!< -- Torque on S/C
    std::vector<std::vector<double>> lookupSHat_B;      //!< -- S/C to sun unit vector defined in the body frame.

private:
    StateData *hubSigma;            //!< -- spacecraft hub attitude
    StateData *hubPosition;         //!< -- spacecraft hub position
    bool    useCannonballModel;         //!< -- Use cannnonball or lookup table model
    int64_t sunEphmInMsgID;             //!< -- Message ID for incoming data
    SpicePlanetState sunEphmInBuffer;   //!< -- Buffer for incoming ephemeris message data
//    OutputStateData stateInBuffer;      //!< -- Buffer for incoming state message data
};

/*! @} */

#endif
