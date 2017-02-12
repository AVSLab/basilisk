
#ifndef RADIATION_PRESSURE_H
#define RADIATION_PRESSURE_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "environment/spice/spice_planet_state.h"
#include "dynamics/spacecraftPlus/spacecraftPlus.h"
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
    void computeBodyForceTorque(double integTime);
    void setUseCannonballModel(bool use);
    void addForceLookupBEntry(Eigen::Vector3d vec);
    void addTorqueLookupBEntry(Eigen::Vector3d vec);
    void addSHatLookupBEntry(Eigen::Vector3d vec);
    
private:
    void computeCannonballModel(Eigen::Vector3d rSunB_B);
    void computeLookupModel(Eigen::Vector3d rSunB_B);

public:
    double  area;                    //!< m^2 Body surface area
    double  coefficientReflection;   //!< -- Factor grouping surface optical properties
    std::string sunEphmInMsgName;   //!< -- Message name for the sun state
    std::string stateInMsgName;     //!< -- Message name for the S/C state
    std::string inertialPosPropName; //!< -- Property name for the S/C inertial position
    std::vector<Eigen::Vector3d> lookupForce_B;     //!< -- Force on S/C at 1 AU from sun
    std::vector<Eigen::Vector3d> lookupTorque_B;    //!< -- Torque on S/C
    std::vector<Eigen::Vector3d> lookupSHat_B;      //!< -- S/C to sun unit vector defined in the body frame.

private:
    StateData *hubSigma;            //!< -- spacecraft hub attitude
    StateData *hubPosition;         //!< -- spacecraft hub position
    Eigen::MatrixXd *inertialPosProp;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    bool    useCannonballModel;         //!< -- Use cannnonball or lookup table model
    int64_t sunEphmInMsgId;             //!< -- Message ID for incoming data
    SpicePlanetState sunEphmInBuffer;   //!< -- Buffer for incoming ephemeris message data
    int64_t stateInMsgId;
    bool stateRead;
    SCPlusOutputStateData stateInBuffer;      //!< -- Buffer for incoming state message data
};

/*! @} */

#endif
