
#ifndef BORE_ANG_CALC_H
#define BORE_ANG_CALC_H

#include <vector>
#include "utilities/sys_model.h"
#include "environment/spice/spice_planet_state.h"

/*! \addtogroup SimModelGroup
 * @{
 */

typedef struct {
   double azimuth;      //! (r) the azimuth angle of the object
   double elevation;    //! (r) the elevation angle of the object
}AngOffValues;

//! An orbital element/cartesian position and velocity converter
class BoreAngCalc: public SysModel {
public:
    BoreAngCalc();
    ~BoreAngCalc();
    
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void computeAxisPoint();
    void WriteOutputMessages(uint64_t CurrentClock);
    void ReadInputs();
    
public:
    std::string StateString;          //!< (-) port to use for conversion
    std::string celBodyString;        //!< (-) celestial body we are pointing at
    std::string OutputDataString;     //!< (-) port to use for output data
    uint64_t OutputBufferCount;       //!< (-) Count on number of buffers to output
    bool ReinitSelf;                  //!< (-) Indicator to reset conversion type
    double strBoreVec[3];             //!< (-) boresight vector in structure
    double boreVecPoint[3];           //!< (-) pointing vector in the target relative point frame
    AngOffValues boresightAng;        //!< (-) Boresigt angles relative to target
    
private:
    SpicePlanetState localPlanet;     //!< (-) planet that we are pointing at
    OutputStateData localState;       //!< (-) observed state of the spacecraft
    int64_t StateInMsgID;              // (-) MEssage ID for incoming data
    int64_t celInMsgID;              // (-) MEssage ID for incoming data
    int64_t AngOutMsgID;             // (-) Message ID for outgoing data
};

/*! @} */

#endif
