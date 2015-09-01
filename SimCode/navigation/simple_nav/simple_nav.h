
#ifndef SIMPLE_NAV_H
#define SIMPLE_NAV_H

#include <vector>
#include "utilities/sys_model.h"
#include "utilities/dyn_effector.h"
#include "utilities/gauss_markov.h"
#include "../ADCSAlgorithms/attDetermination/CSSEst/navStateOut.h"
/*! \addtogroup SimModelGroup
 * @{
 */

//!@brief Simple navigation model used to provide error-ed truth (or truth)
/*! This class is used to perturb the truth state away using a gauss-markov 
 error model.  It is designed to look like a random walk process put on top of 
 the nominal position, velocity, attitude, and attitude rate.  This is meant to 
 be used in place of the nominal navigation system output*/
class SimpleNav: public SysModel {
public:
    SimpleNav();
    ~SimpleNav();
   
    void SelfInit();
    void CrossInit(); 
    void UpdateState(uint64_t CurrentSimNanos);
    void computeOutput(uint64_t Clock);
    
public:
    uint64_t outputBufferCount;        //!< -- Number of output state buffers in msg
    std::vector<double> PMatrix;       //!< -- Covariance matrix used to perturb state
    std::vector<double> walkBounds;    //!< -- "3-sigma" errors to permit for states
    std::vector<double> navErrors;     //!< -- Current navigation errors applied to truth
    std::string inputStateName;        //!< -- Message that contains s/c state
    std::string outputNavName;         //!< -- Message that we output state to
    std::string inputSunName;          //!< -- Message name for the sun state
    bool crossTrans;                   //!< -- Have position error depend on velocity
    bool crossAtt;                     //!< -- Have attitude depend on attitude rate
    NavStateOut outState;              //!< -- navigation state provided by this model
private:
    int64_t inputStateID;              //!< -- Message ID associated with s/c state
    int64_t outputDataID;              //!< -- Message ID associated with nav state
    int64_t inputSunID;                //!< -- Message ID associated with the sun position
    std::vector<double> AMatrix;       //!< -- The matrix used to propagate the state
    GaussMarkov errorModel;            //!< -- Gauss-markov error states
    uint64_t prevTime;                 //!< -- Previous simulation time observed
};

/*! @} */

#endif
