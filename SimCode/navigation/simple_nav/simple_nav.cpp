#include "navigation/simple_nav/simple_nav.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/rigidBodyKinematics.h"
#include "environment/spice/spice_interface.h"
#include <iostream>
#include <cstring>

/*! This is the constructor for the simple nav model.  It sets default variable 
    values and initializes the various parts of the model */
SimpleNav::SimpleNav()
{
    inputStateName = "inertial_state_output";
    outputNavName = "simple_nav_output";
    inputSunName = "sun_planet_data";
    crossTrans = false;
    crossAtt = false;
    outputBufferCount = 2;
    inputStateID = -1;
    outputDataID = -1;
    AMatrix.clear();
    PMatrix.clear();
    prevTime = 0;
    memset(&outState, 0x0, sizeof(NavStateOut));
    return;
}

/*! Destructor.  Nothing here. */
SimpleNav::~SimpleNav()
{
    return;
}

/*! This is the self-init routine for the simple navigation model.  It 
    initializes the various containers used in the model as well as creates the 
    output message.  The error states are allocated as follows:
    Total states: 18
    Position errors [0-2]
    Velocity errors [3-5]
    Attitude errors [6-8]
    Body Rate errors [9-11]
    Sun Point error [12-14]
    Accumulated DV errors [15-17]
    @return void
*/
void SimpleNav::SelfInit()
{
    //! Begin method steps
    uint64_t numStates = 18;
    std::vector<double>::iterator it;
    //! - Create a new message for the output simple nav state data
    outputDataID = SystemMessaging::GetInstance()->
        CreateNewMessage(
                     outputNavName, sizeof(NavStateOut), outputBufferCount);
    //! - Initialize the propagation matrix to default values for use in update
    AMatrix.clear();
    AMatrix.insert(AMatrix.begin(), numStates*numStates, 0.0);
    mSetIdentity(AMatrix.data(), numStates, numStates);
    it = AMatrix.begin();
    //! - Depending on whether we cross-prop position/attitude, set off-diagonal terms
    for(uint32_t i=0; i<3 && (crossTrans || crossAtt); i++)
    {
        it += i + 3;  // Velocity propagation location
        *it = crossTrans ? 1.0 : 0.0; // Scale this by dt in the UpdateState method
        it += 6;
        *it = crossAtt ? 0.25 : 0.0; // MRPs propagate as 1/4 omega when near zero (SAA for errors)
        it += 9 - i;
    }
    //! - Alert the user if the noise matrix was not the right size.  That'd be bad.
    if(PMatrix.size() != numStates*numStates)
    {
        std::cerr << "Your process noise matrix (PMatrix) is not 18*18.";
        std::cerr << "  You should fix that.  Popping zeros onto end"<<std::endl;
        PMatrix.insert(PMatrix.begin()+PMatrix.size(), numStates*numStates - PMatrix.size(),
                       0.0);
    }
    //! - Set the matrices of the lower level error propagation (GaussMarkov)
    errorModel.setNoiseMatrix(PMatrix);
    errorModel.setRNGSeed(RNGSeed);
    errorModel.setUpperBounds(walkBounds);
}

/*! This method pulls the input message IDs from the messaging system.  It will 
    alert the user if either of them are not found in the messaging database
    @return void
*/
void SimpleNav::CrossInit()
{
    //! Begin method steps
    //! - Obtain the ID associated with the input state name and alert if not found.
    inputStateID = SystemMessaging::GetInstance()->
        FindMessageID(inputStateName);
    if(inputStateID < 0)
    {
        std::cerr << "Warning: input state message name: " << inputStateName;
        std::cerr << " could not be isolated, message disabled" << std::endl;
    }
    //! - Obtain the ID associated with the input Sun name and alert if not found.
    inputSunID =SystemMessaging::GetInstance()->
    FindMessageID(inputSunName);
    if(inputSunID < 0)
    {
        std::cerr << "Warning: input Sun message name: " << inputSunName;
        std::cerr << " could not be isolated, message disabled" << std::endl;
    }
}

/*! This method computes the output states for the module.  It reads the input 
    messages that were mapped in cross-init and then uses those inputs as well 
    as the calculated model errors to compute what the output navigation state 
    should be.  It then writes the aggregate information to the output message.
    @return void
    @param Clock The clock time associated with the model's update call
*/
void SimpleNav::computeOutput(uint64_t Clock)
{
    OutputStateData localState;
    SingleMessageHeader localHeader;
    SpicePlanetState sunState;
    double sc2SunInrtl[3];
    double T_inrtl2bdy[3][3];
    double T_bdyT2bdyO[3][3];
  
    //! Begin method steps
    //! - Obtain the messages associated with the vehicle state and the sun state
    memset(&sunState, 0x0, sizeof(SpicePlanetState));
    memset(&localState, 0x0, sizeof(SpicePlanetState));
    if(inputStateID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(inputStateID, &localHeader,
            sizeof(OutputStateData), reinterpret_cast<uint8_t*>(&localState));
    }
    if(inputSunID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(inputSunID, &localHeader,
            sizeof(SpicePlanetState), reinterpret_cast<uint8_t*>(&sunState));
    }
    
    //! - Add errors to the simple cases (everything except sun-pointing)
    v3Add(localState.r_N, &(navErrors.data()[0]), outState.vehPosition);
    v3Add(localState.v_N, &(navErrors.data()[3]), outState.vehVelocity);
    addMRP(localState.sigma, &(navErrors.data()[6]), outState.vehSigma);
    v3Add(localState.omega, &(navErrors.data()[9]), outState.vehBodyRate);
    v3Add(localState.TotalAccumDVBdy, &(navErrors.data()[15]),
          outState.vehAccumDV);
    //! - For the sun pointing output, compute the spacecraft to sun vector, normalize, and trans 2 body.
    v3Subtract(sunState.PositionVector, localState.r_N, sc2SunInrtl);
    v3Normalize(sc2SunInrtl, sc2SunInrtl);
    MRP2C(localState.sigma, T_inrtl2bdy);
    m33MultV3(T_inrtl2bdy, sc2SunInrtl, outState.vehSunPntBdy);
    MRP2C(&(navErrors.data()[12]), T_bdyT2bdyO);
    m33MultV3(T_bdyT2bdyO, outState.vehSunPntBdy, outState.vehSunPntBdy);
    
    //! - Write the composite information into the output state message.
    SystemMessaging::GetInstance()->
        WriteMessage(outputDataID, Clock, sizeof(NavStateOut),
                     reinterpret_cast<uint8_t*> (&outState));
}

/*! This method performs all of the run-time operations for the simple nav model.
    It primarily sets the propagation matrix, requests new random errors from 
    its GaussMarkov model, and then applies those errors to the observed truth 
    states.
    @return void
    @param CurrentSimNanos The clock time associated with the model call
*/
void SimpleNav::UpdateState(uint64_t CurrentSimNanos)
{
    double timeStep;
    std::vector<double>::iterator it;
    
    //! Begin method steps
    std::vector<double> localProp = AMatrix;
    //! - Compute timestep since the last call
    timeStep = (CurrentSimNanos - prevTime)*1.0E-9;
    it = localProp.begin();
    //! - Apply that time step to the pos/vel att/rate cross correlation terms
    for(uint32_t i=0; i<3; i++)
    {
        it += i+3;
        *it *= timeStep;
        it += 6;
        *it *= timeStep;
        it += 9-i;
    }
    //! - Set the GaussMarkov propagation matrix and compute errors
    errorModel.setPropMatrix(localProp);
    errorModel.computeNextState();
    navErrors = errorModel.getCurrentState();
    //! - Apply observed errors to truth inputs and output information. 
    computeOutput(CurrentSimNanos);
    prevTime = CurrentSimNanos;
}
