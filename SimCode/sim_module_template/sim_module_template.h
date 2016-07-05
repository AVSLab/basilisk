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

#ifndef sim_module_template_h
#define sim_module_template_h

#include <vector>
#include <array>
#include "utilities/sys_model.h"
#include "utilities/dyn_effector.h"
#include "environment/spice/spice_planet_state.h"

/*! \addtogroup SimModelGroup
 * @{
 */

//! @brief The brief note is a single sentence to describe the function of this sim module.
/*! This is followed by a more lengthy description if necessary. This class is used to ...*/
/* Delete: The module class must inhert from SysModel class. The module class may inheret the DynEffector class is being included in uncoupled dynamics calculations. */
class SimModuleTemplate: public SysModel, public DynEffector{
    public:
    SimModuleTemplate();
    ~SimModuleTemplate();
    
    /* Delete: The module should override the inherented SysModel class method SelfInit() with its own implementation. All other SysModel class methods are optional. See the class SysModel for other initialization methods. This example overids CrossInit() and UpdateState(uint64_t CurrentSimNanos). */
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    
    /* Delete: If the module is outputing data to the messaging system then you should include a writeOutputMessages() method. This is not required but adheres to the standard module layout employed by Basilisk. The writeOutputMessages() method will be called from UpdateState() method. */
    void writeOutputMessages(uint64_t CurrentClock);
    void readInputs();
    
    /* Delete: ComputeDynamics() is overriding the method of the same name inhereted from the DynEffector class. */
    void ComputeDynamics(MassPropsData *massPropsData, OutputStateData *bodyState, double currentTime);
    
    private:
    void computeDummyData(double *s_B);
    
    public:
    std::string sunEphmInMsgName;   //!< -- Example message name for the sun state
    std::string stateInMsgName;     //!< -- Example message name for the spacecraft state
    std::vector<double> exampleDoubleVector;            //!< -- Example templated vector class
    std::vector<std::string> exampleStringVector;       //!< -- Example templated vector class
    std::vector<int> exampleIntVector;                  //!< -- Example templated vector class
    
    private:
    double force_B[3];                  //!< N  Computed force in body due to radiation pressure
    double torque_B[3];                 //!< Nm Computed torque in body due to radiation pressure
    std::string exampleOutMsgName;      //!< -- Example message name for outgoing data
    int64_t exampleOutMsgID;            //!< -- Example message ID for outgoing data
    int64_t sunEphmInMsgID;             //!< -- Example message ID for incoming data
    SpicePlanetState sunEphmInBuffer;   //!< -- Example buffer for incoming ephemeris message data
    int64_t stateInMsgID;               //!< -- Example message ID for incoming spacecraft state data
    OutputStateData stateInBuffer;      //!< -- Example buffer for incoming state message data
};

/*! @} */

#endif /* sim_module_template_h */
