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

#ifndef Eclipse_H
#define Eclipse_H

#include <vector>
#include <map>
#include <Eigen/Dense>
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/eclipseSimMsg.h"
#include "utilities/linearAlgebra.h"

/*! \addtogroup SimModelGroup
 *  This group is used to model parts of the vehicle and the surrounding environment
 *  in the simulation system.  All components/dynamics/environment models are a
 *  part of this group.
 * @{
 */

//! @brief The eclipse class gets generates sun illumination state at a particular inertial position
/*!
 The module
 [PDF Description](Basilisk-eclipse-20171101.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.

 */

class Eclipse: public SysModel {
public:
    Eclipse();
    ~Eclipse();
    
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void writeOutputMessages(uint64_t CurrentClock);
    std::string addPositionMsgName(std::string msgName);
    void addPlanetName(std::string planetName);
    
public:
    uint64_t outputBufferCount; //!< -- Number of output buffers to use
    std::string sunInMsgName; //!< -- Internal vector of planets

private:
    std::vector<std::string> planetNames;  //!< -- Names of planets we want to track
    std::vector<std::string> planetInMsgNames; //!< -- A vector of planet incoming message names ordered by the sequence in which planet names are added to the module
    std::map<int64_t, SpicePlanetStateSimMsg> planetInMsgIdAndStates; //!< -- A map of incoming planet message Ids and planet state ordered by the sequence in which planet names are added to the module
    std::vector<float> planetRadii; //!< [m] A vector of planet radii ordered by the sequence in which planet names are added to the module
    int64_t sunInMsgId; //!< -- Internal
    SpicePlanetStateSimMsg sunInMsgState;
    std::vector<std::string> positionMsgNames;  //!< -- vector of msg names for each position state for which to evaluate eclipse conditions.
    std::map<int64_t, SCPlusStatesSimMsg> positionInMsgIdAndState;
    std::vector<int64_t> eclipseOutMsgId;
    std::vector<std::string> eclipseOutMsgNames;
    std::vector<double> eclipseShadowFactors;

private:
    void readInputMessages();
    double computePercentShadow(double planetRadius, Eigen::Vector3d r_HB_N, Eigen::Vector3d s_BP_N);
    double getPlanetEquatorialRadius(std::string planetSpiceName);

};

/*! @} */

#endif
