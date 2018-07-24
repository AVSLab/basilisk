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
//
// boost_communication.cpp
//
//

#include "boost_communication.h"
#include <string>


OpenGLIO::OpenGLIO()
    : ipAddress("127.0.0.1")
{
    this->scSim = SpacecraftSim();
}

OpenGLIO::~OpenGLIO()
{
    shutdown();
}

void OpenGLIO::setOutputSpacecraft(SpacecraftSim scSim)
{
    this->scSim = scSim;
}

bool OpenGLIO::initialize()
{
    std::stringstream ss;
    ss << IP_BASE_PORT;
    if(!this->server.acceptConnections(this->ipAddress, ss.str())) {
        return false;
    }
    return true;
}

bool OpenGLIO::shutdown()
{
    if(!this->server.close()) {
        return false;
    }
    return true;
}

bool OpenGLIO::send()
{
    this->server.setOutboundData(&this->scSim);
    return true;
}

bool OpenGLIO::receive(SpacecraftSim *scSim)
{
    SpacecraftSim visualizationData = this->server.getInboundData();
    
//    if(visualizationData.adcsState != temp.adcsState) {
//        sc->stateRequest = visualizationData.adcsState;
//    }
//    if(visualizationData.perturbRates) {
//        for(size_t i = 0; i < 3; i++) {
//            scSim->omega[i] = scSim->perturb.initialOmegaDisp * (2.0 * prn_uniform() - 1.0);
//        }
//    }
//    if(visualizationData.controlState != temp.controlState) {
//        scSim->ctrlState = visualizationData.controlState;
//    }
    if(visualizationData.realTimeSpeedUpFactor != scSim->realTimeSpeedUpFactor)
    {
        scSim->realTimeSpeedUpFactor = visualizationData.realTimeSpeedUpFactor;
    }
    return true;
}

void OpenGLIO::setIpAddress(std::string ipAddress)
{
    // Maybe check for an excisting connection before allowing ipAddress to be set
    // or just ensure a graceful transition between connections.
    this->ipAddress = ipAddress;
}

BoostCommunication::BoostCommunication()
{

}

BoostCommunication::~BoostCommunication()
{

}

bool BoostCommunication::initializeAll(std::string tmpDataVariable)
{
//    if(scSim->useOpenGLVisualization && scSim->codeBehavior != CODE_FSW) {
//        if(!m_openGLIO.initialize(scSim)) {
//            std::cout << "called by " << __FUNCTION__ << std::endl;
//            return false;
//        }
//    }
//    switch(scSim->codeBehavior) {
//        case CODE_REALITY:
//        case CODE_REALITY_ON_DEMAND:
//            printf("codeBehavior: REALITY");
//            if(scSim->codeBehavior == CODE_REALITY_ON_DEMAND) {
//                printf("_ON_DEMAND");
//            }
//            printf("\nWaiting to connect to FSW or Flatsat Module.\n");
//            if(!m_realityCommIO.initialize(scSim)) {
//                std::cout << "called by " << __FUNCTION__ << std::endl;
//                return false;
//            }
//            break;
//
//        case CODE_FSW:
//            printf("codeBehavior: FSW\n");
//            if(!m_fswCommIO.initialize(scSim)) {
//                std::cout << "called by " << __FUNCTION__ << std::endl;
//                return false;
//            }
//            break;
//
//        case CODE_STANDALONE:
//            printf("codeBehavior: STAND_ALONE\n");
//            break;
//
//        default:
//            break;
//    }
    return true;
}

bool BoostCommunication::shutdownAll(std::string tmpDataVariable)
{
//    if(scSim->codeBehavior == CODE_REALITY || scSim->codeBehavior == CODE_REALITY_ON_DEMAND) {
//        if(!m_realityCommIO.shutdown()) {
//            std::cout << "called by " << __FUNCTION__ << std::endl;
//            return false;
//        }
//    }
//    if(scSim->codeBehavior == CODE_FSW) {
//        if(!m_fswCommIO.shutdown()) {
//            std::cout << "called by " << __FUNCTION__ << std::endl;
//            return false;
//        }
//    }
//    if(scSim->useOpenGLVisualization && scSim->codeBehavior != CODE_FSW) {
//        if(!m_openGLIO.shutdown()) {
//            std::cout << "called by " << __FUNCTION__ << std::endl;
//            return false;
//        }
//    }
    return true;
}
