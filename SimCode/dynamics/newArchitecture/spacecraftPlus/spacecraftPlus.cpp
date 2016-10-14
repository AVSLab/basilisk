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


#include "spacecraftPlus.h"

SpacecraftPlus::SpacecraftPlus()
{
    return;
}


SpacecraftPlus::~SpacecraftPlus()
{
    return;
}

void SpacecraftPlus::computeEnergyMomentum()
{
    
}

void SpacecraftPlus::equationsOfMotion(double t)
{
    std::vector<StateEffector*>::iterator it;
    std::vector<DynamicEffector*>::iterator dynIt;

    //! - Zero all Matrices and vectors
    matrixAContrSCP.setZero();
    matrixBContrSCP.setZero();
    matrixCContrSCP.setZero();
    matrixDContrSCP.setZero();
    vecTransContrSCP.setZero();
    vecRotContrSCP.setZero();
    matrixASCP.setZero();
    matrixBSCP.setZero();
    matrixCSCP.setZero();
    matrixDSCP.setZero();
    vecTransSCP.setZero();
    vecRotSCP.setZero();

    //! - This is where gravity will be called

    //! - Loop through state effectors
    for(it = states.begin(); it != states.end(); it++)
    {
        (*it)->updateEffectorMassProps(t);
        (*it)->updateEffectorMassPropRates(t);
        (*it)->updateContributions(t, matrixAContrSCP, matrixBContrSCP, matrixCContrSCP, matrixDContrSCP, vecTransContrSCP, vecRotContrSCP);
        //! Add contributions to matrices
        matrixASCP += matrixAContrSCP;
        matrixBSCP += matrixBContrSCP;
        matrixCSCP += matrixCContrSCP;
        matrixDSCP += matrixDContrSCP;
        vecTransSCP += vecTransContrSCP;
        vecRotSCP += vecRotContrSCP;
    }

    //! - Loop through dynEffectors
    for(dynIt = dynEffectors.begin(); dynIt != dynEffectors.end(); dynIt++)
    {
        //! Empty for now
    }

    hub.computeDerivatives(t, matrixASCP, matrixBSCP, matrixCSCP, matrixDSCP, vecTransSCP, vecRotSCP);

    //! - Loop through state effectors for compute derivatives
    for(it = states.begin(); it != states.end(); it++)
    {
        //! These matrices should be NULL, because the stateEffectors don't need to know about these Matrices
        (*it)->computeDerivatives(t, matrixASCP, matrixBSCP, matrixCSCP, matrixDSCP, vecTransSCP, vecRotSCP);
    }

}
void SpacecraftPlus::integrateState(double t)
{
    
}

void SpacecraftPlus::initializeDynamics()
{
    std::vector<StateEffector*>::iterator it;
    std::vector<DynamicEffector*>::iterator dynIt;
    
    hub.registerStates(dynManager);
    for(it = states.begin(); it != states.end(); it++)
    {
        (*it)->registerStates(dynManager);
    }
    
    hub.linkInStates(dynManager);
    for(it = states.begin(); it != states.end(); it++)
    {
        (*it)->linkInStates(dynManager);
    }
    
    for(dynIt = dynEffectors.begin(); dynIt != dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInStates(dynManager);
    }
    //! Need to zero m_SC, c_B, ISCPntB_B, cPrime_B, and ISCPntBPrime_B in the properties manager
}

void SpacecraftPlus::SelfInit()
{
    
}
void SpacecraftPlus::CrossInit()
{
    
}
void SpacecraftPlus::UpdateState(uint64_t CurrentSimNanos)
{
    
}
