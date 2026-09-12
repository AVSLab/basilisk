// Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
// This file is distributed under the ISC License in LICENSE.

%module effectorNamingTestSupport

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
%}

class DynParamManager;

%inline %{
/** @brief Enable the internal naming policy for Python lifecycle tests.
 * @param manager Empty manager whose policy will be set before preparation.
 */
void enableManagerLocalNaming(DynParamManager& manager)
{
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
}
%}
