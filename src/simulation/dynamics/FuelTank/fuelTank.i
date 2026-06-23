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

%module fuelTank

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "fuelTank.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_eigen.i"
%include "swig_conly_data.i"
%include "swig_deprecated.i"

%include "sys_model.i"
%include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.i"
%include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
%deprecated_variable(
    FuelTank,
    nameOfMassState,
    "2027/06/22",
    "Direct access to nameOfMassState is deprecated. Use setNameOfMassState() and getNameOfMassState() instead."
)
%deprecated_variable(
    FuelTank,
    dcm_TB,
    "2027/06/22",
    "Direct access to dcm_TB is deprecated. Use setDcm_TB() and getDcm_TB() instead."
)
%deprecated_variable(
    FuelTank,
    r_TB_B,
    "2027/06/22",
    "Direct access to r_TB_B is deprecated. Use setR_TB_B() and getR_TB_B() instead."
)
%deprecated_variable(
    FuelTank,
    updateOnly,
    "2027/06/22",
    "Direct access to updateOnly is deprecated. Use setUpdateOnly() and getUpdateOnly() instead."
)
%deprecated_variable(
    FuelTank,
    fuelLeakRate,
    "2027/06/22",
    "Direct access to fuelLeakRate is deprecated. Use setFuelLeakRate() and getFuelLeakRate() instead."
)
%include "fuelTank.h"

%include "architecture/msgPayloadDefC/FuelTankMsgPayload.h"
struct FuelTankMsg_C;
%include "architecture/msgPayloadDefC/MassFlowRateMsgPayload.h"
struct MassFlowRateMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
