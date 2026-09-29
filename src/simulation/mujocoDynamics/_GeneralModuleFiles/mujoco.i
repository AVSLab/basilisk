/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

%module(package="Basilisk.simulation") mujoco

%include "std_string.i"
%include "std_vector.i"
%include "swig_eigen.i"
%include "swig_conly_data.i"

%include "architecture/utilities/bskException.swg"

%define MUJOCO_BSK_EXCEPTION_POLICY
%default_bsk_exception(
  catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
);
%enddef

MUJOCO_BSK_EXCEPTION_POLICY

%pythonbegin %{
from Basilisk.architecture import messaging
%}

%include "simulation/dynamics/_GeneralModuleFiles/dynParamManagerImport.swg"
%include "simulation/dynamics/_GeneralModuleFiles/dynamicObjectImport.swg"

MUJOCO_BSK_EXCEPTION_POLICY
%include "MJInterpolators.swg"
MUJOCO_BSK_EXCEPTION_POLICY
%include "MJActuator.swg"
MUJOCO_BSK_EXCEPTION_POLICY
%include "MJSite.swg"
MUJOCO_BSK_EXCEPTION_POLICY
%include "MJJoint.swg"
MUJOCO_BSK_EXCEPTION_POLICY
%include "MJBody.swg"
MUJOCO_BSK_EXCEPTION_POLICY
%include "MJEquality.swg"
MUJOCO_BSK_EXCEPTION_POLICY
%include "MJScene.swg"
