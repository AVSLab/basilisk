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

%{
   #include "MJScene.h"
%}

%pythonbegin %{
from typing import List
%}

%include "std_vector.i"
%include "std_string.i"
%template() std::vector<std::string>;
%template(MJDoubleVector) std::vector<double>;

%include "simulation/dynamics/_GeneralModuleFiles/dynParamManagerImport.swg"
%include "simulation/dynamics/_GeneralModuleFiles/dynamicObjectImport.swg"
%include "MJGeomInfo.h"

// MJScene exposes an existing fixed-size gear-vector API.
EIGEN_MAT_WRAP(Eigen::Vector6d, 159)

// Retain the owner on borrowed results using the generated Python methods.
%pythonappend MJScene::getBody %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::getSite %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::getEquality %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::getSingleActuator %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::getForceActuator %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::getTorqueActuator %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::getForceTorqueActuator %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::addJointSingleActuator %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::addSingleActuator %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::addForceActuator %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::addTorqueActuator %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::addForceTorqueActuator %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::getActState %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScene::getMassState %{
    if val is not None:
        val._swig_bsk_owner = self
%}

%ignore MJScene::fromFile;

%ignore MJScene::SelfInit;
%ignore MJScene::UpdateState;
%ignore MJScene::equationsOfMotion;
%ignore MJScene::preIntegration;
%ignore MJScene::postIntegration;
%ignore MJScene::markAsNeedingToRecompileModel;
%ignore MJScene::areKinematicsStale;
%ignore MJScene::markKinematicsAsStale;
%ignore MJScene::writeFwdKinematicsMessages;
%ignore MJScene::getMujocoModel;
%ignore MJScene::getMujocoData;
%ignore MJScene::areMujocoModelConstStale;
%ignore MJScene::markMujocoModelConstAsStale;


%extend MJScene
{
%pythoncode %{
    @classmethod
    def fromFile(cls, filePath: str, files: List[str] = []):
        with open(filePath, "r") as f:
            return cls(f.read(), files)
%}
}

// Included base interfaces reset %exception. MJScene::Reset must translate
// director and registration failures instead of allowing them to leave Python.
MUJOCO_BSK_EXCEPTION_POLICY

%include "MJScene.h"
%template(MJGeomInfoVector) std::vector<MJGeomInfo>;

%pythoncode %{
MJScene.highOrderAttitudeIntegration = property(
    MJScene.getHighOrderAttitudeIntegration,
    MJScene.setHighOrderAttitudeIntegration,
)

# simIncludeGravBody retains its task model here. Declare the slot before the
# inherited protected-attribute policy is applied to instances.
MJScene._gravBodyFactoryNBodyGravity = None
MJScene._vizGravBodies = None
%}

%include "architecture/msgPayloadDefCpp/MJSceneStateMsgPayload.h"
