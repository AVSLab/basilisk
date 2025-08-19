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

%module vizInterface

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "vizInterface.h"
   #include "simulation/vizard/_GeneralModuleFiles/vizStructures.h"
%}

%include "swig_deprecated.i"

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"

%include "sys_model.i"
%include "std_vector.i"

// Instantiate templates used by example
namespace std {
    %template(PointLineConfig) vector<PointLine>;
    %template(LocationConfig) vector<LocationPbMsg *>;
    %template(CustomModelConfig) vector<CustomModel>;
    %template(ActuatorGuiSettingsConfig) vector<ActuatorGuiSettings>;
    %template(InstrumentGuiSettingsConfig) vector<InstrumentGuiSettings>;
    %template(KeepOutInConeConfig) vector<KeepOutInCone>;
    %template(StdCameraConfig) vector<StdCameraSettings>;
    %template(VizSCVector) vector<VizSpacecraftData>;
    %template(ThrClusterVector) vector<ThrClusterMap>;
    %template(GravBodyInfoVector) vector<GravBodyInfo>;
    %template(GenericSensorVector) vector<GenericSensor *>;
    %template(LightVector) vector<Light *>;
    %template(TransceiverVector) vector<Transceiver *>;
    %template(GenericStorageVector) vector<GenericStorage *>;
    %template(MultiShapeVector) vector<MultiShape *>;
    %template(EllipsoidVector) vector<Ellipsoid *>;
    %template(QuadMapVector) vector<QuadMap *>;
    %template(VizEventDialogVector) vector<VizEventDialog *>;
    %template(VizEventReplyVector) vector<VizEventReply>;
}

%include "vizInterface.h"
%include "simulation/vizard/_GeneralModuleFiles/vizStructures.h"

// Dan Padilha: Include the reactionWheelSupport to ensure that SWIG knows about the
// RWModels enum, and can correctly interpret it as an integer and destroy it
// without leaking memory. This needs to be imported here because of the way
// that Basilisk is built, which causes copies of types to be scattered across
// different modules. This means that instead of a model using the correct
// message type of `Basilisk.architecture.messaging.RWConfigLogMsgPayload`, they
// use `Basilisk.simulation.vizInterface.RWConfigLogMsgPayload` instead... :(
// TODO: We should clean up the SWIG build system so such issues don't occur.
%include "simulation/dynamics/reactionWheels/reactionWheelSupport.h"

%include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"
struct CameraConfigMsg_C;
%include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"
struct RWConfigLogMsg_C;
%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsg_C;
%include "architecture/msgPayloadDefC/CameraImageMsgPayload.h"
struct CameraImageMsg_C;
%include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
struct SpicePlanetStateMsg_C;
%include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
struct RWSpeedMsg_C;
%include "architecture/msgPayloadDefC/EpochMsgPayload.h"
struct EpochMsg_C;

%include "architecture/msgPayloadDefCpp/CSSConfigLogMsgPayload.h"
%include "architecture/msgPayloadDefCpp/THROutputMsgPayload.h"
%include "architecture/msgPayloadDefCpp/ChargeMsmMsgPayload.h"
%include "architecture/msgPayloadDefCpp/VizUserInputMsgPayload.h"

%pythoncode %{
import sys

mod = sys.modules[__name__]

# ------ Deprecated variable/structure list ------ #
# Remove from here when support is expired.
mod.MultiShape = _DeprecatedWrapper(
    mod.MultiShape,
    targetName="MultiShape",
    deprecatedFields={"radius": "dimensions"},
    typeConversion="scalarTo3D",
    removalDate="2026/03/07"
)

mod.MultiSphere = _DeprecatedWrapper(
    mod.MultiShape,
    aliasName="MultiSphere",
    targetName="MultiShape",
    removalDate="2026/03/07"
)
mod.MultiSphereInfo = _DeprecatedWrapper(
    mod.MultiShapeInfo,
    aliasName="MultiSphereInfo",
    targetName="MultiShapeInfo",
    removalDate="2026/03/07"
)
mod.MultiSphereVector = _DeprecatedWrapper(
    mod.MultiShapeVector,
    aliasName="MultiSphereVector",
    targetName="MultiShapeVector",
    removalDate="2026/03/07"
)

# when removing, also remove line 1649 that sets settings explicitly in src/utilities/vizSupport.py
mod.VizSettings = _DeprecatedWrapper(
    mod.VizSettings,
    targetName="VizSettings",
    deprecatedFields={"customGUIScale": "customGUIReferenceHeight"},
    typeConversion="useDefaultDouble",
    removalDate="2026/05/27"
)

protectAllClasses(sys.modules[__name__])
%}
