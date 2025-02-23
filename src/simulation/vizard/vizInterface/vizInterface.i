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
%{
   #include "vizInterface.h"
   #include "simulation/vizard/_GeneralModuleFiles/vizStructures.h"
%}

%include "swig_deprecated.i"
%deprecated_variable(VizInterface, opNavMode, "2025/04/17", "opNavMode is deprecated. Use liveStream and noDisplay flags instead.")

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
import warnings

mod = sys.modules[__name__]

class _DeprecatedWrapper:
    def __init__(self, target, aliasName=None, targetName=None, deprecatedFields=None, removalDate=None):
        self._target = target
        self._aliasName = aliasName
        self._targetName = targetName
        self._deprecatedFields = deprecatedFields or {}
        self._removalDate = removalDate

    def __call__(self, *args, **kwargs):
        """Handles alias deprecation and returns an instance of the target class."""
        if self._aliasName:
            warnings.warn(
                f"'{self._aliasName}' is deprecated and will be removed after {self._removalDate}. "
                f"Use '{self._targetName}' instead.",
                DeprecationWarning,
                stacklevel=2
            )

        instance = self._target(*args, **kwargs)

        # Inject deprecation warnings dynamically without changing the instance type
        for old_attr, new_attr in self._deprecatedFields.items():
            if hasattr(instance, new_attr):  # Ensure new attribute exists
                _inject_deprecated_property(instance, old_attr, new_attr, self._removalDate)

        return instance  # Always return the original instance

def _inject_deprecated_property(instance, old_attr, new_attr, removal_date):
    """Dynamically injects a deprecated property that redirects to the new attribute."""
    def getter(self):
        warnings.warn(
            f"'{old_attr}' is deprecated and will be removed after {removal_date}. "
            f"Use '{new_attr}' instead.",
            DeprecationWarning,
            stacklevel=2
        )
        return getattr(self, new_attr)

    def setter(self, value):
        warnings.warn(
            f"Setting '{old_attr}' is deprecated and will be removed after {removal_date}. "
            f"Use '{new_attr}' instead.",
            DeprecationWarning,
            stacklevel=2
        )
        if new_attr == "dimensions":
            setattr(self, new_attr, [value, value, value])
        else:
            setattr(self, new_attr, value)

    # Inject property into the class itself (not instance-specific)
    setattr(instance.__class__, old_attr, property(getter, setter))

# ------ Deprecated variable/structure list ------ #
# Remove from here when support is expired.

# Since dimensions is a 3-element list while radius is a scalar, there is an extra if/else conversion above which will
# need to be removed as well.
mod.MultiShape = _DeprecatedWrapper(
    mod.MultiShape,
    targetName="MultiShape",
    deprecatedFields={"radius": "dimensions"},
    removalDate="02/21/26"
)
mod.MultiSphere = _DeprecatedWrapper(
    mod.MultiShape,
    aliasName="MultiSphere",
    targetName="MultiShape",
    removalDate="02/21/26"
)
mod.MultiSphereInfo = _DeprecatedWrapper(
    mod.MultiShapeInfo,
    aliasName="MultiSphereInfo",
    targetName="MultiShapeInfo",
    removalDate="02/21/26"
)
mod.MultiSphereVector = _DeprecatedWrapper(
    mod.MultiShapeVector,
    aliasName="MultiSphereVector",
    targetName="MultiShapeVector",
    removalDate="02/21/26"
)

warnings.simplefilter("default", DeprecationWarning)

protectAllClasses(sys.modules[__name__])
%}
