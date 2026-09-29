
%module(package="Basilisk.simulation") dynParamManager

// BSK_SWIG_RUNTIME_DEPENDS: swig_common_model

%include "architecture/utilities/bskException.swg"
%default_bsk_exception(catch (const std::exception& error) {
    SWIG_exception(SWIG_RuntimeError, error.what());
});

%{
   #include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
   #include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
%}

%include "stdint.i"
%include "std_pair.i"
%include "std_vector.i"
%include "std_string.i"
%include "swig_eigen.i"

// swig_eigen.i includes interfaces that reset %exception.
%default_bsk_exception(catch (const std::exception& error) {
    SWIG_exception(SWIG_RuntimeError, error.what());
});

// There are other accesor methods to query these objects that are
// better than using the class variables directly
%ignore DynParamManager::dynProperties;

// Internal bookkeeping not intended for Python use.
%ignore DynParamManager::bskLogger;
%ignore StateUpdatePolicy;
%ignore StateRegistry;
%ignore DynParamManager::getStateRegistry;

// Uses unique_ptr, don't need it at the Python level
%ignore DynParamManager::registerState(
    std::string,
    const StateSpec&,
    std::unique_ptr<StateUpdatePolicy>);

// Comparison operators are C++ topology helpers.
%ignore MatrixShape::operator==;
%ignore MatrixShape::operator!=;
%ignore StateSpec::operator==;
%ignore StateSpec::operator!=;

// These accessors expose internal Eigen storage and are intended for C++ use only.
%ignore StateData::stateView;
%ignore StateData::derivativeView;
%ignore StateData::diffusionView;
%ignore StateData::~StateData;
%ignore StateData::getStateReference;
%ignore StateData::getStateDerivReference;
%ignore StateData::stateData;
%ignore StateData::derivativeData;
%ignore StateData::diffusionData;
%ignore StateUpdatePolicy::buildDriftCandidate;
%ignore StateUpdatePolicy::applyNoiseIncrement;
%ignore StateData::setState(Eigen::Ref<const Eigen::MatrixXd>);
%ignore StateData::setDerivative(Eigen::Ref<const Eigen::MatrixXd>);
%ignore StateData::setDiffusion(Eigen::Ref<const Eigen::MatrixXd>, size_t);

// Keep Python's MatrixXd conversion at the wrapper boundary while the native
// API accepts Eigen::Ref and therefore does not materialize Eigen::Map inputs.
%extend StateData {
    void setState(const Eigen::MatrixXd& value) {
        $self->setState(value);
    }
    void setDerivative(const Eigen::MatrixXd& value) {
        $self->setDerivative(value);
    }
    void setDiffusion(const Eigen::MatrixXd& value, size_t index) {
        $self->setDiffusion(value, index);
    }
}

%include "simulation/dynamics/_GeneralModuleFiles/stateData.h"

// Current limitation of SWIG for complex templated types like
// std::pair<const StateData*, size_t>, we need to declare these manually
%traits_swigtype(StateData);
%fragment(SWIG_Traits_frag(StateData));

%template() std::pair<const StateData*, size_t>;
%template() std::vector<std::pair<const StateData*, size_t>>;

%extend DynParamManager {
   // SWIG doesnt like const StateData& so we have to use const StateData* and convert
   void registerSharedNoiseSource(std::vector<std::pair<const StateData*, size_t>> list) {
      // Convert from pointer pairs to reference pairs
      std::vector<std::pair<const StateData&, size_t>> refList;
      refList.reserve(list.size());
      for (const auto& p : list) {
         if (p.first == nullptr) {
            throw BasiliskError("registerSharedNoiseSource received a null StateData pointer.");
         }
         refList.emplace_back(*p.first, p.second);
      }
      $self->registerSharedNoiseSource(refList);
   }
}

%ignore DynParamManager::registerSharedNoiseSource(std::vector<std::pair<const StateData&, size_t>>);

// Keep the established keyword signature independent of the new specification API.
%rename(registerStateSpec) DynParamManager::registerState(std::string, const StateSpec&);

// States borrow their manager. A manager embedded in a dynamics object is itself
// borrowed: callers must keep that dynamics object alive.
%pythonappend DynParamManager::registerState %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend DynParamManager::getStateObject %{
    if val is not None:
        val._swig_bsk_owner = self
%}

%include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"

%pythoncode %{
from Basilisk.architecture import swig_common_model as _swig_common_model

# Compatibility aliases for the former public StateData members.
StateData.state = property(StateData.getState, StateData.setState)
StateData.stateDeriv = property(
    StateData.getStateDeriv, StateData.setDerivative
)
StateData.stateName = property(StateData.getName)
StateData.perComponentErrorControl = property(
    StateData.usesPerComponentErrorControl
)

def _state_diffusions(state):
    return _swig_common_model.FixedSizeSequence(
        state,
        state.getNumNoiseSources,
        state.getStateDiffusion,
        lambda index, value: state.setDiffusion(value, index),
    )

def _replace_state_diffusions(state, values):
    _state_diffusions(state).replace(values)

StateData.stateDiffusion = property(
    _state_diffusions, _replace_state_diffusions
)

_COMPATIBILITY_EXPORTS = (
    "DynParamManager",
    "StateData",
    "MatrixShape",
    "StateSpec",
    "ErrorControlMode_WholeState",
    "ErrorControlMode_PerComponent",
    "StateUpdateKind_Euclidean",
    "StateUpdateKind_Special",
)

def _exportCompatibilityAPI(namespace):
    for name in _COMPATIBILITY_EXPORTS:
        namespace[name] = globals()[name]

import sys as _sys
_swig_common_model.protectAllClasses(_sys.modules[__name__])
del _sys
%}
