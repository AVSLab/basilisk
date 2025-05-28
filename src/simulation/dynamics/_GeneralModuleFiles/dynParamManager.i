
%module(package="Basilisk.simulation") dynParamManager

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
   #include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
%}

%include "std_pair.i"
%include "std_vector.i"
%include "std_string.i"
%include "swig_eigen.i"

// Suppress assignment operator warning before parsing the class
%warnfilter(362) StateVector::operator=;

// There are other accesor methods to query these objects that are
// better than using the class variables directly
%ignore StateVector::stateMap;
%ignore DynParamManager::dynProperties;

// Uses unique_ptr, don't need it at the Python level
%ignore StateData::clone;

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
         refList.emplace_back(*p.first, p.second);
      }
      $self->registerSharedNoiseSource(refList);
   }
}

%ignore DynParamManager::registerSharedNoiseSource(std::vector<std::pair<const StateData&, size_t>>);

%include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"

// Used in testing in Python, we use the default template arguments
%template(registerState) DynParamManager::registerState<StateData, true>;
