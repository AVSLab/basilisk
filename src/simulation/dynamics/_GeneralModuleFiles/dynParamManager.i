
%module dynParamManager
%{
   #include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
   #include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
%}

// There are other accesor methods to query these objects that are
// better than using the class variables directly
%ignore StateVector::stateMap;
%ignore DynParamManager::dynProperties;

// Uses unique_ptr, don't need it at the Python level
%ignore StateData::clone;

%include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
%include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"

// Used in testing in Python, we use the default template arguments
%template(registerState) DynParamManager::registerState<StateData, true>;
