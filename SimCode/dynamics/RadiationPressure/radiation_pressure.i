%module radiation_pressure
%{
   #include "radiation_pressure.h"
%}

%include "swig_common_model.i"

%include "../_GeneralModuleFiles/dynamicEffector.h"
%include "../_GeneralModuleFiles/stateData.h"
%include "sys_model.h"
%include "radiation_pressure.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
