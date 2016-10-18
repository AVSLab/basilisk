%module attitude_ukf
%{
   #include "attitude_ukf.h"
%}

%include "swig_common_model.i"

%include "sys_model.h"
%include "../_GeneralModuleFiles/unscent_kalfilt.h"
%include "../_GeneralModuleFiles/navStateOut.h"
GEN_SIZEOF(RWConfigElement);
%include "attitude_ukf.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}


