%module inertial3DSpin
%{
   #include "inertial3DSpin.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_inertial3DSpin(void*, uint64_t, uint64_t);
%ignore Update_inertial3DSpin;
%constant void SelfInit_inertial3DSpin(void*, uint64_t);
%ignore SelfInit_inertial3DSpin;
%constant void CrossInit_inertial3DSpin(void*, uint64_t);
%ignore CrossInit_inertial3DSpin;
%constant void Reset_inertial3DSpin(void*);
%ignore Reset_inertial3DSpin;
%include "inertial3DSpin.h"

// sample Module supportfile to be included in this sub-module
%include "../_GeneralModuleFiles/attGuidOut.h"