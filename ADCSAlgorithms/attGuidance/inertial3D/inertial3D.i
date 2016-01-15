%module inertial3D
%{
   #include "inertial3D.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_inertial3D(void*, uint64_t, uint64_t);
%ignore Update_inertial3D;
%constant void SelfInit_inertial3D(void*, uint64_t);
%ignore SelfInit_inertial3D;
%constant void CrossInit_inertial3D(void*, uint64_t);
%ignore CrossInit_inertial3D;
%constant void Reset_inertial3D(void*);
%ignore Reset_inertial3D;
%include "inertial3D.h"

// sample Module supportfile to be included in this sub-module
%include "../_GeneralModuleFiles/attGuidOut.h"