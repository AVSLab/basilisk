%module PRV_Steering
%{
   #include "PRV_Steering.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_PRV_Steering(void*, uint64_t, uint64_t);
%ignore Update_PRV_Steering;
%constant void SelfInit_PRV_Steering(void*, uint64_t);
%ignore SelfInit_PRV_Steering;
%constant void CrossInit_PRV_Steering(void*, uint64_t);
%ignore CrossInit_PRV_Steering;
%constant void Reset_PRV_Steering(void*);
%ignore Reset_PRV_Steering;
%include "../_GeneralModuleFiles/vehControlOut.h"
%include "PRV_Steering.h"
