%module MRP_Steering
%{
   #include "MRP_Steering.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_MRP_Steering(void*, uint64_t, uint64_t);
%ignore Update_MRP_Steering;
%constant void SelfInit_MRP_Steering(void*, uint64_t);
%ignore SelfInit_MRP_Steering;
%constant void CrossInit_MRP_Steering(void*, uint64_t);
%ignore CrossInit_MRP_Steering;
%constant void Reset_MRP_Steering(void*);
%ignore Reset_MRP_Steering;
%include "../_GeneralModuleFiles/vehControlOut.h"
%include "MRP_Steering.h"
