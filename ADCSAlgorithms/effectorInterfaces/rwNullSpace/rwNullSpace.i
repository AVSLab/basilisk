%module rwNullSpace
%{
   #include "rwNullSpace.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_rwNullSpace(void*, uint64_t, uint64_t);
%ignore Update_rwNullSpace;
%constant void SelfInit_rwNullSpace(void*, uint64_t);
%ignore SelfInit_rwNullSpace;
%constant void CrossInit_rwNullSpace(void*, uint64_t);
%ignore CrossInit_rwNullSpace;
%include "rwNullSpace.h"
%include "../_GeneralModuleFiles/rwSpeedData.h"
