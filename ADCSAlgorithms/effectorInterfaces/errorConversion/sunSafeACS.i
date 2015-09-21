%module sunSafeACS
%{
   #include "sunSafeACS.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_sunSafeACS(void*, uint64_t);
%ignore Update_sunSafeACS;
%constant void SelfInit_sunSafeACS(void*);
%ignore SelfInit_sunSafeACS;
%constant void CrossInit_sunSafeACS(void*);
%ignore CrossInit_sunSafeACS;
%include "vehEffectorOut.h"
%include "dvAttEffect.h"
%include "sunSafeACS.h"
