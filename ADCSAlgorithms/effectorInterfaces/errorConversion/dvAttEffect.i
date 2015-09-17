%module dvAttEffect
%{
   #include "dvAttEffect.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_dvAttEffect(void*, uint64_t);
%ignore Update_dvAttEffect;
%constant void SelfInit_dvAttEffect(void*);
%ignore SelfInit_dvAttEffect;
%constant void CrossInit_dvAttEffect(void*);
%ignore CrossInit_dvAttEffect;
%array_functions(ThrustGroupData, ThrustGroupArray);
%include "vehEffectorOut.h"
%include "dvAttEffect.h"
