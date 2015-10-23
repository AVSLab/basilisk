%module dvGuidance
%{
   #include "dvGuidance.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_dvGuidance(void*, uint64_t, uint64_t);
%ignore Update_dvGuidance;
%constant void SelfInit_dvGuidance(void*, uint64_t);
%ignore SelfInit_dvGuidance;
%constant void CrossInit_dvGuidance(void*, uint64_t);
%ignore CrossInit_dvGuidance;
%include "dvGuidance.h"
