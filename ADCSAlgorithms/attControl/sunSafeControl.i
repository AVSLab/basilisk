%module sunSafeControl
%{
   #include "sunSafeControl.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_sunSafeControl(void*, uint64_t, uint64_t);
%ignore Update_sunSafeControl;
%constant void SelfInit_sunSafeControl(void*, uint64_t);
%ignore SelfInit_sunSafeControl;
%constant void CrossInit_sunSafeControl(void*, uint64_t);
%ignore CrossInit_sunSafeControl;
%include "vehControlOut.h"
%include "sunSafeControl.h"
