%module hillPoint
%{
   #include "hillPoint.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_hillPoint(void*, uint64_t, uint64_t);
%ignore Update_hillPoint;
%constant void SelfInit_hillPoint(void*, uint64_t);
%ignore SelfInit_hillPoint;
%constant void CrossInit_hillPoint(void*, uint64_t);
%ignore CrossInit_hillPoint;
%constant void Reset_hillPoint(void*);
%ignore Reset_hillPoint;
%include "hillPoint.h"

// sample Module supportfile to be included in this sub-module
%include "../_GeneralModuleFiles/attGuidOut.h"