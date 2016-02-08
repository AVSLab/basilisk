%module velocityPoint
%{
   #include "velocityPoint.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_velocityPoint(void*, uint64_t, uint64_t);
%ignore Update_velocityPoint;
%constant void SelfInit_velocityPoint(void*, uint64_t);
%ignore SelfInit_velocityPoint;
%constant void CrossInit_velocityPoint(void*, uint64_t);
%ignore CrossInit_velocityPoint;
%constant void Reset_velocityPoint(void*);
%ignore Reset_velocityPoint;
%include "velocityPoint.h"

// sample Module supportfile to be included in this sub-module
%include "../_GeneralModuleFiles/attGuidOut.h"