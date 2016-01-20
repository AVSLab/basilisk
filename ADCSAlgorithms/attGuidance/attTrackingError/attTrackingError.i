%module attTrackingError
%{
   #include "attTrackingError.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_attTrackingError(void*, uint64_t, uint64_t);
%ignore Update_attTrackingError;
%constant void SelfInit_attTrackingError(void*, uint64_t);
%ignore SelfInit_attTrackingError;
%constant void CrossInit_attTrackingError(void*, uint64_t);
%ignore CrossInit_attTrackingError;
%constant void Reset_attTrackingError(void*);
%ignore Reset_attTrackingError;
%include "attTrackingError.h"

// sample Module supportfile to be included in this sub-module
%include "../_GeneralModuleFiles/attGuidOut.h"