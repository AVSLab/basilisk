%module celestialBodyPoint
%{
   #include "celestialBodyPoint.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_celestialBodyPoint(void*, uint64_t, uint64_t);
%ignore Update_celestialBodyPoint;
%constant void SelfInit_celestialBodyPoint(void*, uint64_t);
%ignore SelfInit_celestialBodyPoint;
%constant void CrossInit_celestialBodyPoint(void*, uint64_t);
%ignore CrossInit_celestialBodyPoint;
%include "../_GeneralModuleFiles/attGuidOut.h"
%include "celestialBodyPoint.h"
