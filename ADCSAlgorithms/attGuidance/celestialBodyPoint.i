%module celestialBodyPoint
%{
   #include "celestialBodyPoint.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_celestialBodyPoint(void*, uint64_t);
%ignore Update_celestialBodyPoint;
%constant void SelfInit_celestialBodyPoint(void*);
%ignore SelfInit_celestialBodyPoint;
%constant void CrossInit_celestialBodyPoint(void*);
%ignore CrossInit_celestialBodyPoint;
%include "celestialBodyPoint.h"
