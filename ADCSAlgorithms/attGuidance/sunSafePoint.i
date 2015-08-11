%module sunSafePoint
%{
   #include "sunSafePoint.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_sunSafePoint(void*, uint64_t);
%ignore Update_sunSafePoint;
%constant void SelfInit_sunSafePoint(void*);
%ignore SelfInit_sunSafePoint;
%constant void CrossInit_sunSafePoint(void*);
%ignore CrossInit_sunSafePoint;
%array_functions(SingleCSSConfig, CSSWlsConfigArray);
%include "attGuidOut.h"
%include "sunSafePoint.h"
