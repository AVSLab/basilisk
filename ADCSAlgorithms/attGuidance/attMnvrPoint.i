%module attMnvrPoint
%{
   #include "attMnvrPoint.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_attMnvrPoint(void*, uint64_t, uint64_t);
%ignore Update_attMnvrPoint;
%constant void SelfInit_attMnvrPoint(void*, uint64_t);
%ignore SelfInit_attMnvrPoint;
%constant void CrossInit_attMnvrPoint(void*, uint64_t);
%ignore CrossInit_attMnvrPoint;
%array_functions(SingleCSSConfig, CSSWlsConfigArray);
%include "attGuidOut.h"
%include "attMnvrPoint.h"
