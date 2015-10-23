%module cssWlsEst
%{
   #include "cssWlsEst.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_cssWlsEst(void*, uint64_t, uint64_t);
%ignore Update_cssWlsEst;
%constant void SelfInit_cssWlsEst(void*, uint64_t);
%ignore SelfInit_cssWlsEst;
%constant void CrossInit_cssWlsEst(void*, uint64_t);
%ignore CrossInit_cssWlsEst;
%array_functions(SingleCSSConfig, CSSWlsConfigArray);
%include "cssWlsEst.h"
