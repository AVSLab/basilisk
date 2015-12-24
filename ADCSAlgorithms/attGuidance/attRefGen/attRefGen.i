%module attRefGen
%{
   #include "attRefGen.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_attRefGen(void*, uint64_t, uint64_t);
%ignore Update_attRefGen;
%constant void SelfInit_attRefGen(void*, uint64_t);
%ignore SelfInit_attRefGen;
%constant void CrossInit_attRefGen(void*, uint64_t);
%ignore CrossInit_attRefGen;
%array_functions(SingleCSSConfig, CSSWlsConfigArray);
%include "../_GeneralModuleFiles/attGuidOut.h"
%include "attRefGen.h"
