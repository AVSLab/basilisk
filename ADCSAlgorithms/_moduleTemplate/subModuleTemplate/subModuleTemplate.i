%module subModuleTemplate
%{
   #include "subModuleTemplate.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_subModuleTemplate(void*, uint64_t, uint64_t);
%ignore Update_subModuleTemplate;
%constant void SelfInit_subModuleTemplate(void*, uint64_t);
%ignore SelfInit_subModuleTemplate;
%constant void CrossInit_subModuleTemplate(void*, uint64_t);
%ignore CrossInit_subModuleTemplate;
%constant void Reset_subModuleTemplate(void*);
%ignore Reset_subModuleTemplate;
%include "subModuleTemplate.h"

// sample Module supportfile to be included in this sub-module
%include "../_GeneralModuleFiles/subModuleOut.h"