%module LowPassFilterTorqueCommand
%{
   #include "LowPassFilterTorqueCommand.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_LowPassFilterTorqueCommand(void*, uint64_t, uint64_t);
%ignore Update_LowPassFilterTorqueCommand;
%constant void SelfInit_LowPassFilterTorqueCommand(void*, uint64_t);
%ignore SelfInit_LowPassFilterTorqueCommand;
%constant void CrossInit_LowPassFilterTorqueCommand(void*, uint64_t);
%ignore CrossInit_LowPassFilterTorqueCommand;
%constant void Reset_LowPassFilterTorqueCommand(void*);
%ignore Reset_LowPassFilterTorqueCommand;
%include "LowPassFilterTorqueCommand.h"

// sample Module supportfile to be included in this sub-module
//%include "../_GeneralModuleFiles/dummy.h"