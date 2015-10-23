%module cssComm
%{
   #include "cssComm.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_cssProcessTelem(void*, uint64_t, uint64_t);
%ignore Update_cssProcessTelem;
%constant void SelfInit_cssProcessTelem(void*, uint64_t);
%ignore SelfInit_cssProcessTelem;
%constant void CrossInit_cssProcessTelem(void*, uint64_t);
%ignore CrossInit_cssProcessTelem;
%array_functions(SensorMsgNameCarrier, SensorNameArray);
%include "cssComm.h"
