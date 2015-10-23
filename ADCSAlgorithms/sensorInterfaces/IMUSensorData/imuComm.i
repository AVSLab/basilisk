%module imuComm
%{
   #include "imuComm.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_imuProcessTelem(void*, uint64_t, uint64_t);
%ignore Update_imuProcessTelem;
%constant void SelfInit_imuProcessTelem(void*, uint64_t);
%ignore SelfInit_imuProcessTelem;
%constant void CrossInit_imuProcessTelem(void*, uint64_t);
%ignore CrossInit_imuProcessTelem;
%include "imuComm.h"
