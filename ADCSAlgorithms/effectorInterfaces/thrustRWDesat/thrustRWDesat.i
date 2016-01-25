%module thrustRWDesat
%{
   #include "thrustRWDesat.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_thrustRWDesat(void*, uint64_t, uint64_t);
%ignore Update_thrustRWDesat;
%constant void SelfInit_thrustRWDesat(void*, uint64_t);
%ignore SelfInit_thrustRWDesat;
%constant void CrossInit_thrustRWDesat(void*, uint64_t);
%ignore CrossInit_thrustRWDesat;
%constant void Reset_thrustRWDesat(void*, uint64_t);
%ignore Reset_thrustRWDesat;
%include "vehEffectorOut.h"
%include "thrustRWDesat.h"
