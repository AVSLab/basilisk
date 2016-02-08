%module orbitAxisSpin
%{
   #include "orbitAxisSpin.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_orbitAxisSpin(void*, uint64_t, uint64_t);
%ignore Update_orbitAxisSpin;
%constant void SelfInit_orbitAxisSpin(void*, uint64_t);
%ignore SelfInit_orbitAxisSpin;
%constant void CrossInit_orbitAxisSpin(void*, uint64_t);
%ignore CrossInit_orbitAxisSpin;
%constant void Reset_orbitAxisSpin(void*);
%ignore Reset_orbitAxisSpin;
%include "orbitAxisSpin.h"

// supportfile to be included in this sub-module
%include "../_GeneralModuleFiles/attGuidOut.h"