%module singleAxisRot
%{
   #include "singleAxisRot.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_singleAxisRot(void*, uint64_t, uint64_t);
%ignore Update_singleAxisRot;
%constant void SelfInit_singleAxisRot(void*, uint64_t);
%ignore SelfInit_singleAxisRot;
%constant void CrossInit_singleAxisRot(void*, uint64_t);
%ignore CrossInit_singleAxisRot;
%include "singleAxisRot.h"
