%module MRP_Feedback
%{
   #include "MRP_Feedback.h"
%}

%include "carrays.i"
%include "stdint.i"
%constant void Update_MRP_Feedback(void*, uint64_t, uint64_t);
%ignore Update_MRP_Feedback;
%constant void SelfInit_MRP_Feedback(void*, uint64_t);
%ignore SelfInit_MRP_Feedback;
%constant void CrossInit_MRP_Feedback(void*, uint64_t);
%ignore CrossInit_MRP_Feedback;
%constant void Reset_MRP_Feedback(void*);
%ignore Reset_MRP_Feedback;
%include "../_GeneralModuleFiles/vehControlOut.h"
%include "MRP_Feedback.h"
