%module EMM_FSW_Autocode
%{
	#include "EMM_FSW_Autocode.h"
%}

%include "swig_conly_data.i"

%constant void DataInit(void*, unit64_t);
%ignore DataInit(void*, unit64_t);
%constant void AllAlg_SelfInit(void*, unit64_t);
%ignore AllAlg_SelfInit(void*, unit64_t);
%constant void AllAlg_CrossInit(void*, unit64_t);
%ignore AllAlg_CrossInit(void*, unit64_t);
%constant void AllAlg_Reset(void*, unit64_t, uint64_t);
%ignore AllAlg_Reset(void*, unit64_t, uint64_t);
%constant void AllTasks_Update(void*, unit64_t, uint64_t);
%ignore AllTasks_Update(void*, unit64_t, uint64_t);
%include "EMM_FSW_Autocode.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}