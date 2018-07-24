/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */
%module AVS_FSW_Autocode
%{
	#include "AVS_FSW_Autocode.h"
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
%include "AVS_FSW_Autocode.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}