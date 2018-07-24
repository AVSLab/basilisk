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
%module PRV_Steering
%{
   #include "PRV_Steering.h"
%}

%include "swig_conly_data.i"
%constant void Update_PRV_Steering(void*, uint64_t, uint64_t);
%ignore Update_PRV_Steering;
%constant void SelfInit_PRV_Steering(void*, uint64_t);
%ignore SelfInit_PRV_Steering;
%constant void CrossInit_PRV_Steering(void*, uint64_t);
%ignore CrossInit_PRV_Steering;
%constant void Reset_PRV_Steering(void*, uint64_t, uint64_t);
%ignore Reset_PRV_Steering;
%include "PRV_Steering.h"
%include "../../fswMessages/attGuidFswMsg.h"
%include "../../fswMessages/rateCmdFswMsg.h"
GEN_SIZEOF(PRV_SteeringConfig);
GEN_SIZEOF(AttGuidFswMsg);
GEN_SIZEOF(RateCmdFswMsg);

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

