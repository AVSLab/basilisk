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
%module MRP_Feedback
%{
   #include "MRP_Feedback.h"
%}

%include "swig_conly_data.i"
%constant void Update_MRP_Feedback(void*, uint64_t, uint64_t);
%ignore Update_MRP_Feedback;
%constant void SelfInit_MRP_Feedback(void*, uint64_t);
%ignore SelfInit_MRP_Feedback;
%constant void CrossInit_MRP_Feedback(void*, uint64_t);
%ignore CrossInit_MRP_Feedback;
%constant void Reset_MRP_Feedback(void*, uint64_t, uint64_t);
%ignore Reset_MRP_Feedback;
GEN_SIZEOF(MRP_FeedbackConfig);
GEN_SIZEOF(AttGuidFswMsg);
GEN_SIZEOF(VehicleConfigFswMsg);
GEN_SIZEOF(RWArrayConfigFswMsg);
GEN_SIZEOF(RWSpeedIntMsg);
%include "MRP_Feedback.h"
%include "../../fswMessages/attGuidFswMsg.h"
%include "../../fswMessages/vehicleConfigFswMsg.h"
%include "../../fswMessages/rwArrayConfigFswMsg.h"
%include "simFswInterfaceMessages/rwSpeedIntMsg.h"
%include "simFswInterfaceMessages/cmdTorqueBodyIntMsg.h"
GEN_SIZEOF(CmdTorqueBodyIntMsg);

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

