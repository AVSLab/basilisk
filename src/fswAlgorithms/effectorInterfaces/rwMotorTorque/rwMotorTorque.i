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
%module rwMotorTorque
%{
   #include "rwMotorTorque.h"
%}

%include "swig_conly_data.i"
%constant void Update_rwMotorTorque(void*, uint64_t, uint64_t);
%ignore Update_rwMotorTorque;
%constant void SelfInit_rwMotorTorque(void*, uint64_t);
%ignore SelfInit_rwMotorTorque;
%constant void CrossInit_rwMotorTorque(void*, uint64_t);
%ignore CrossInit_rwMotorTorque;
%constant void Reset_rwMotorTorque(void*, uint64_t, uint64_t);
%ignore Reset_rwMotorTorque;
ARRAYASLIST(FSWdeviceAvailability)
GEN_SIZEOF(rwMotorTorqueConfig);
GEN_SIZEOF(RWAvailabilityFswMsg);
GEN_SIZEOF(RWArrayConfigFswMsg);
GEN_SIZEOF(RWArrayTorqueIntMsg);
GEN_SIZEOF(CmdTorqueBodyIntMsg);
%include "rwMotorTorque.h"
%include "simFswInterfaceMessages/rwSpeedIntMsg.h"
%include "simFswInterfaceMessages/cmdTorqueBodyIntMsg.h"
%include "simFswInterfaceMessages/rwArrayTorqueIntMsg.h"
%include "../../fswMessages/rwAvailabilityFswMsg.h"
%include "../../fswUtilities/fswDefinitions.h"
%include "../../fswMessages/rwArrayConfigFswMsg.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
