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
%module rwMotorVoltage
%{
   #include "rwMotorVoltage.h"
%}

%include "swig_conly_data.i"
%constant void Update_rwMotorVoltage(void*, uint64_t, uint64_t);
%ignore Update_rwMotorVoltage;
%constant void SelfInit_rwMotorVoltage(void*, uint64_t);
%ignore SelfInit_rwMotorVoltage;
%constant void CrossInit_rwMotorVoltage(void*, uint64_t);
%ignore CrossInit_rwMotorVoltage;
%constant void Reset_rwMotorVoltage(void*, uint64_t, uint64_t);
%ignore Reset_rwMotorVoltage;
ARRAYASLIST(FSWdeviceAvailability)
GEN_SIZEOF(rwMotorVoltageConfig);
GEN_SIZEOF(RWArrayTorqueIntMsg);
GEN_SIZEOF(RWAvailabilityFswMsg);
GEN_SIZEOF(RWSpeedIntMsg);
GEN_SIZEOF(RWArrayConfigFswMsg);
%include "rwMotorVoltage.h"
%include "simFswInterfaceMessages/rwSpeedIntMsg.h"
%include "simFswInterfaceMessages/macroDefinitions.h"
%include "simFswInterfaceMessages/rwArrayTorqueIntMsg.h"
%include "simFswInterfaceMessages/rwArrayVoltageIntMsg.h"
%include "../../fswMessages/rwAvailabilityFswMsg.h"
%include "../../fswUtilities/fswDefinitions.h"
%include "../../fswMessages/rwArrayConfigFswMsg.h"
%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
