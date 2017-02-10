/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
%module thrForceMapping
%{
   #include "thrForceMapping.h"
%}

%include "swig_conly_data.i"
%constant void Update_thrForceMapping(void*, uint64_t, uint64_t);
%ignore Update_thrForceMapping;
%constant void SelfInit_thrForceMapping(void*, uint64_t);
%ignore SelfInit_thrForceMapping;
%constant void CrossInit_thrForceMapping(void*, uint64_t);
%ignore CrossInit_thrForceMapping;
%constant void Reset_thrForceMapping(void*, uint64_t, uint64_t);
%ignore Reset_thrForceMapping;

%include "thrForceMapping.h"
GEN_SIZEOF(thrForceMappingConfig);
%include "../../fswMessages/vehicleConfigMessage.h"
GEN_SIZEOF(VehicleConfigMessage);
%include "../../fswMessages/thrArrayMessage.h"
GEN_SIZEOF(THRArrayMessage);
%include "../../fswMessages/THRArrayCmdForceMessage.h"
%include "../../../SimFswInterface/cmdTorqueBodyMessage.h"
GEN_SIZEOF(CmdTorqueBodyMessage);
%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
