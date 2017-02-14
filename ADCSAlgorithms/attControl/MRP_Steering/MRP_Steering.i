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
%module MRP_Steering
%{
   #include "MRP_Steering.h"
%}

%include "swig_conly_data.i"
%constant void Update_MRP_Steering(void*, uint64_t, uint64_t);
%ignore Update_MRP_Steering;
%constant void SelfInit_MRP_Steering(void*, uint64_t);
%ignore SelfInit_MRP_Steering;
%constant void CrossInit_MRP_Steering(void*, uint64_t);
%ignore CrossInit_MRP_Steering;
%constant void Reset_MRP_Steering(void*, uint64_t, uint64_t);
%ignore Reset_MRP_Steering;
ARRAYASLIST(FSWdeviceAvailability)
GEN_SIZEOF(MRP_SteeringConfig);
GEN_SIZEOF(RWAvailabilityFswMsg);
GEN_SIZEOF(AttGuidMessage);
GEN_SIZEOF(VehicleConfigMessage);
GEN_SIZEOF(RWConfigMessage);
GEN_SIZEOF(RWSpeedMessage);
%include "MRP_Steering.h"
%include "../../fswMessages/rwAvailabilityFswMsg.h"
%include "../../fswUtilities/fswDefinitions.h"
%include "../../fswMessages/attGuidMessage.h"
%include "../../fswMessages/vehicleConfigMessage.h"
%include "../../fswMessages/rwArrayConfigMessage.h"
%include "../../../SimFswInterfaceMessages/rwSpeedMessage.h"
%include "../../../SimFswInterfaceMessages/cmdTorqueBodyMessage.h"
GEN_SIZEOF(CmdTorqueBodyMessage);

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
