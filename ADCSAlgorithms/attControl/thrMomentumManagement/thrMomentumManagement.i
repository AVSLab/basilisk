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
%module thrMomentumManagement
%{
   #include "thrMomentumManagement.h"
%}

%include "swig_conly_data.i"
%constant void Update_thrMomentumManagement(void*, uint64_t, uint64_t);
%ignore Update_thrMomentumManagement;
%constant void SelfInit_thrMomentumManagement(void*, uint64_t);
%ignore SelfInit_thrMomentumManagement;
%constant void CrossInit_thrMomentumManagement(void*, uint64_t);
%ignore CrossInit_thrMomentumManagement;
%constant void Reset_thrMomentumManagement(void*, uint64_t, uint64_t);
%ignore Reset_thrMomentumManagement;
GEN_SIZEOF(thrMomentumManagementConfig);
GEN_SIZEOF(VehicleConfigMessage);
GEN_SIZEOF(RWConfigMessage);
%include "thrMomentumManagement.h"
%include "../../fswMessages/vehicleConfigMessage.h"
%include "../../fswMessages/rwConfigMessage.h"
%include "../../fswMessages/cmdDelHMessage.h"
GEN_SIZEOF(CmdDelHMessage);
%include "../../../SimFswInterface/rwSpeedMessage.h"
GEN_SIZEOF(RWSpeedMessage);

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
