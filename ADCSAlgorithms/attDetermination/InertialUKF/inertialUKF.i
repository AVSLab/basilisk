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
%module inertialUKF
%{
   #include "inertialUKF.h"
   #include "../_GeneralModuleFiles/ukfUtilities.h"
%}

%include "swig_conly_data.i"
%constant void Update_inertialUKF(void*, uint64_t, uint64_t);
%ignore Update_inertialUKF;
%constant void SelfInit_inertialUKF(void*, uint64_t);
%ignore SelfInit_inertialUKF;
%constant void CrossInit_inertialUKF(void*, uint64_t);
%ignore CrossInit_inertialUKF;
%constant void Reset_inertialUKF(void*, uint64_t, uint64_t);
%ignore Reset_inertialUKF;
%include "inertialUKF.h"
%include "../_GeneralModuleFiles/ukfUtilities.h"
%include "../../fswMessages/inertialFilterMessage.h"
%include "../../fswMessages/stAttMessage.h"
%include "../../fswMessages/vehicleConfigMessage.h"
%include "../../fswMessages/rwArrayConfigMessage.h"
%include "../../../SimFswInterface/rwSpeedMessage.h"
GEN_SIZEOF(STAttMessage);
GEN_SIZEOF(RWConfigMessage);
GEN_SIZEOF(RWSpeedMessage);
GEN_SIZEOF(VehicleConfigMessage);

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

