/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

%module fswMessages
%{
    #include "../fswMessages/rwAvailabilityFswMsg.h"
    #include "../fswMessages/rwAvailabilityFswMsg.h"
    #include "../fswMessages/attRefFswMsg.h"
    #include "../fswMessages/AccDataFswMsg.h"
    #include "../fswMessages/AccPktDataFswMsg.h"
    #include "../fswMessages/TDBVehicleClockCorrelationFswMsg.h"
    #include "../fswMessages/attGuidFswMsg.h"
    #include "../fswMessages/attRefFswMsg.h"
    #include "../fswMessages/cssConfigFswMsg.h"
    #include "../fswMessages/cssUnitConfigFswMsg.h"
    #include "../fswMessages/dvBurnCmdFswMsg.h"
    #include "../fswMessages/eulerAngleFswMsg.h"
    #include "../fswMessages/imuSensorBodyFswMsg.h"
    #include "../fswMessages/inertialFilterFswMsg.h"
    #include "../fswMessages/rwArrayConfigFswMsg.h"
    #include "../fswMessages/rwAvailabilityFswMsg.h"
    #include "../fswMessages/rwConfigElementFswMsg.h"
    #include "../fswMessages/rwConstellationFswMsg.h"
    #include "../fswMessages/stAttFswMsg.h"
    #include "../fswMessages/sunlineFilterFswMsg.h"
    #include "../fswMessages/thrArrayCmdForceFswMsg.h"
    #include "../fswMessages/thrArrayConfigFswMsg.h"
    #include "../fswMessages/thrConfigFswMsg.h"
    #include "../fswMessages/vehicleConfigFswMsg.h"
    #include "../fswMessages/rateCmdFswMsg.h"
%}

%include "swig_conly_data.i"

%array_functions(RWConfigElementFswMsg, RWConfigArray);
%array_functions(THRConfigFswMsg, ThrustConfigArray);
STRUCTASLIST(CSSUnitConfigFswMsg)

%include "../simFswInterfaceMessages/macroDefinitions.h"
%include "../fswMessages/AccDataFswMsg.h"
GEN_SIZEOF(AccDataFswMsg)
%include "../fswMessages/AccPktDataFswMsg.h"
GEN_SIZEOF(AccPktDataFswMsg)
%include "../fswMessages/TDBVehicleClockCorrelationFswMsg.h"
GEN_SIZEOF(TDBVehicleClockCorrelationFswMsg)
%include "../fswMessages/attGuidFswMsg.h"
GEN_SIZEOF(AttGuidFswMsg)
%include "../fswMessages/attRefFswMsg.h"
GEN_SIZEOF(AttRefFswMsg);
%include "../fswMessages/cssConfigFswMsg.h"
GEN_SIZEOF(CSSConfigFswMsg)
%include "../fswMessages/cssUnitConfigFswMsg.h"
GEN_SIZEOF(CSSUnitConfigFswMsg)
%include "../fswMessages/dvBurnCmdFswMsg.h"
GEN_SIZEOF(DvBurnCmdFswMsg)
%include "../fswMessages/eulerAngleFswMsg.h"
GEN_SIZEOF(EulerAngleFswMsg)
%include "../fswMessages/imuSensorBodyFswMsg.h"
GEN_SIZEOF(IMUSensorBodyFswMsg)
%include "../fswMessages/inertialFilterFswMsg.h"
GEN_SIZEOF(InertialFilterFswMsg)
%include "../fswMessages/rwArrayConfigFswMsg.h"
GEN_SIZEOF(RWArrayConfigFswMsg)
%include "../fswMessages/rwAvailabilityFswMsg.h"
GEN_SIZEOF(RWAvailabilityFswMsg)
%include "../fswMessages/rwConfigElementFswMsg.h"
GEN_SIZEOF(RWConfigElementFswMsg);
%include "../fswMessages/rwConstellationFswMsg.h"
GEN_SIZEOF(RWConstellationFswMsg);
%include "../fswMessages/stAttFswMsg.h"
GEN_SIZEOF(STAttFswMsg);
%include "../fswMessages/sunlineFilterFswMsg.h"
GEN_SIZEOF(SunlineFilterFswMsg);
%include "../fswMessages/thrArrayCmdForceFswMsg.h"
GEN_SIZEOF(THRArrayCmdForceFswMsg);
%include "../fswMessages/thrArrayConfigFswMsg.h"
GEN_SIZEOF(THRArrayConfigFswMsg);
%include "../fswMessages/thrConfigFswMsg.h"
GEN_SIZEOF(THRConfigFswMsg);
%include "../fswMessages/vehicleConfigFswMsg.h"
GEN_SIZEOF(VehicleConfigFswMsg);
%include "../fswMessages/rateCmdFswMsg.h"
GEN_SIZEOF(RateCmdFswMsg);


%pythoncode %{
    import sys
    protectAllClasses(sys.modules[__name__])
    %}
