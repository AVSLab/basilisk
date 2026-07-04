/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
%module constrainedAttitudeManeuver

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "constrainedAttitudeManeuver.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"
%include "sys_model.i"
// The module exposes public InputDataSet/OutputDataSet members (read from Python,
// e.g. testModule.Output.X1). Wrap the BSpline types here, with the Eigen typemaps,
// so SWIG has their destructors instead of emitting an un-destructed proxy on
// access (memory leak, issue #422).
%include "swig_eigen.i"
%include "architecture/utilities/BSpline.h"
%include "constrainedAttitudeManeuver.h"

%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsg_C;
%include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
struct VehicleConfigMsg_C;
%include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
struct SpicePlanetStateMsg_C;
%include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
struct AttRefMsg_C;


%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
