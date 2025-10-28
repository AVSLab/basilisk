/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

%module simpleAntenna

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "simpleAntenna.h"
    #include "AntennaDefinitions.h"
    #include "simulation/power/_GeneralModuleFiles/powerNodeBase.h"
%}

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "std_vector.i"
%include "swig_conly_data.i"

namespace std {
    %template(DoubleVector) vector<double>;
}

%include "AntennaDefinitions.h"
%include "sys_model.i"
%include "simulation/power/_GeneralModuleFiles/powerNodeBase.h"
%include "simpleAntenna.h"

%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct scStateOutMsg_C;
%include "architecture/msgPayloadDefC/GroundStateMsgPayload.h"
struct GroundStateMsg_C;
%include "architecture/msgPayloadDefC/AntennaStateMsgPayload.h"
struct antennaStateMsg_C;
%include "architecture/msgPayloadDefC/AntennaOutMsgPayload.h"
struct antennaOutMsg_C;
%include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
struct SpicePlanetStateMsg_C;
%include "architecture/msgPayloadDefC/PowerNodeUsageMsgPayload.h"
struct PowerNodeUsageMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
