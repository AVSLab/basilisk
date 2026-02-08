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

%module vscmgVelocitySteering

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "vscmgVelocitySteering.h"
%}

%include "std_vector.i"
%template(DoubleVector) std::vector<double>;

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"

%include "sys_model.i"
%include "vscmgVelocitySteering.h"

%include "architecture/msgPayloadDefC/VSCMGArrayConfigMsgPayload.h"
struct VSCMGArrayConfigMsg_C;
%include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
struct CmdTorqueBodyMsg_C;
%include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
struct NavAttMsg_C;
%include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
struct AttGuidMsg_C;
%include "architecture/msgPayloadDefC/VSCMGSpeedMsgPayload.h"
struct VSCMGSpeedMsg_C;
%include "architecture/msgPayloadDefC/VSCMGRefStatesMsgPayload.h"
struct VSCMGRefStatesMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
