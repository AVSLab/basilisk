/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

%{
   #include "MJActuator.h"
%}

%include "MJObject.swg"
%template_mujoco_object(mjsActuator)

%ignore MJActuatorObject;

%ignore MJActuator::MJActuator;
%ignore MJActuator::configure;
%ignore MJActuator::updateCtrl;
%ignore MJActuator::readControlMessages;

%ignore MJSingleActuator::MJSingleActuator;
%ignore MJSingleActuator::readControlMessages;

%ignore MJForceActuator::MJForceActuator;
%ignore MJForceActuator::readControlMessages;

%ignore MJTorqueActuator::MJTorqueActuator;
%ignore MJTorqueActuator::readControlMessages;

%ignore MJForceTorqueActuator::MJForceTorqueActuator;
%ignore MJForceTorqueActuator::readControlMessages;

%include "MJActuator.h"

%include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"
struct SingleActuatorMsg_C;
%include "architecture/msgPayloadDefC/ForceAtSiteMsgPayload.h"
struct ForceMsg_C;
%include "architecture/msgPayloadDefC/TorqueAtSiteMsgPayload.h"
struct TorqueMsg_C;
