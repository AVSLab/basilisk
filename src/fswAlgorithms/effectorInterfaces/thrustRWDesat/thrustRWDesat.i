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
%module thrustRWDesat
%{
   #include "thrustRWDesat.h"
%}

%include "swig_conly_data.i"
%constant void Update_thrustRWDesat(void*, uint64_t, uint64_t);
%ignore Update_thrustRWDesat;
%constant void SelfInit_thrustRWDesat(void*, uint64_t);
%ignore SelfInit_thrustRWDesat;
%constant void Reset_thrustRWDesat(void*, uint64_t, uint64_t);
%ignore Reset_thrustRWDesat;


%include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
struct RWSpeedMsg_C;
%include "architecture/msgPayloadDefC/RWConstellationMsgPayload.h"
struct RWConstellationMsg_C;
%include "architecture/msgPayloadDefC/THRArrayConfigMsgPayload.h"
struct THRArrayConfigMsg_C;
%include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
struct VehicleConfigMsg_C;
%include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
struct THRArrayOnTimeCmdMsg_C;

%include "thrustRWDesat.h"


%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}


