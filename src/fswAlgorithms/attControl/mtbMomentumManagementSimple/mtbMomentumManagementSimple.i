/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

%module mtbMomentumManagementSimple
%{
    #include "mtbMomentumManagementSimple.h"
%}

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}
%include "swig_conly_data.i"
%constant void Update_mtbMomentumManagementSimple(void*, uint64_t, uint64_t);
%ignore Update_mtbMomentumManagementSimple;
%constant void SelfInit_mtbMomentumManagementSimple(void*, uint64_t);
%ignore SelfInit_mtbMomentumManagementSimple;
%constant void Reset_mtbMomentumManagementSimple(void*, uint64_t, uint64_t);
%ignore Reset_mtbMomentumManagementSimple;

%include "mtbMomentumManagementSimple.h"

%include "architecture/msgPayloadDefC/RWArrayConfigMsgPayload.h"
struct RWArrayConfigMsg_C;
%include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
struct RWSpeedMsg_C;
%include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
struct CmdTorqueBodyMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

