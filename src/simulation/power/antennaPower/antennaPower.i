/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU, Norway

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


%module antennaPower

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "simulation/communication/_GeneralModuleFiles/AntennaDefinitions.h"
    #include "antennaPower.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

%include "stdint.i"
%include "std_string.i"

%include "sys_model.i"
%include "simulation/power/_GeneralModuleFiles/powerNodeBase.h"
%include "simulation/communication/_GeneralModuleFiles/AntennaDefinitions.h"
%include "antennaPower.h"

%include "architecture/msgPayloadDefC/PowerNodeUsageMsgPayload.h"
struct PowerNodeUsageMsg_C;
%include "architecture/msgPayloadDefC/AntennaLogMsgPayload.h"
struct AntennaOutMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
