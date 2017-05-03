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

%module simFswInterfaceMessages
%{
#include "cmdForceBodyIntMsg.h"
#include "cmdForceInertialIntMsg.h"
#include "cmdTorqueBodyIntMsg.h"
#include "cssArraySensorIntMsg.h"
#include "ephemerisIntMsg.h"
#include "imuSensorIntMsg.h"
#include "navAttIntMsg.h"
#include "navTransIntMsg.h"
#include "rwArrayTorqueIntMsg.h"
#include "rwArrayVoltageIntMsg.h"
#include "rwSpeedIntMsg.h"
#include "stSensorIntMsg.h"
#include "thrArrayOnTimeCmdIntMsg.h"
%}

%include "swig_conly_data.i"
%include "swig_common_model.i"

%include "cmdForceBodyIntMsg.h"
GEN_SIZEOF(CmdForceBodyIntMsg)
%include "cmdForceInertialIntMsg.h"
GEN_SIZEOF(CmdForceInertialIntMsg)
%include "cmdTorqueBodyIntMsg.h"
GEN_SIZEOF(CmdTorqueBodyIntMsg)
%include "cssArraySensorIntMsg.h"
GEN_SIZEOF(CSSArraySensorIntMsg)
%include "ephemerisIntMsg.h"
GEN_SIZEOF(EphemerisIntMsg)
%include "imuSensorIntMsg.h"
GEN_SIZEOF(IMUSensorIntMsg)
%include "navAttIntMsg.h"
GEN_SIZEOF(NavAttIntMsg)
%include "navTransIntMsg.h"
GEN_SIZEOF(NavTransIntMsg)
%include "rwArrayTorqueIntMsg.h"
GEN_SIZEOF(RWArrayTorqueIntMsg)
%include "rwArrayVoltageIntMsg.h"
GEN_SIZEOF(RWArrayVoltageIntMsg)
%include "rwSpeedIntMsg.h"
GEN_SIZEOF(RWSpeedIntMsg)
%include "stSensorIntMsg.h"
GEN_SIZEOF(STSensorIntMsg)
%include "thrArrayOnTimeCmdIntMsg.h"
GEN_SIZEOF(THRArrayOnTimeCmdIntMsg)


%pythoncode %{
    import sys
    protectAllClasses(sys.modules[__name__])
    %}
