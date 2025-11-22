/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab

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

%module(directors="1",threads="1") MJMeanRevertingNoise

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
  #include "meanRevertingNoise.h"
  #include "stochasticAtmDensity.h"
  #include "stochasticDragCoeff.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

/* Common SWIG helpers */
%include "swig_eigen.i"
%include "std_string.i"
%include "exception.i"

/* Basilisk system-model base and helpers */
%import "simulation/mujocoDynamics/_GeneralModuleFiles/pyStatefulSysModel.i"

/* ============================
   Base class: MeanRevertingNoise
   ============================ */

/* Enable directors so Python subclasses can override virtual methods */
%feature("director") MeanRevertingNoise;

/* Rename the raw SWIG proxy to _MeanRevertingNoise so we can define
 * a Python-side wrapper class named MeanRevertingNoise that also
 * inherits from the Python StatefulSysModel wrapper.
 */
%rename("_MeanRevertingNoise") MeanRevertingNoise;
%include "meanRevertingNoise.h"

/* ============================
   StochasticAtmDensity
   ============================ */

/* Keep these concrete C++ classes as they are. We do not want them
 * to use the Python StatefulSysModel wrapper, and we still do not
 * expose their writeOutput directly.
 */
%ignore StochasticAtmDensity::writeOutput;
%include "stochasticAtmDensity.h"

%include "architecture/msgPayloadDefC/AtmoPropsMsgPayload.h"
struct AtmoPropsMsgPayload_C;

/* ============================
   StochasticDragCoeff
   ============================ */

%ignore StochasticDragCoeff::writeOutput;
%include "stochasticDragCoeff.h"

%include "architecture/msgPayloadDefC/DragGeometryMsgPayload.h"
struct DragGeometryMsgPayload_C;

%pythoncode %{
from Basilisk.architecture.sysModel import SysModelMixin

class MeanRevertingNoise(SysModelMixin, _MeanRevertingNoise):
    """Python wrapper for the C++ MeanRevertingNoise."""
%}
