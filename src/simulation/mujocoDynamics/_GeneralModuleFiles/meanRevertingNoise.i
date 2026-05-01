/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab

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

%module(directors="1",threads="1",package="Basilisk.simulation") MJMeanRevertingNoise

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
  #include "simulation/mujocoDynamics/_GeneralModuleFiles/meanRevertingNoise.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

%include "swig_eigen.i"
%include "std_string.i"
%include "exception.i"

%import "simulation/mujocoDynamics/_GeneralModuleFiles/StatefulSysModel.i"

// Enable Python subclassing: director allows Python to override the C++ virtual writeOutput().
%feature("director") MeanRevertingNoise;
// Rename the C++ class so that the Python wrapper class defined below can use the unqualified name.
%rename("_MeanRevertingNoise") MeanRevertingNoise;
%include "simulation/mujocoDynamics/_GeneralModuleFiles/meanRevertingNoise.h"

%pythoncode %{
from Basilisk.architecture.sysModel import SysModelMixin

# Mixin must come first so that SysModelMixin.__init_subclass__ runs before _MeanRevertingNoise.__init__.
class MeanRevertingNoise(SysModelMixin, _MeanRevertingNoise):
    """Python-subclassable wrapper for the C++ MeanRevertingNoise base class."""
%}
