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
%module(directors="1",threads="1") StatefulSysModel
%{
   #include "StatefulSysModel.h"
%}

%pythoncode %{
import sys
import traceback
from Basilisk.architecture.swig_common_model import *
%}

%include "architecture/utilities/bskException.swg"
%include "architecture/utilities/bskLogging.h"
%import "architecture/_GeneralModuleFiles/py_sys_model.i"

// We don't need to construct the DynParamRegisterer on the Python side
%ignore DynParamRegisterer::DynParamRegisterer;

%feature("director") StatefulSysModel;
%feature("pythonappend") StatefulSysModel::StatefulSysModel %{
    self.__super_init_called__ = True%}
%rename("_StatefulSysModel") StatefulSysModel;
%include "StatefulSysModel.h"

%template(registerState) DynParamRegisterer::registerState<StateData, true>;

%pythoncode %{
class StatefulSysModel(_StatefulSysModel, metaclass=Basilisk.architecture.sysModel.SuperInitChecker):
    bskLogger: BSKLogger = None

    def __init_subclass__(cls):
        # Make it so any exceptions in UpdateState and Reset
        # print any exceptions before returning control to
        # C++ (at which point exceptions will crash the program)
        cls.UpdateState = Basilisk.architecture.sysModel.logError(cls.UpdateState)
        cls.Reset = Basilisk.architecture.sysModel.logError(cls.Reset)
        cls.registerStates = Basilisk.architecture.sysModel.logError(cls.registerStates)
%}
