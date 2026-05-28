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
%module bskLogging

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
  #include "bskLogging.h"
  static PyObject *PyExc_BasiliskError;
%}

%init %{
    PyExc_BasiliskError = PyErr_NewException("_bskLogging.BasiliskError", NULL, NULL);
    Py_INCREF(PyExc_BasiliskError);
    PyModule_AddObject(m, "BasiliskError", PyExc_BasiliskError);
%}

%include "bskLogging.h"
%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}


%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])

BasiliskError = _bskLogging.BasiliskError

import enum

class LogLevel(enum.IntEnum):
    DEBUG = BSK_DEBUG
    INFO = BSK_INFORMATION
    WARNING = BSK_WARNING
    ERROR = BSK_ERROR
    SILENT = BSK_SILENT

DEBUG = LogLevel.DEBUG
INFO = LogLevel.INFO
WARNING = LogLevel.WARNING
ERROR = LogLevel.ERROR
SILENT = LogLevel.SILENT

# The debug()/info()/warning()/error()/setLevel() convenience methods are defined
# as SWIG %extend methods in bskLogging.h, not monkey-patched here. That way they
# exist on every BSKLogger proxy class regardless of module import order; a Python
# patch on this module's BSKLogger would not reach instances vended by other
# modules that %include bskLogging.h (e.g. anything derived from SysModel).
%}
