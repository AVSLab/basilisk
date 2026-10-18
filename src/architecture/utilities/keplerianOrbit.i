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
%module keplerianOrbit

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "keplerianOrbit.h"
   #include <stdexcept>
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "swig_eigen.i"

// swig_eigen.i installs its own handler; extend it after that include.
%default_bsk_exception(catch (const std::invalid_argument& error) {
    SWIG_exception_fail(SWIG_ValueError, error.what());
});

%ignore KeplerianOrbit::operator=;
%include "keplerianOrbit.h"
%include "swig_orbitalMotion.swg"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
