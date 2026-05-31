/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

%module(package="Basilisk.simulation") dynamicObject

%{
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
%}

%include "sys_model.i"
%include "simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h"

// setIntegrator() takes ownership of the integrator pointer in C++ (it
// `delete`s the previous one and stores the new one).  Tell SWIG to
// transfer Python ownership on call so the integrator is not double-freed
// when the Python reference later goes out of scope.
%apply SWIGTYPE *DISOWN { StateVecIntegrator* newIntegrator };
%include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
%clear StateVecIntegrator* newIntegrator;
