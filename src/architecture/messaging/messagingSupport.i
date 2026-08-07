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

%module(threads="1", package="Basilisk.architecture.messaging") messagingSupport

%include "std_vector.i"
%include "std_string.i"
%include "_GeneralModuleFiles/swig_eigen.i"
%include "_GeneralModuleFiles/swig_conly_data.i"
%include "stdint.i"

%template(TimeVector) std::vector<unsigned long long, std::allocator<unsigned long long>>;
%template(DoubleVector) std::vector<double, std::allocator<double>>;
%template(StringVector) std::vector<std::string, std::allocator<std::string>>;

%typemap(out) uintptr_t {
    $result = PyLong_FromUnsignedLongLong((unsigned long long)$1);
}

/*
 * Payload wrappers import this file for its shared SWIG type information, but
 * only this extension owns the common runtime helpers and their C/C++ headers.
 */
#ifndef BSK_MESSAGING_SUPPORT_IMPORT
%{
#include "architecture/msgPayloadDefC/ReconfigBurnInfoMsgPayload.h"
#include "architecture/msgPayloadDefC/RWConfigElementMsgPayload.h"
#include "architecture/msgPayloadDefC/THRConfigMsgPayload.h"
#include "architecture/utilities/macroDefinitions.h"
#include "fswAlgorithms/fswUtilities/fswDefinitions.h"
#include "simulation/dynamics/reactionWheels/reactionWheelSupport.h"
%}

%include "architecture/utilities/macroDefinitions.h"
%include "fswAlgorithms/fswUtilities/fswDefinitions.h"
%include "simulation/dynamics/reactionWheels/reactionWheelSupport.h"
%include "architecture/messaging/messagingSupportTypemaps.swg"

%array_functions(THRConfigMsgPayload, ThrustConfigArray);
%array_functions(RWConfigElementMsgPayload, RWConfigArray);
%array_functions(ReconfigBurnInfoMsgPayload, ReconfigBurnArray);
#endif
