/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

%module lambertSurfaceRelativeVelocity

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "lambertSurfaceRelativeVelocity.h"
%}

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"
%include "std_vector.i"
%include "swig_eigen.i"

%include "sys_model.i"
%include "lambertSurfaceRelativeVelocity.h"

%include "architecture/msgPayloadDefC/LambertProblemMsgPayload.h"
struct LambertProblemMsg_C;
%include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
struct EphemerisMsg_C;
%include "architecture/msgPayloadDefC/DesiredVelocityMsgPayload.h"
struct DesiredVelocityMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
