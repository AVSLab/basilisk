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
%module simpleDeadband
%{
   #include "simpleDeadband.h"
%}

%include "swig_conly_data.i"
%constant void Update_simpleDeadband(void*, uint64_t, uint64_t);
%ignore Update_simpleDeadband;
%constant void SelfInit_simpleDeadband(void*, uint64_t);
%ignore SelfInit_simpleDeadband;
%constant void CrossInit_simpleDeadband(void*, uint64_t);
%ignore CrossInit_simpleDeadband;
%constant void Reset_simpleDeadband(void*, uint64_t, uint64_t);
%ignore Reset_simpleDeadband;
GEN_SIZEOF(simpleDeadbandConfig);
GEN_SIZEOF(AttGuidFswMsg);
%include "simpleDeadband.h"

// sample Module supportfile to be included in this sub-module
%include "../../fswMessages/attGuidFswMsg.h"

%pythoncode %{
    import sys
    protectAllClasses(sys.modules[__name__])
%}
