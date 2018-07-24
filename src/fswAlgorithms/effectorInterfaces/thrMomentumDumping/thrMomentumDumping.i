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
%module thrMomentumDumping
%{
   #include "thrMomentumDumping.h"
%}

%include "swig_conly_data.i"
%constant void Update_thrMomentumDumping(void*, uint64_t, uint64_t);
%ignore Update_thrMomentumDumping;
%constant void SelfInit_thrMomentumDumping(void*, uint64_t);
%ignore SelfInit_thrMomentumDumping;
%constant void CrossInit_thrMomentumDumping(void*, uint64_t);
%ignore CrossInit_thrMomentumDumping;
%constant void Reset_thrMomentumDumping(void*, uint64_t, uint64_t);
%ignore Reset_thrMomentumDumping;
GEN_SIZEOF(thrMomentumDumpingConfig);
%include "thrMomentumDumping.h"
%include "../../fswMessages/thrArrayConfigFswMsg.h"
%include "../../fswMessages/thrArrayCmdForceFswMsg.h"
%include "simFswInterfaceMessages/thrArrayOnTimeCmdIntMsg.h"
GEN_SIZEOF(THRArrayConfigFswMsg);
GEN_SIZEOF(THRArrayCmdForceFswMsg);

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
