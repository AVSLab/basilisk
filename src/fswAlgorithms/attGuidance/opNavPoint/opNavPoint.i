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
%module opNavPoint
%{
   #include "opNavPoint.h"
%}

%include "swig_conly_data.i"

%constant void Update_opNavPoint(void*, uint64_t, uint64_t);
%ignore Update_opNavPoint;
%constant void SelfInit_opNavPoint(void*, uint64_t);
%ignore SelfInit_opNavPoint;
%constant void CrossInit_opNavPoint(void*, uint64_t);
%ignore CrossInit_opNavPoint;
%constant void Reset_opNavPoint(void*, uint64_t, uint64_t);
%ignore Reset_opNavPoint;
%include "simFswInterfaceMessages/navAttIntMsg.h"
%include "../../fswMessages/attGuidFswMsg.h"
%include "../../fswMessages/opNavFswMsg.h"

GEN_SIZEOF(OpNavPointConfig);
GEN_SIZEOF(AttGuidFswMsg);
GEN_SIZEOF(OpNavFswMsg);
GEN_SIZEOF(NavAttIntMsg);
%include "opNavPoint.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
