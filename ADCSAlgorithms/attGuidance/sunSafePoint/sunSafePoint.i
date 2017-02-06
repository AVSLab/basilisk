/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
%module sunSafePoint
%{
   #include "sunSafePoint.h"
%}

%include "swig_conly_data.i"

%constant void Update_sunSafePoint(void*, uint64_t, uint64_t);
%ignore Update_sunSafePoint;
%constant void SelfInit_sunSafePoint(void*, uint64_t);
%ignore SelfInit_sunSafePoint;
%constant void CrossInit_sunSafePoint(void*, uint64_t);
%ignore CrossInit_sunSafePoint;
%array_functions(SingleCSSConfig, CSSWlsConfigArray);
%include "../../fswMessages/sunHeadingEstMessage.h"
%include "../../attDetermination/_GeneralModuleFiles/singleCSSConfig.h"
GEN_SIZEOF(sunSafePointConfig);
GEN_SIZEOF(AttGuidMessage);
GEN_SIZEOF(SunHeadingEstMessage);
%include "sunSafePoint.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
