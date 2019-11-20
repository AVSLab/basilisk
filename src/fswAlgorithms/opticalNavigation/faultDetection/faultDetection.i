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
%module faultDetection
%{
   #include "faultDetection.h"
%}

%include "swig_conly_data.i"
%constant void SelfInit_faultDetection(void*, uint64_t);
%ignore SelfInit_faultDetection;
%constant void CrossInit_faultDetection(void*, uint64_t);
%ignore CrossInit_faultDetection;
%constant void Reset_faultDetection(void*, uint64_t, uint64_t);
%ignore Reset_faultDetection;
%constant void Update_faultDetection(void*, uint64_t, uint64_t);
%ignore Update_faultDetection;
STRUCTASLIST(FaultDetectionData)
GEN_SIZEOF(CameraConfigMsg)
GEN_SIZEOF(OpNavFswMsg)
GEN_SIZEOF(NavAttIntMsg)
%include "simFswInterfaceMessages/cameraConfigMsg.h"
%include "simFswInterfaceMessages/navAttIntMsg.h"
%include "../fswAlgorithms/fswMessages/opNavFswMsg.h"
%include "faultDetection.h"


%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}


