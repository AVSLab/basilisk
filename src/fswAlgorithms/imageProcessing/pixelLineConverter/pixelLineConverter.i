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
%module pixelLineConverter
%{
   #include "pixelLineConverter.h"
%}

%include "swig_conly_data.i"
%constant void SelfInit_pixelLineConverter(void*, uint64_t);
%ignore SelfInit_pixelLineConverter;
%constant void CrossInit_pixelLineConverter(void*, uint64_t);
%ignore CrossInit_pixelLineConverter;
%constant void Reset_pixelLineConverter(void*, uint64_t, uint64_t);
%ignore Reset_pixelLineConverter;
%constant void Update_pixelLineConverter(void*, uint64_t, uint64_t);
%ignore Update_pixelLineConverter;
STRUCTASLIST(PixelLineConvertData)
GEN_SIZEOF(CirclesOpNavMsg)
GEN_SIZEOF(CameraConfigMsg)
GEN_SIZEOF(NavAttIntMsg)
GEN_SIZEOF(OpNavFswMsg)
%include "simFswInterfaceMessages/circlesOpNavMsg.h"
%include "simFswInterfaceMessages/cameraConfigMsg.h"
%include "simFswInterfaceMessages/navAttIntMsg.h"
%include "../fswAlgorithms/fswMessages/opNavFswMsg.h"
%include "pixelLineConverter.h"


%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}


