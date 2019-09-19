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
%module horizonOpNav
%{
   #include "horizonOpNav.h"
%}

%include "swig_conly_data.i"
%constant void SelfInit_horizonOpNav(void*, uint64_t);
%ignore SelfInit_horizonOpNav;
%constant void CrossInit_horizonOpNav(void*, uint64_t);
%ignore CrossInit_horizonOpNav;
%constant void Reset_horizonOpNav(void*, uint64_t, uint64_t);
%ignore Reset_horizonOpNav;
%constant void Update_horizonOpNav(void*, uint64_t, uint64_t);
%ignore Update_horizonOpNav;
STRUCTASLIST(HorizonOpNavData)
GEN_SIZEOF(LimbOpNavMsg)
GEN_SIZEOF(CameraConfigMsg)
GEN_SIZEOF(NavAttIntMsg)
GEN_SIZEOF(OpNavFswMsg)
%include "simFswInterfaceMessages/limbOpNavMsg.h"
%include "simFswInterfaceMessages/cameraConfigMsg.h"
%include "simFswInterfaceMessages/navAttIntMsg.h"
%include "../fswAlgorithms/fswMessages/opNavFswMsg.h"
%include "horizonOpNav.h"


%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}


