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

%module spiceInterface

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "spiceInterface.h"
   #include "SpiceUsr.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

%include "swig_conly_data.i"
%include "std_string.i"
%include "std_vector.i"
%include "swig_deprecated.i"

%deprecated_function(
   SpiceInterface::clearKeeper,
   "2026/11/20",
   "This method will delete kernels that other simulations running in parallel may be using. spiceInterface will clear automatically the kernels that it has loaded."
)

%template() std::vector<std::string>;
%naturalvar SpiceInterface::planetFrames;
%ignore SpiceKernel;

%include "sys_model.i"
%include "spiceInterface.h"

%include "architecture/msgPayloadDefC/EpochMsgPayload.h"
struct EpochMsg_C;
%include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
struct SpicePlanetStateMsg_C;
%include "architecture/msgPayloadDefC/SpiceTimeMsgPayload.h"
struct SpiceTimeMsg_C;
%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsg_C;
%include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
struct AttRefMsg_C;
%include "architecture/msgPayloadDefC/TransRefMsgPayload.h"
struct TransRefMsg_C;

%inline %{
bool isKernelLoaded(const std::string &path)
{
    SpiceInt count = 0;
    ktotal_c("ALL", &count);

    for (SpiceInt i = 0; i < count; ++i)
    {
        SpiceChar file[512];
        SpiceChar type[32];
        SpiceChar source[512];
        SpiceInt handle;
        SpiceBoolean found;

        kdata_c(i,
                "ALL",
                (SpiceInt)sizeof(file),
                (SpiceInt)sizeof(type),
                (SpiceInt)sizeof(source),
                file,
                type,
                source,
                &handle,
                &found);

        if (found && path == std::string(file))
        {
            return true;
        }
    }
    return false;
}

size_t countKernelsLoaded()
{
    SpiceInt count = 0;
    ktotal_c("ALL", &count);
    return static_cast<size_t>(count);
}
%}   // <-- IMPORTANT: close %inline before any %pythoncode

%pythoncode %{
def _bsk_configure_spice_default_kernels(self, auto_configure_kernels=True):
    if not auto_configure_kernels:
        return
    try:
        from Basilisk.utilities.supportDataTools.spiceKernels import configure_spiceinterface_default_kernels
        configure_spiceinterface_default_kernels(self)
    except Exception as e:
        import warnings
        warnings.warn(f"SpiceInterface autoconfig failed: {e!r}")

_old_init = SpiceInterface.__init__
def __init__(self, *args, auto_configure_kernels=True, **kwargs):
    _old_init(self, *args, **kwargs)
    _bsk_configure_spice_default_kernels(self, auto_configure_kernels=auto_configure_kernels)

SpiceInterface.__init__ = __init__
%}

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
