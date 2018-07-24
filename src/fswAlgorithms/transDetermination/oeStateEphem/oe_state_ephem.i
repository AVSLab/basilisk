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
%module oe_state_ephem
%{
   #include "oeStateEphem.h"
%}

%include "swig_conly_data.i"
%constant void Update_oeStateEphem(void*, uint64_t, uint64_t);
%ignore Update_oeStateEphem;
%constant void SelfInit_oeStateEphem(void*, uint64_t);
%ignore SelfInit_oeStateEphem;
%constant void CrossInit_oeStateEphem(void*, uint64_t);
%ignore CrossInit_oeStateEphem;
%constant void Reset_oeStateEphem(void*, uint64_t, uint64_t);
%ignore Reset_oeStateEphem;
STRUCTASLIST(ChebyOERecord)
GEN_SIZEOF(TDBVehicleClockCorrelationFswMsg)
GEN_SIZEOF(EphemerisIntMsg)
%include "oeStateEphem.h"
%include "../../fswMessages/TDBVehicleClockCorrelationFswMsg.h"
%include "simFswInterfaceMessages/ephemerisIntMsg.h"
%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

