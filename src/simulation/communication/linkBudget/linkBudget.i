/*
 ISC License

 Copyright (c) 2026, Departement of Engineering Cybernetics, NTNU

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

%module linkBudget

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "linkBudget.h"
%}

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"

%include "sys_model.i"
%include "linkBudget.h"

%include "architecture/msgPayloadDefC/AntennaLogMsgPayload.h"
struct AntennaLogMsg_C;
%include "architecture/msgPayloadDefC/LinkBudgetMsgPayload.h"
struct LinkBudgetMsg_C;

%pythoncode %{
def _configure_atm_lookup_paths_from_pooch(self):
    """Resolve ITU attenuation lookup files via ``supportDataTools.dataFetcher``.

    This keeps atmospheric attenuation data loading robust for wheel installs.
    Existing user-configured paths are preserved.
    """
    oxygen_path_set = bool(self.getOxygenLookupFilePath())
    water_path_set = bool(self.getWaterVaporLookupFilePath())
    if oxygen_path_set and water_path_set:
        return True

    try:
        from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

        if not oxygen_path_set:
            oxygen_file = get_path(DataFile.AtmRadioFreqDampData.oxygen)
            self.setOxygenLookupFilePath(str(oxygen_file))
        if not water_path_set:
            water_file = get_path(DataFile.AtmRadioFreqDampData.waterVapor)
            self.setWaterVaporLookupFilePath(str(water_file))
        return True
    except Exception:
        # Keep C++ relative-path fallback behavior if pooch resolution fails.
        return False

LinkBudget.configureAtmLookupPathsFromPooch = _configure_atm_lookup_paths_from_pooch
%}

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

%pythoncode %{
def _linkbudget_setattr_wrapper(self, name, value):
    protectSetAttr(self, name, value)
    if name == "atmosAtt" and bool(value):
        _configure_atm_lookup_paths_from_pooch(self)

LinkBudget.__setattr__ = _linkbudget_setattr_wrapper
%}
