/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU, Norway

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

%module simpleAntenna

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "simpleAntenna.h"
    #include "../_GeneralModuleFiles/AntennaDefinitions.h"
%}

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "std_vector.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"

%include "../_GeneralModuleFiles/AntennaDefinitions.h"
%include "sys_model.i"

// Rename C++ methods so we can wrap them in Python
%rename(_configureBrightnessFile) SimpleAntenna::configureBrightnessFile;
%rename(_setUseHaslamMap) SimpleAntenna::setUseHaslamMap;

%include "simpleAntenna.h"

%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct scStateOutMsg_C;
%include "architecture/msgPayloadDefC/GroundStateMsgPayload.h"
struct GroundStateMsg_C;
%include "architecture/msgPayloadDefC/AntennaStateMsgPayload.h"
struct antennaStateMsg_C;
%include "architecture/msgPayloadDefC/AntennaLogMsgPayload.h"
struct antennaOutMsg_C;
%include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
struct SpicePlanetStateMsg_C;
%include "architecture/msgPayloadDefC/EclipseMsgPayload.h"
struct EclipseMsg_C;

// Python wrappers
%pythoncode %{
def _configureBrightnessFile_wrapper(self, file):
    """Configure the Haslam sky brightness FITS file path.

    Args:
        file: Path to the FITS file (string or Path-like object)
    """
    if hasattr(file, '__fspath__'):
        file = str(file)
    self._configureBrightnessFile(file)

def _setUseHaslamMap_wrapper(self, value):
    """Enable or disable the Haslam sky map for background noise calculation.

    When enabled, automatically configures the brightness file path if not already set.

    Args:
        value: True to enable, False to disable
    """
    if value:
        # Auto-configure the brightness file path
        try:
            from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile
            brightness_file = get_path(DataFile.SkyBrightnessData.skyTemperature408MHz)
            self._configureBrightnessFile(str(brightness_file))
        except Exception as e:
            print(f"Warning: Could not auto-configure Haslam brightness file: {e}")
            print("Please call configureBrightnessFile() manually before setUseHaslamMap(True)")
    self._setUseHaslamMap(value)

SimpleAntenna.configureBrightnessFile = _configureBrightnessFile_wrapper
SimpleAntenna.setUseHaslamMap = _setUseHaslamMap_wrapper
%}

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
