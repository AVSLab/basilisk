# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Keep native Debug assertions visible in the temporary Visual Studio CI run."""

import ctypes
import os


def pytest_configure():
    """Send Debug CRT reports to stderr instead of opening unattended dialogs."""
    if os.name != "nt":
        return
    from Basilisk import getBuildInfo

    if not getBuildInfo()["abi"]["build"]["debugRuntime"]:
        return
    runtime = ctypes.CDLL("ucrtbased.dll")
    runtime._set_error_mode.argtypes = [ctypes.c_int]
    runtime._set_error_mode(1)  # _OUT_TO_STDERR
    runtime._CrtSetReportMode.argtypes = [ctypes.c_int, ctypes.c_int]
    runtime._CrtSetReportFile.argtypes = [ctypes.c_int, ctypes.c_void_p]
    runtime._CrtSetReportFile.restype = ctypes.c_void_p
    for report_type in (0, 1, 2):  # _CRT_WARN, _CRT_ERROR, _CRT_ASSERT
        runtime._CrtSetReportMode(report_type, 1)  # _CRTDBG_MODE_FILE
        runtime._CrtSetReportFile(report_type, ctypes.c_void_p(-5))  # _CRTDBG_FILE_STDERR
