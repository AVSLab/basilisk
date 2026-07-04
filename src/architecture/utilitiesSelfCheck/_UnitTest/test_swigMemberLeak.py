#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

#
#   Unit Test Script
#   Module Name:        utilitiesSelfCheck (SWIG member wrapping)
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      July 1, 2026
#
#   Purpose:
#   Regression test for issue #422. Several modules exposed internal C++ members
#   (a std::pair-keyed map, the BSKLogger, thruster operating state, a
#   message-reader vector, IMU noise-model helpers, an Eigen-typed struct member)
#   that SWIG wrapped without a visible destructor, so reading them from Python
#   leaked an un-destructed proxy ("swig/python detected a memory leak of type
#   ...").  Each affected module is now instantiated in an isolated subprocess and
#   all of its attributes are read; the test fails if SWIG reports any such leak.
#

import subprocess
import sys

import pytest

# (import path, class name) for fixed module classes and affected wrapped data classes.
FIXED_MODULES = [
    ("Basilisk.simulation.spacecraft", "Spacecraft"),                                 # DynParamManager::sharedNoiseMap, bskLogger
    ("Basilisk.simulation.imuSensor", "ImuSensor"),                                   # Discretize/Saturate helpers
    ("Basilisk.simulation.sphericalHarmonicsGravityModel", "SphericalHarmonicsGravityModel"),  # bskLogger
    ("Basilisk.simulation.THRSimConfig", "THRSimConfig"),                             # THROperation ThrustOps
    ("Basilisk.simulation.dataFileToViz", "DataFileToViz"),                           # dataFileToViz module class
    ("Basilisk.simulation.dataFileToViz", "ThrClusterMap"),                           # Eigen::Vector3d thrOffset
    ("Basilisk.simulation.dataFileToViz", "RWConfigLogMsgPayload"),                   # RWModels enum in payload
    ("Basilisk.fswAlgorithms.smallBodyNavEKF", "SmallBodyNavEKF"),                    # ReadFunctor vector
    # BSpline data sets
    (
        "Basilisk.fswAlgorithms.constrainedAttitudeManeuver",
        "ConstrainedAttitudeManeuver",
    ),
    ("Basilisk.simulation.ReactionWheelPower", "RWConfigLogMsgPayload"),              # RWModels enum in payload
    ("Basilisk.simulation.motorThermal", "RWConfigLogMsgPayload"),                    # RWModels enum in payload
    ("Basilisk.simulation.linkBudget", "AttenuationLookupTable"),                     # std::vector<double> in lookup struct
    ("Basilisk.simulation.simpleInstrument", "DataStorageStatusMsgPayload"),          # std::vector<double> storedData
]

# Worker: import the module, instantiate the class, read every public attribute,
# force garbage collection. SWIG prints the leak warning to stdout/stderr when an
# un-destructed proxy is collected. Runs in a fresh interpreter per module so the
# parent test is immune to any process-level fault.
_WORKER = (
    "import gc, importlib, sys\n"
    "mod = importlib.import_module(sys.argv[1])\n"
    "obj = getattr(mod, sys.argv[2])()\n"
    "for a in dir(obj):\n"
    "    if not a.startswith('__'):\n"
    "        try:\n"
    "            v = getattr(obj, a); del v\n"
    "        except Exception:\n"
    "            pass\n"
    "del obj; gc.collect()\n"
)


@pytest.mark.parametrize("module_path,class_name", FIXED_MODULES)
def test_no_swig_member_leak(module_path, class_name):
    """Instantiating the module and reading its attributes must not leak a SWIG proxy."""
    proc = subprocess.run(
        [sys.executable, "-c", _WORKER, module_path, class_name],
        capture_output=True, text=True, timeout=120,
    )
    combined = proc.stdout + proc.stderr
    assert "detected a memory leak" not in combined, (
        f"{module_path}.{class_name} leaked a SWIG proxy:\n{combined}"
    )
    assert proc.returncode == 0, (
        f"{module_path}.{class_name} worker exited abnormally (rc={proc.returncode}):\n{combined}"
    )
