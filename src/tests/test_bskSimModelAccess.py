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

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MASTER_CONFIGURATIONS = (
    (
        "BskSim",
        REPOSITORY_ROOT / "examples/BskSim/BSK_masters.py",
        "BSKDynamicModels",
        "BSKFswModels",
    ),
    (
        "OpNav",
        REPOSITORY_ROOT / "examples/OpNavScenarios/BSK_OpNav.py",
        "BSKDynamicModels",
        "BSKFswModels",
    ),
    (
        "MuJoCo",
        REPOSITORY_ROOT / "examples/mujoco/BSK_mujocoMasters.py",
        "BSKMujocoDynamicsModels",
        "BSKMujocoFSWModels",
    ),
)


def loadMasterClass(masterName, masterPath):
    """Load and return a ``BSKSim`` class from an example master file."""
    moduleName = f"test_{masterName}_master"
    moduleSpec = importlib.util.spec_from_file_location(moduleName, masterPath)
    module = importlib.util.module_from_spec(moduleSpec)
    moduleSpec.loader.exec_module(module)
    return module.BSKSim


def makeModelModule(dynClassName, fswClassName):
    """Create lightweight model factories for exercising the master classes."""

    class DynamicsModel:
        def __init__(self, simBase, dynRate):
            self.simBase = simBase
            self.rate = dynRate

    class FswModel:
        def __init__(self, simBase, fswRate):
            self.simBase = simBase
            self.rate = fswRate
            self.dynModels = simBase.DynModels

    modelModule = SimpleNamespace()
    setattr(modelModule, dynClassName, DynamicsModel)
    setattr(modelModule, fswClassName, FswModel)
    return modelModule


@pytest.mark.parametrize(
    "masterName, masterPath, dynClassName, fswClassName",
    MASTER_CONFIGURATIONS,
    ids=[configuration[0] for configuration in MASTER_CONFIGURATIONS],
)
def test_modelAccessRequiresInitialization(masterName, masterPath, dynClassName, fswClassName):
    """Verify that both access styles reject uninitialized model access."""
    del dynClassName, fswClassName
    masterClass = loadMasterClass(masterName, masterPath)
    simulation = masterClass()

    with pytest.raises(RuntimeError, match="dynamics model has not been added"):
        _ = simulation.DynModels
    with pytest.raises(RuntimeError, match="dynamics model has not been added"):
        simulation.get_DynModel()
    with pytest.raises(RuntimeError, match="flight software model has not been added"):
        _ = simulation.FSWModels
    with pytest.raises(RuntimeError, match="flight software model has not been added"):
        simulation.get_FswModel()


@pytest.mark.parametrize(
    "masterName, masterPath, dynClassName, fswClassName",
    MASTER_CONFIGURATIONS,
    ids=[configuration[0] for configuration in MASTER_CONFIGURATIONS],
)
def test_modelReferencesAreReadOnlyAndSingleUse(masterName, masterPath, dynClassName, fswClassName):
    """Verify model references cannot be replaced or initialized more than once."""
    masterClass = loadMasterClass(masterName, masterPath)
    simulation = masterClass(fswRate=0.25, dynRate=0.5)  # [s]
    modelModule = makeModelModule(dynClassName, fswClassName)

    simulation.set_DynModel(modelModule)
    dynModels = simulation.DynModels
    assert simulation.get_DynModel() is dynModels
    assert dynModels.rate == simulation.dynRate
    assert simulation.dynamics_added is True

    simulation.set_FswModel(modelModule)
    fswModels = simulation.FSWModels
    assert simulation.get_FswModel() is fswModels
    assert fswModels.dynModels is dynModels
    assert fswModels.rate == simulation.fswRate
    assert simulation.fsw_added is True

    with pytest.raises(AttributeError):
        simulation.DynModels = object()
    with pytest.raises(AttributeError):
        simulation.FSWModels = object()
    with pytest.raises(RuntimeError, match="dynamics model has already been added"):
        simulation.set_DynModel(modelModule)
    with pytest.raises(RuntimeError, match="flight software model has already been added"):
        simulation.set_FswModel(modelModule)


@pytest.mark.parametrize(
    "masterName, masterPath, dynClassName, fswClassName",
    MASTER_CONFIGURATIONS,
    ids=[configuration[0] for configuration in MASTER_CONFIGURATIONS],
)
def test_failedConstructionDoesNotPublishModel(masterName, masterPath, dynClassName, fswClassName):
    """Verify failed constructors leave the corresponding model unpublished."""
    masterClass = loadMasterClass(masterName, masterPath)
    modelModule = makeModelModule(dynClassName, fswClassName)

    class FailingDynamicsModel:
        def __init__(self, simBase, dynRate):
            del simBase, dynRate
            raise ValueError("dynamics construction failed")

    setattr(modelModule, dynClassName, FailingDynamicsModel)
    simulation = masterClass()
    with pytest.raises(ValueError, match="dynamics construction failed"):
        simulation.set_DynModel(modelModule)
    assert simulation.dynamics_added is False
    with pytest.raises(RuntimeError, match="dynamics model has not been added"):
        _ = simulation.DynModels

    class FailingFswModel:
        def __init__(self, simBase, fswRate):
            del simBase, fswRate
            raise ValueError("flight software construction failed")

    modelModule = makeModelModule(dynClassName, fswClassName)
    setattr(modelModule, fswClassName, FailingFswModel)
    simulation = masterClass()
    simulation.set_DynModel(modelModule)
    with pytest.raises(ValueError, match="flight software construction failed"):
        simulation.set_FswModel(modelModule)
    assert simulation.fsw_added is False
    with pytest.raises(RuntimeError, match="flight software model has not been added"):
        _ = simulation.FSWModels
