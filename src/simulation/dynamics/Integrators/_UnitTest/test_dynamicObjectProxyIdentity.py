# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import subprocess
import sys

import pytest

from Basilisk import hasBuildFeature


EXPECTED_COMPATIBILITY_EXPORTS = (
    "SysModel",
    "DynamicObject",
    "StateVecIntegrator",
)


IMPORT_ORDERS = (
    (
        "dynamicObject",
        "spacecraft",
        "spacecraftChargingDynamics",
        "mujoco",
        "svIntegrators",
    ),
    (
        "svIntegrators",
        "mujoco",
        "spacecraftChargingDynamics",
        "spacecraft",
        "dynamicObject",
    ),
    (
        "spacecraft",
        "svIntegrators",
        "dynamicObject",
        "mujoco",
        "spacecraftChargingDynamics",
    ),
    (
        "mujoco",
        "svIntegrators",
        "spacecraftChargingDynamics",
        "dynamicObject",
        "spacecraft",
    ),
)


def test_swig_api_simplifications():
    """Keep the core integrator API check active in every build."""
    from Basilisk.simulation import dynamicObject

    assert not hasattr(dynamicObject.StateVecIntegrator, "getDynamics")


@pytest.mark.skipif(
    not hasBuildFeature("mujoco"),
    reason="Requires Basilisk built with --mujoco True",
)
def test_mujoco_swig_api_simplifications():
    """Reuse the canonical SysModel proxy for MuJoCo interpolators."""
    from Basilisk.simulation import dynamicObject
    from Basilisk.simulation import mujoco

    assert mujoco.SysModel is dynamicObject.SysModel
    assert mujoco.SingleActuatorInterpolatorBase.__base__ is dynamicObject.SysModel
    assert (
        mujoco.ScalarJointStateInterpolatorBase.__base__
        is dynamicObject.SysModel
    )
    assert dynamicObject.SysModel in mujoco.SingleActuatorInterpolatorBase.__mro__
    assert dynamicObject.SysModel in mujoco.ScalarJointStateInterpolatorBase.__mro__
    assert not hasattr(mujoco._mujoco, "new_SysModel")
    assert not hasattr(mujoco._mujoco, "delete_SysModel")
    assert not hasattr(mujoco._mujoco, "SysModel_swigregister")

    for interpolator_class in (
        mujoco.SingleActuatorInterpolator,
        mujoco.ScalarJointStateInterpolator,
    ):
        interpolator = interpolator_class()
        interpolator.ModelTag = "retained"
        interpolator.moduleID = 42
        assert interpolator.ModelTag == "retained"
        assert interpolator.moduleID == 42
        assert callable(interpolator.logger)


@pytest.mark.parametrize("import_order", IMPORT_ORDERS)
def test_canonical_dynamic_proxy_identity_in_fresh_process(import_order):
    """Check core and enabled optional consumers in several import orders."""
    worker = r"""
import importlib
import sys

from Basilisk import hasBuildFeature

expected_compatibility_exports = tuple(sys.argv[1].split(","))
for module_name in sys.argv[2:]:
    if module_name == "mujoco" and not hasBuildFeature("mujoco"):
        continue
    importlib.import_module(f"Basilisk.simulation.{module_name}")

from Basilisk.simulation import dynamicObject
from Basilisk.simulation import spacecraft
from Basilisk.simulation import spacecraftChargingDynamics
from Basilisk.simulation import svIntegrators

dynamic_consumers = [
    spacecraft,
    spacecraftChargingDynamics,
]
if hasBuildFeature("mujoco"):
    from Basilisk.simulation import mujoco
    dynamic_consumers.append(mujoco)

consumers = [*dynamic_consumers, svIntegrators]
assert dynamicObject._COMPATIBILITY_EXPORTS == expected_compatibility_exports
assert not hasattr(dynamicObject.StateVecIntegrator, "getDynamics")
for consumer in consumers:
    assert consumer.SysModel is dynamicObject.SysModel
    assert consumer.DynamicObject is dynamicObject.DynamicObject
    assert consumer.StateVecIntegrator is dynamicObject.StateVecIntegrator
    assert consumer.SysModel.__module__ == dynamicObject.__name__
    assert consumer.DynamicObject.__module__ == dynamicObject.__name__
    assert consumer.StateVecIntegrator.__module__ == dynamicObject.__name__
    for name in expected_compatibility_exports:
        assert getattr(consumer, name) is getattr(dynamicObject, name)

dynamic_classes = [
    spacecraft.Spacecraft,
    spacecraftChargingDynamics.SpacecraftChargingDynamics,
]
if hasBuildFeature("mujoco"):
    dynamic_classes.append(mujoco.MJScene)
for dynamic_class in dynamic_classes:
    assert dynamicObject.DynamicObject in dynamic_class.__mro__

if hasBuildFeature("mujoco"):
    for interpolator_class in (
        mujoco.SingleActuatorInterpolatorBase,
        mujoco.ScalarJointStateInterpolatorBase,
    ):
        assert dynamicObject.SysModel in interpolator_class.__mro__
        assert issubclass(interpolator_class, dynamicObject.SysModel)
        assert issubclass(interpolator_class, mujoco.SysModel)

integrator_classes = (
    svIntegrators.StateVecStochasticIntegrator,
    svIntegrators.svIntegratorRK2,
    svIntegrators.svIntegratorRKF45,
    svIntegrators.svStochasticIntegratorEulerHeun,
)
for integrator_class in integrator_classes:
    assert dynamicObject.StateVecIntegrator in integrator_class.__mro__

dynamic_instances = [
    spacecraft.Spacecraft(),
    spacecraftChargingDynamics.SpacecraftChargingDynamics(),
]
if hasBuildFeature("mujoco"):
    dynamic_instances.append(mujoco.MJScene("<mujoco/>"))
for dynamic_instance, consumer in zip(dynamic_instances, dynamic_consumers):
    assert isinstance(dynamic_instance, consumer.SysModel)
    assert isinstance(dynamic_instance, dynamicObject.DynamicObject)
    assert isinstance(
        dynamic_instance.integrator,
        dynamicObject.StateVecIntegrator,
    )

spacecraft_instance = dynamic_instances[0]
integrator = svIntegrators.svIntegratorRK2(spacecraft_instance)
assert isinstance(integrator, dynamicObject.StateVecIntegrator)
spacecraft_instance.setIntegrator(integrator)
assert not integrator.thisown

stochastic_integrator = svIntegrators.svStochasticIntegratorEulerHeun(
    spacecraft_instance
)
assert isinstance(stochastic_integrator, dynamicObject.StateVecIntegrator)
"""
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            worker,
            ",".join(EXPECTED_COMPATIBILITY_EXPORTS),
            *import_order,
        ],
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stdout + result.stderr
