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
# Basilisk BSK Faults Integrated Test
#
# Purpose:  Verify that BskSim fault classes in ``examples/BskSim/models/BSK_Faults.py``
#           can be configured and injected through ``addFaultToSimulation`` and
#           execute correctly in a ``BSKSim``/``BSKScenario`` run.
# Author:   Yumeka Nagano
# Creation Date:  Feb 16, 2026
#

import os
import sys

import numpy as np
import pytest
from Basilisk.utilities import macros, orbitalMotion, vizSupport


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
BSK_BASE_DIR = os.path.abspath(os.path.join(THIS_DIR, "../../examples/BskSim"))
BSK_MODELS_DIR = os.path.join(BSK_BASE_DIR, "models")
if BSK_MODELS_DIR not in sys.path:
    sys.path.append(BSK_MODELS_DIR)
if BSK_BASE_DIR not in sys.path:
    sys.path.append(BSK_BASE_DIR)

import BSK_Dynamics
import BSK_Faults
import BSK_Fsw
from BSK_masters import BSKScenario, BSKSim


class FaultInjectionTestScenario(BSKSim, BSKScenario):
    def __init__(self):
        super(FaultInjectionTestScenario, self).__init__()
        self.name = "faultInjectionTestScenario"
        self.set_DynModel(BSK_Dynamics)
        self.set_FswModel(BSK_Fsw)
        self.configure_initial_conditions()

    def configure_initial_conditions(self):
        oe = orbitalMotion.ClassicElements()
        oe.a = 6835525.292
        oe.e = 0.00063
        oe.i = 98.0 * macros.D2R
        oe.Omega = 85.47 * macros.D2R
        oe.omega = 0.0 * macros.D2R
        oe.f = 30.0 * macros.D2R

        dyn_models = self.get_DynModel()
        mu = dyn_models.gravFactory.gravBodies["earth"].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        dyn_models.scObject.hub.r_CN_NInit = rN
        dyn_models.scObject.hub.v_CN_NInit = vN
        dyn_models.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
        dyn_models.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]

    def configure_faults(self, fault_list):
        self.faultList = fault_list
        for i, fault in enumerate(self.faultList):
            fault.addFaultToSimulation(self, i)

    def log_outputs(self):
        return

    def pull_outputs(self):
        return


def run_faults(faults, stop_sec=1.0):
    scenario = FaultInjectionTestScenario()
    scenario.modeRequest = "hillPoint"
    scenario.configure_faults(faults)
    scenario.InitializeSimulation()
    scenario.ConfigureStopTime(macros.sec2nano(stop_sec))
    scenario.ExecuteSimulation()
    return scenario


def as_flat_float_array(value):
    """Convert Basilisk/SWIG containers (possibly nested) into a flat float ndarray."""
    return np.asarray(value, dtype=float).ravel()


def test_fault_helpers():
    a = BSK_Faults.RwPowerFault(time=0, reducedLimit=0.5, wheelIdx=0)
    b = BSK_Faults.RwFrictionFault(time=0, frictionMultiplier=2.0, wheelIdx=1)
    fault_array = np.array([a, b], dtype=object)

    assert BSK_Faults.faultTypeInList(fault_array, BSK_Faults.RwPowerFault)
    found = BSK_Faults.getFaultFromList(fault_array, BSK_Faults.RwFrictionFault)
    assert len(found) == 1
    assert isinstance(found[0], BSK_Faults.RwFrictionFault)


@pytest.mark.parametrize("wheel_idx,expected_rw", [(0, 1), (1, 2), (2, 3), (3, 4)])
def test_rw_power_fault(wheel_idx, expected_rw):
    fault = BSK_Faults.RwPowerFault(
        time=0,
        reducedLimit=0.25,
        wheelIdx=wheel_idx,
    )
    scenario = run_faults([fault])
    rw_obj = getattr(scenario.get_DynModel(), f"RW{expected_rw}")
    assert rw_obj.P_max == pytest.approx(0.25)


def test_rw_power_fault_all_and_invalid():
    scenario = run_faults([BSK_Faults.RwPowerFault(time=0, reducedLimit=0.2, wheelIdx="all")])
    assert scenario.get_DynModel().RW1.P_max == pytest.approx(0.2)
    assert scenario.get_DynModel().RW4.P_max == pytest.approx(0.2)

    with pytest.raises(ValueError):
        BSK_Faults.RwPowerFault(time=0, reducedLimit=0.2, wheelIdx=9)
    with pytest.raises(ValueError):
        BSK_Faults.RwPowerFault(time=0, reducedLimit=0.2, wheelIdx=4)


@pytest.mark.parametrize("wheel_idx,expected_rw", [(0, 1), (2, 3)])
def test_rw_friction_fault(wheel_idx, expected_rw):
    baseline = run_faults([]).get_DynModel()
    before = getattr(baseline, f"RW{expected_rw}").fCoulomb

    scenario = run_faults(
        [BSK_Faults.RwFrictionFault(time=0, frictionMultiplier=3.0, wheelIdx=wheel_idx)]
    )
    after = getattr(scenario.get_DynModel(), f"RW{expected_rw}").fCoulomb
    assert after == pytest.approx(before * 3.0)
    with pytest.raises(ValueError):
        BSK_Faults.RwFrictionFault(time=0, frictionMultiplier=3.0, wheelIdx=4)


def test_debris_impact_on_off():
    scenario = run_faults(
        [
            BSK_Faults.DebrisImpactOn(
                time=macros.sec2nano(0.0),
                debris_location=[0.0, 0.05, 0.0],
                direction=[0.0, -1.0, 0.0],
                mass=0.1,
                velocityMag=1000.0,
            ),
            BSK_Faults.DebrisImpactOff(time=macros.sec2nano(0.5)),
        ],
        stop_sec=1.0,
    )
    ext = scenario.get_DynModel().extForceTorqueObject
    assert np.allclose(ext.extForce_B, np.zeros(3))
    with pytest.raises(ValueError):
        BSK_Faults.DebrisImpactOn(
            time=0,
            debris_location=[0.0, 0.0],
            direction=[1.0, 0.0, 0.0],
            mass=0.1,
        )
    with pytest.raises(ValueError):
        BSK_Faults.DebrisImpactOn(
            time=0,
            debris_location=[0.0, 0.0, 0.0],
            direction=[0.0, 0.0, 0.0],
            mass=0.1,
        )


def test_css_signal_fault():
    scenario = run_faults(
        [BSK_Faults.CssSignalFault(time=0, type="CSSFAULT_OFF", cssIdx=[0, 2, 7])]
    )
    sensors = scenario.get_DynModel().CSSConstellationObject.sensorList
    assert sensors[0].faultState is not None
    assert sensors[2].faultState is not None
    assert sensors[7].faultState is not None
    with pytest.raises(ValueError):
        BSK_Faults.CssSignalFault(time=0, type="CSSFAULT_OFF", cssIdx=[8])
    with pytest.raises(ValueError):
        BSK_Faults.CssSignalFault(time=0, type="CSSFAULT_STUCK", cssIdx=[0])


@pytest.mark.parametrize("rw_idx,mapped", [(0, 0), (2, 2)])
def test_rw_bit_flip_fault(rw_idx, mapped):
    scenario = run_faults(
        [BSK_Faults.RWBitFlipFault(time=0, bitFlipIdx=0, RW_Idx=rw_idx, elementIdx=0)]
    )
    assert scenario.get_FswModel().rwMotorTorque.GsMatrix_B[mapped * 3] != 0.0
    with pytest.raises(ValueError):
        BSK_Faults.RWBitFlipFault(time=0, bitFlipIdx=0, RW_Idx=4, elementIdx=0)
    with pytest.raises(ValueError):
        BSK_Faults.RWBitFlipFault(time=0, bitFlipIdx=16, RW_Idx=0, elementIdx=0)
    with pytest.raises(ValueError):
        BSK_Faults.RWBitFlipFault(time=0, bitFlipIdx=0, RW_Idx=0, elementIdx=3)


def test_rw_gain_bit_flip_fault():
    baseline = run_faults([]).get_FswModel().mrpFeedbackRWs.K
    scenario = run_faults(
        [BSK_Faults.RWGainBitFlipFault(time=0, bitFlipIdx=0, gainType="K")]
    )
    assert scenario.get_FswModel().mrpFeedbackRWs.K != baseline
    with pytest.raises(ValueError):
        BSK_Faults.RWGainBitFlipFault(time=0, bitFlipIdx=16, gainType="K")


def test_magnetometer_fault_modes():
    scenario = run_faults(
        [
            BSK_Faults.MagnetometerFault(
                time=macros.sec2nano(0.0),
                faultType="MAG_FAULT_STUCK_VALUE",
                faultyAxis=0,
                stuckValue=1.23,
            ),
            BSK_Faults.MagnetometerFault(
                time=macros.sec2nano(0.5),
                faultType="MAG_FAULT_SPIKING",
                faultyAxis=1,
                spikeProbability=0.5,
                spikeAmount=9.9,
            ),
        ],
        stop_sec=1.0,
    )
    tam = scenario.get_DynModel().TAM
    assert np.allclose(as_flat_float_array(tam.stuckValue), [1.23, 1.23, 1.23])
    assert np.allclose(as_flat_float_array(tam.spikeProbability), [0.5, 0.5, 0.5])
    assert np.allclose(as_flat_float_array(tam.spikeAmount), [9.9, 9.9, 9.9])
    with pytest.raises(ValueError):
        BSK_Faults.MagnetometerFault(
            time=0,
            faultType="MAG_FAULT_STUCK_VALUE",
            faultyAxis=3,
            stuckValue=1.0,
        )
    with pytest.raises(ValueError):
        BSK_Faults.MagnetometerFault(
            time=0,
            faultType="MAG_FAULT_STUCK",
            faultyAxis=0,
            stuckValue=1.0,
        )


def test_mag_polar_noise_modes():
    scenario_noise = run_faults([BSK_Faults.MagPolarNoise(time=0, faultType="NOISE")])
    tam_noise = scenario_noise.get_DynModel().TAM
    assert as_flat_float_array(tam_noise.senNoiseStd).size == 3
    assert as_flat_float_array(tam_noise.walkBounds).size == 3

    scenario_spike = run_faults([BSK_Faults.MagPolarNoise(time=0, faultType="SPIKE")])
    tam_spike = scenario_spike.get_DynModel().TAM
    assert as_flat_float_array(tam_spike.spikeProbability)[0] > 0
    assert as_flat_float_array(tam_spike.spikeAmount)[0] > 0


def test_fixedframe2lla_basic_case():
    # Equator point on WGS84 x-axis should have near-zero latitude.
    lat, lon, _alt = vizSupport.fixedframe2lla(
        [6378137.0, 0.0, 0.0],
        orbitalMotion.REQ_EARTH * 1000.0,
        orbitalMotion.RP_EARTH / orbitalMotion.REQ_EARTH,
    )
    assert abs(float(lat)) < 1e-6
    assert abs(float(lon)) < 1e-6
