#
#  ISC License
#
#  Copyright (c) 2026,
#  Autonomous Vehicle Systems Laboratory, University of Colorado at Boulder
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

"""Python reimplementation of spacecraft charging equilibrium math for unit-test cross-checks."""

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Tuple
import math

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging
from Basilisk.simulation import spacecraftChargingEquilibrium
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


# Constants mirrored from `spacecraftChargingEquilibrium.h` defaults.
Q0 = 1.60217663e-19
TRAPZN = 1000
JPH = 40e-6
KTPH = 2.0
TSEE = 5.0
TEB = 20.0

DEFAULT_AREA = 50.264
DEFAULT_SUNLIT_AREA = 0.0
ROOT_BRACKET = (-400000.0, 400000.0)


@dataclass(frozen=True)
class BeamParameters:
    """Electron-beam parameters matching ElectronBeamMsgPayload fields."""

    energy_eb: float
    current_eb: float
    alpha_eb: float


class SpacecraftChargingEquilibriumCppReference:
    """Ground-truth reference implementation matching spacecraftChargingEquilibrium C++ math."""

    def __init__(
        self,
        energies: np.ndarray,
        electron_flux: np.ndarray,
        ion_flux: np.ndarray,
        yield_see_electron: np.ndarray,
        yield_see_ion: np.ndarray,
        yield_backscattered: np.ndarray
    ) -> None:
        self.energies = np.asarray(energies, dtype=float)
        self.electronFlux = np.asarray(electron_flux, dtype=float)
        self.ionFlux = np.asarray(ion_flux, dtype=float)
        self.yieldSEEelectron = np.asarray(yield_see_electron, dtype=float)
        self.yieldSEEion = np.asarray(yield_see_ion, dtype=float)
        self.yieldBackscattered = np.asarray(yield_backscattered, dtype=float)

    #  Numerical utilities
    @staticmethod
    def interp(x_vector: np.ndarray, y_vector: np.ndarray, x: float) -> float:
        """Linear interpolation with the same edge behavior as C++ interp()."""
        idx1 = -1
        for c in range(x_vector.size):
            if x_vector[c] > x:
                idx1 = c
                break

        if idx1 == -1:
            idx1 = x_vector.size - 1
        elif idx1 == 0:
            idx1 = 1

        ind_x0 = idx1 - 1
        ind_x1 = idx1
        y0 = y_vector[ind_x0]
        y1 = y_vector[ind_x1]
        return y0 + ((y1 - y0) / (x_vector[ind_x1] - x_vector[ind_x0])) * (x - x_vector[ind_x0])

    @staticmethod
    def trapz(f: Callable[[float], float], a: float, b: float, n: int) -> float:
        """Trapezoidal integration matching spacecraftChargingEquilibrium.cpp::trapz()."""
        if (not math.isfinite(a)) or (not math.isfinite(b)) or n <= 0 or b == a:
            return 0.0

        h = (b - a) / float(n)
        total = 0.0
        for i in range(1, n):
            total += f(a + i * h)
        return h * (0.5 * f(a) + total + 0.5 * f(b))

    def getFlux(self, energy: float, particle_type: str) -> float:
        if particle_type == "electron":
            flux = self.interp(self.energies, self.electronFlux, energy)
        elif particle_type == "ion":
            flux = self.interp(self.energies, self.ionFlux, energy)
        else:
            return math.nan
        return max(0.0, flux)

    def getYield(self, energy: float, yield_type: str) -> float:
        if yield_type == "electron":
            yield_value = self.interp(self.energies, self.yieldSEEelectron, energy)
        elif yield_type == "ion":
            yield_value = self.interp(self.energies, self.yieldSEEion, energy)
        elif yield_type == "backscattered":
            yield_value = self.interp(self.energies, self.yieldBackscattered, energy)
        else:
            return math.nan
        return max(0.0, yield_value)

    #  Current components
    def electronCurrent(self, phi: float, area: float) -> float:
        constant = -Q0 * area

        def integrand(energy: float) -> float:
            return (energy / (energy - phi)) * self.getFlux(energy - phi, "electron")

        if phi < 0.0:
            lower_bound = 0.1
            upper_bound = self.energies[-1]
        else:
            lower_bound = 0.1 + abs(phi)
            upper_bound = self.energies[-1] + abs(phi)

        integral = self.trapz(integrand, lower_bound, upper_bound, TRAPZN)
        return constant * integral

    def ionCurrent(self, phi: float, area: float) -> float:
        constant = Q0 * area

        def integrand(energy: float) -> float:
            return (energy / (energy + phi)) * self.getFlux(energy + phi, "ion")

        if phi > 0.0:
            lower_bound = 0.1
            upper_bound = self.energies[-1]
        else:
            lower_bound = 0.1 + abs(phi)
            upper_bound = self.energies[-1] + abs(phi)

        integral = self.trapz(integrand, lower_bound, upper_bound, TRAPZN)
        return constant * integral

    def SEEelectronCurrent(self, phi: float, area: float) -> float:
        constant = Q0 * area

        def integrand(energy: float) -> float:
            return self.getYield(energy, "electron") * (energy / (energy - phi)) * self.getFlux(energy - phi, "electron")

        if phi < 0.0:
            lower_bound = 0.1
            upper_bound = self.energies[-1]
        else:
            lower_bound = 0.1 + abs(phi)
            upper_bound = self.energies[-1] + abs(phi)

        integral = self.trapz(integrand, lower_bound, upper_bound, TRAPZN)
        i_see_e = constant * integral

        if phi <= 0.0:
            return i_see_e
        if phi > 0.0:
            return i_see_e * math.exp(-phi / TSEE)
        return math.nan

    def SEEionCurrent(self, phi: float, area: float) -> float:
        constant = Q0 * area

        def integrand(energy: float) -> float:
            return self.getYield(energy, "ion") * (energy / (energy + phi)) * self.getFlux(energy + phi, "ion")

        if phi > 0.0:
            lower_bound = 0.1
            upper_bound = self.energies[-1]
        else:
            lower_bound = 0.1 + abs(phi)
            upper_bound = self.energies[-1] + abs(phi)

        integral = self.trapz(integrand, lower_bound, upper_bound, TRAPZN)
        i_see_i = constant * integral

        if phi <= 0.0:
            return i_see_i
        if phi > 0.0:
            return i_see_i * math.exp(-phi / TSEE)
        return math.nan

    def backscatteringCurrent(self, phi: float, area: float) -> float:
        constant = Q0 * area

        def integrand(energy: float) -> float:
            return self.getYield(energy, "backscattered") * (energy / (energy - phi)) * self.getFlux(energy - phi, "electron")

        if phi < 0.0:
            lower_bound = 0.1
            upper_bound = self.energies[-1]
        else:
            lower_bound = 0.1 + abs(phi)
            upper_bound = self.energies[-1] + abs(phi)

        integral = self.trapz(integrand, lower_bound, upper_bound, TRAPZN)
        i_bs = constant * integral

        if phi <= 0.0:
            return i_bs
        if phi > 0.0:
            return i_bs * math.exp(-phi / TSEE)
        return math.nan

    @staticmethod
    def photoelectricCurrent(phi: float, area: float) -> float:
        if phi > 0.0:
            return JPH * area * math.exp(-phi / KTPH)
        if phi <= 0.0:
            return JPH * area
        return math.nan

    @staticmethod
    def electronBeamCurrent(phi_s: float, phi_t: float, craft_type: str, eeb: float, ieb: float, alpha_eb: float) -> float:
        if craft_type == "servicer":
            if eeb > (phi_s - phi_t):
                return ieb * (1.0 - math.exp(-(eeb - phi_s + phi_t) / TEB))
            return 0.0
        if craft_type == "target":
            if eeb > (phi_s - phi_t):
                return -alpha_eb * ieb * (1.0 - math.exp(-(eeb - phi_s + phi_t) / TEB))
            return 0.0
        return math.nan

    def SEEelectronBeamCurrent(self, phi_s: float, phi_t: float, eeb: float, ieb: float, alpha_eb: float) -> float:
        e_eff = eeb - phi_s + phi_t
        i_beam_target = self.electronBeamCurrent(phi_s, phi_t, "target", eeb, ieb, alpha_eb)
        incoming_beam_magnitude = max(0.0, -i_beam_target)
        return self.getYield(e_eff, "electron") * incoming_beam_magnitude

    def electronBeamBackscattering(self, phi_s: float, phi_t: float, eeb: float, ieb: float, alpha_eb: float) -> float:
        e_eff = eeb - phi_s + phi_t
        i_beam_target = self.electronBeamCurrent(phi_s, phi_t, "target", eeb, ieb, alpha_eb)
        incoming_beam_magnitude = max(0.0, -i_beam_target)
        return self.getYield(e_eff, "backscattered") * incoming_beam_magnitude

    #  Coupled current sums and root solve
    def sumCurrentsServicer(self, phi_s: float, area: float, sunlit_area: float, beam: BeamParameters) -> float:
        non_beam_currents = (
            self.electronCurrent(phi_s, area)
            + self.ionCurrent(phi_s, area)
            + self.SEEelectronCurrent(phi_s, area)
            + self.SEEionCurrent(phi_s, area)
            + self.backscatteringCurrent(phi_s, area)
            + self.photoelectricCurrent(phi_s, sunlit_area)
        )

        i_beam_s = self.electronBeamCurrent(phi_s, 0.0, "servicer", beam.energy_eb, beam.current_eb, beam.alpha_eb)
        return non_beam_currents + i_beam_s

    def sumCurrentsTarget(self, phi_t: float, phi_s: float, area: float, sunlit_area: float, beam: BeamParameters) -> float:
        non_beam_currents = (
            self.electronCurrent(phi_t, area)
            + self.ionCurrent(phi_t, area)
            + self.SEEelectronCurrent(phi_t, area)
            + self.SEEionCurrent(phi_t, area)
            + self.backscatteringCurrent(phi_t, area)
            + self.photoelectricCurrent(phi_t, sunlit_area)
        )

        i_beam_t = self.electronBeamCurrent(phi_s, phi_t, "target", beam.energy_eb, beam.current_eb, beam.alpha_eb)
        i_see_eb = self.SEEelectronBeamCurrent(phi_s, phi_t, beam.energy_eb, beam.current_eb, beam.alpha_eb)
        i_bs_eb = self.electronBeamBackscattering(phi_s, phi_t, beam.energy_eb, beam.current_eb, beam.alpha_eb)
        return non_beam_currents + i_beam_t + i_see_eb + i_bs_eb

    @staticmethod
    def bisectionSolve(interval: Tuple[float, float], accuracy: float, f: Callable[[float], float]) -> float:
        left, right = interval
        current_estimate = (left + right) / 2.0

        if left > right:
            return math.nan
        f_left = f(left)
        f_right = f(right)
        if f_left == 0.0:
            return left
        if f_right == 0.0:
            return right
        if (f_left * f_right) > 0.0:
            return math.nan

        while (right - left) >= accuracy:
            current_estimate = (left + right) / 2.0
            f_mid = f(current_estimate)
            if f_mid == 0.0:
                break
            if (f_left < 0.0 and f_mid > 0.0) or (f_left > 0.0 and f_mid < 0.0):
                right = current_estimate
            else:
                left = current_estimate
                f_left = f_mid

        return current_estimate

    def solveCoupledEquilibrium(
        self,
        beam: BeamParameters,
        servicer_area: float = DEFAULT_AREA,
        target_area: float = DEFAULT_AREA,
        servicer_sunlit_area: float = DEFAULT_SUNLIT_AREA,
        target_sunlit_area: float = DEFAULT_SUNLIT_AREA
    ) -> Tuple[float, float]:
        phi_s = self.bisectionSolve(
            ROOT_BRACKET,
            1e-6,
            lambda phi: self.sumCurrentsServicer(phi, servicer_area, servicer_sunlit_area, beam)
        )

        phi_t = self.bisectionSolve(
            ROOT_BRACKET,
            1e-8,
            lambda phi: self.sumCurrentsTarget(phi, phi_s, target_area, target_sunlit_area, beam)
        )

        return phi_s, phi_t


def _support_dir() -> Path:
    return Path(__file__).resolve().parent / "Support"


def _load_support_data() -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    support = _support_dir()
    energies = np.loadtxt(support / "particleEnergies.txt", delimiter=",")
    electron_flux = np.loadtxt(support / "electronFlux.txt", delimiter=",")
    ion_flux = np.loadtxt(support / "ionFlux.txt", delimiter=",")
    yield_see_electron = np.loadtxt(support / "yieldSEEelectron.txt", delimiter=",")
    yield_see_ion = np.loadtxt(support / "yieldSEEion.txt", delimiter=",")
    yield_backscattered = np.loadtxt(support / "yieldBackscattered.txt", delimiter=",")
    return energies, electron_flux, ion_flux, yield_see_electron, yield_see_ion, yield_backscattered


def _read_currents_out_messages(module):
    """Read servicer/target current breakdown messages if SWIG exposes them as a vector wrapper."""
    if not hasattr(module, "currentsOutMsgs"):
        raise AttributeError("missing currentsOutMsgs attribute")

    out_msgs = module.currentsOutMsgs

    # Preferred path: vector wrapper with Python indexing.
    try:
        currents_servicer = out_msgs[0].read()
        currents_target = out_msgs[1].read()
        return currents_servicer, currents_target
    except Exception:
        pass

    # Fallback path in case wrapper exposes C++ vector .at().
    try:
        currents_servicer = out_msgs.at(0).read()
        currents_target = out_msgs.at(1).read()
        return currents_servicer, currents_target
    except Exception as exc:
        raise TypeError(
            "currentsOutMsgs is present but not readable as a vector of output messages"
        ) from exc


def _snapshot_currents_payload(currents_msg):
    """Copy a SWIG payload object into plain Python floats before module teardown."""
    return {
        "i_e": float(currents_msg.electronCurrent),
        "i_i": float(currents_msg.ionCurrent),
        "i_see_e": float(currents_msg.seeElectronCurrent),
        "i_see_i": float(currents_msg.seeIonCurrent),
        "i_bs": float(currents_msg.backscatteringCurrent),
        "i_ph": float(currents_msg.photoelectricCurrent),
        "env_i": float(currents_msg.nonBeamCurrent),
        "beam_arrival_i": float(currents_msg.beamCurrent),
        "beam_see_i": float(currents_msg.beamSeeCurrent),
        "beam_bs_i": float(currents_msg.beamBackscatteringCurrent),
        "total_i": float(currents_msg.totalCurrent)
    }


def _run_cpp_module_once(
    energies: np.ndarray,
    electron_flux: np.ndarray,
    ion_flux: np.ndarray,
    yield_see_electron: np.ndarray,
    yield_see_ion: np.ndarray,
    yield_backscattered: np.ndarray,
    beam: BeamParameters,
    servicer_sunlit_area: Optional[float] = None,
    target_sunlit_area: Optional[float] = None,
    return_currents: bool = False
) -> Tuple:
    sim = SimulationBaseClass.SimBaseClass()
    proc_name = "chargeProc"
    task_name = "unitTask"
    dt = 0.5

    proc = sim.CreateNewProcess(proc_name)
    proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))

    module = spacecraftChargingEquilibrium.SpacecraftChargingEquilibrium()
    module.ModelTag = "SpacecraftChargingEquilibrium_CPP"
    sim.AddModelToTask(task_name, module)

    # Required indexing convention in the C++ module:
    # index 0 = servicer, index 1 = target
    lt = 12.0
    angle = lt * 360.0 / 24.0 * math.pi / 180.0 - math.pi
    orbit_radius = 42000.0e3

    r_target = np.array([orbit_radius * math.cos(angle), orbit_radius * math.sin(angle), 0.0])
    r_servicer = r_target + np.array([0.0, 10.0, 0.0])

    sc0_state = messaging.SCStatesMsgPayload()
    sc0_state.r_BN_N = r_servicer
    sc0_msg = messaging.SCStatesMsg().write(sc0_state)

    sc1_state = messaging.SCStatesMsgPayload()
    sc1_state.r_BN_N = r_target
    sc1_msg = messaging.SCStatesMsg().write(sc1_state)

    module.addSpacecraft(sc0_msg)
    module.addSpacecraft(sc1_msg)

    plasma_payload = messaging.PlasmaFluxMsgPayload()
    plasma_payload.energies = energies
    plasma_payload.meanElectronFlux = electron_flux
    plasma_payload.meanIonFlux = ion_flux
    plasma_msg = messaging.PlasmaFluxMsg().write(plasma_payload)
    module.plasmaFluxInMsg.subscribeTo(plasma_msg)

    module.setYieldSEEelectron(yield_see_electron)
    module.setYieldSEEion(yield_see_ion)
    module.setYieldBackscattered(yield_backscattered)

    beam_payload = messaging.ElectronBeamMsgPayload()
    beam_payload.energyEB = beam.energy_eb
    beam_payload.currentEB = beam.current_eb
    beam_payload.alphaEB = beam.alpha_eb
    beam_msg = messaging.ElectronBeamMsg().write(beam_payload)
    module.eBeamInMsgs[0].subscribeTo(beam_msg)

    if servicer_sunlit_area is not None:
        sc0_sunlit_payload = messaging.ScSunlitFacetAreaMsgPayload()
        sc0_sunlit_payload.area = servicer_sunlit_area
        sc0_sunlit_msg = messaging.ScSunlitFacetAreaMsg().write(sc0_sunlit_payload)
        module.scSunlitAreaInMsgs[0].subscribeTo(sc0_sunlit_msg)

    if target_sunlit_area is not None:
        sc1_sunlit_payload = messaging.ScSunlitFacetAreaMsgPayload()
        sc1_sunlit_payload.area = target_sunlit_area
        sc1_sunlit_msg = messaging.ScSunlitFacetAreaMsg().write(sc1_sunlit_payload)
        module.scSunlitAreaInMsgs[1].subscribeTo(sc1_sunlit_msg)

    sim.InitializeSimulation()
    sim.TotalSim.SingleStepProcesses()

    phi_servicer = module.voltOutMsgs[0].read().voltage
    phi_target = module.voltOutMsgs[1].read().voltage
    if return_currents:
        try:
            currents_servicer, currents_target = _read_currents_out_messages(module)
        except (AttributeError, TypeError) as exc:
            pytest.fail(
                "spacecraftChargingEquilibrium Python bindings do not expose usable currentsOutMsgs "
                f"({exc}). Rebuild Basilisk/SWIG wrappers for ScChargingCurrentsMsgPayload.",
                pytrace=False
            )
        currents_servicer = _snapshot_currents_payload(currents_servicer)
        currents_target = _snapshot_currents_payload(currents_target)
        return phi_servicer, phi_target, currents_servicer, currents_target
    return phi_servicer, phi_target


def _run_cpp_module_one_spacecraft_once(
    energies: np.ndarray,
    electron_flux: np.ndarray,
    ion_flux: np.ndarray,
    yield_see_electron: np.ndarray,
    yield_see_ion: np.ndarray,
    yield_backscattered: np.ndarray,
    beam: BeamParameters
) -> float:
    """Run module with one spacecraft to exercise exact-two-spacecraft guard behavior."""
    sim = SimulationBaseClass.SimBaseClass()
    proc_name = "chargeProcOneSc"
    task_name = "unitTaskOneSc"
    dt = 0.5

    proc = sim.CreateNewProcess(proc_name)
    proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))

    module = spacecraftChargingEquilibrium.SpacecraftChargingEquilibrium()
    module.ModelTag = "SpacecraftChargingEquilibrium_CPP_OneSc"
    sim.AddModelToTask(task_name, module)

    lt = 12.0
    angle = lt * 360.0 / 24.0 * math.pi / 180.0 - math.pi
    orbit_radius = 42000.0e3
    r_servicer = np.array([orbit_radius * math.cos(angle), orbit_radius * math.sin(angle), 0.0])

    sc0_state = messaging.SCStatesMsgPayload()
    sc0_state.r_BN_N = r_servicer
    sc0_msg = messaging.SCStatesMsg().write(sc0_state)
    module.addSpacecraft(sc0_msg)

    plasma_payload = messaging.PlasmaFluxMsgPayload()
    plasma_payload.energies = energies
    plasma_payload.meanElectronFlux = electron_flux
    plasma_payload.meanIonFlux = ion_flux
    plasma_msg = messaging.PlasmaFluxMsg().write(plasma_payload)
    module.plasmaFluxInMsg.subscribeTo(plasma_msg)

    module.setYieldSEEelectron(yield_see_electron)
    module.setYieldSEEion(yield_see_ion)
    module.setYieldBackscattered(yield_backscattered)

    beam_payload = messaging.ElectronBeamMsgPayload()
    beam_payload.energyEB = beam.energy_eb
    beam_payload.currentEB = beam.current_eb
    beam_payload.alphaEB = beam.alpha_eb
    beam_msg = messaging.ElectronBeamMsg().write(beam_payload)
    module.eBeamInMsgs[0].subscribeTo(beam_msg)

    sim.InitializeSimulation()
    sim.TotalSim.SingleStepProcesses()
    return module.voltOutMsgs[0].read().voltage


def _python_target_breakdown(
    reference: SpacecraftChargingEquilibriumCppReference,
    phi_t: float,
    phi_s: float,
    beam: BeamParameters,
    area: float = DEFAULT_AREA,
    sunlit_area: float = DEFAULT_SUNLIT_AREA
):
    i_e = reference.electronCurrent(phi_t, area)
    i_i = reference.ionCurrent(phi_t, area)
    i_see_e = reference.SEEelectronCurrent(phi_t, area)
    i_see_i = reference.SEEionCurrent(phi_t, area)
    i_bs = reference.backscatteringCurrent(phi_t, area)
    i_ph = reference.photoelectricCurrent(phi_t, sunlit_area)
    env_i = i_e + i_i + i_see_e + i_see_i + i_bs + i_ph

    beam_arrival_i = reference.electronBeamCurrent(
        phi_s, phi_t, "target", beam.energy_eb, beam.current_eb, beam.alpha_eb
    )
    beam_see_i = reference.SEEelectronBeamCurrent(
        phi_s, phi_t, beam.energy_eb, beam.current_eb, beam.alpha_eb
    )
    beam_bs_i = reference.electronBeamBackscattering(
        phi_s, phi_t, beam.energy_eb, beam.current_eb, beam.alpha_eb
    )

    return {
        "i_e": i_e,
        "i_i": i_i,
        "i_see_e": i_see_e,
        "i_see_i": i_see_i,
        "i_bs": i_bs,
        "i_ph": i_ph,
        "env_i": env_i,
        "beam_arrival_i": beam_arrival_i,
        "beam_see_i": beam_see_i,
        "beam_bs_i": beam_bs_i,
        "total_i": env_i + beam_arrival_i + beam_see_i + beam_bs_i
    }


def _cpp_currents_payload_to_dict(currents_msg):
    if isinstance(currents_msg, dict):
        return currents_msg
    return {
        "i_e": currents_msg.electronCurrent,
        "i_i": currents_msg.ionCurrent,
        "i_see_e": currents_msg.seeElectronCurrent,
        "i_see_i": currents_msg.seeIonCurrent,
        "i_bs": currents_msg.backscatteringCurrent,
        "i_ph": currents_msg.photoelectricCurrent,
        "env_i": currents_msg.nonBeamCurrent,
        "beam_arrival_i": currents_msg.beamCurrent,
        "beam_see_i": currents_msg.beamSeeCurrent,
        "beam_bs_i": currents_msg.beamBackscatteringCurrent,
        "total_i": currents_msg.totalCurrent
    }


@pytest.fixture(scope="module")
def support_data() -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    return _load_support_data()


def _compare_python_reference_to_cpp(
    support_data: Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray],
    beam: BeamParameters,
    scale_electron_flux: float = 1.0,
    scale_ion_flux: float = 1.0,
    servicer_sunlit_area: float = DEFAULT_SUNLIT_AREA,
    target_sunlit_area: float = DEFAULT_SUNLIT_AREA,
    atol: float = 1e-3
) -> Tuple[float, float, float, float]:
    energies, electron_flux, ion_flux, yield_see_electron, yield_see_ion, yield_backscattered = support_data
    scaled_electron_flux = electron_flux * scale_electron_flux
    scaled_ion_flux = ion_flux * scale_ion_flux

    phi_servicer_cpp, phi_target_cpp = _run_cpp_module_once(
        energies,
        scaled_electron_flux,
        scaled_ion_flux,
        yield_see_electron,
        yield_see_ion,
        yield_backscattered,
        beam,
        servicer_sunlit_area=servicer_sunlit_area,
        target_sunlit_area=target_sunlit_area
    )

    reference = SpacecraftChargingEquilibriumCppReference(
        energies,
        scaled_electron_flux,
        scaled_ion_flux,
        yield_see_electron,
        yield_see_ion,
        yield_backscattered
    )
    phi_servicer_py, phi_target_py = reference.solveCoupledEquilibrium(
        beam,
        servicer_sunlit_area=servicer_sunlit_area,
        target_sunlit_area=target_sunlit_area
    )

    assert np.isfinite(phi_servicer_cpp)
    assert np.isfinite(phi_target_cpp)
    assert np.isfinite(phi_servicer_py)
    assert np.isfinite(phi_target_py)

    np.testing.assert_allclose(
        np.array([phi_servicer_cpp, phi_target_cpp]),
        np.array([phi_servicer_py, phi_target_py]),
        rtol=0.0,
        atol=atol
    )
    return phi_servicer_cpp, phi_target_cpp, phi_servicer_py, phi_target_py


def test_python_reimplementation_matches_cpp_module(support_data):
    """Baseline cross-check under nominal plasma and beam settings."""
    beam = BeamParameters(energy_eb=33000.0, current_eb=4.5e-6, alpha_eb=1.0)
    _compare_python_reference_to_cpp(
        support_data=support_data,
        beam=beam
    )


@pytest.mark.parametrize("scale_electron_flux, scale_ion_flux", [
    (0.5, 1.0),
    (1.0, 1.0),
    (2.0, 1.0),
    (1.0, 0.5),
    (1.0, 2.0)
])
def test_cpp_vs_python_plasma_scaling_cases(support_data, scale_electron_flux, scale_ion_flux):
    """Compare C++ and Python equilibrium outputs across plasma scaling cases."""
    beam = BeamParameters(energy_eb=33000.0, current_eb=4.5e-6, alpha_eb=1.0)
    _compare_python_reference_to_cpp(
        support_data=support_data,
        beam=beam,
        scale_electron_flux=scale_electron_flux,
        scale_ion_flux=scale_ion_flux
    )


def test_plasma_scaling_trends_target_potential(support_data):
    """Verify target-potential trend ordering across electron/ion flux scaling."""
    beam = BeamParameters(energy_eb=33000.0, current_eb=4.5e-6, alpha_eb=1.0)

    _, phi_t_base_cpp, _, phi_t_base_py = _compare_python_reference_to_cpp(
        support_data=support_data,
        beam=beam,
        scale_electron_flux=1.0,
        scale_ion_flux=1.0
    )

    _, phi_t_e_more_cpp, _, phi_t_e_more_py = _compare_python_reference_to_cpp(
        support_data=support_data,
        beam=beam,
        scale_electron_flux=2.0,
        scale_ion_flux=1.0
    )
    _, phi_t_e_less_cpp, _, phi_t_e_less_py = _compare_python_reference_to_cpp(
        support_data=support_data,
        beam=beam,
        scale_electron_flux=0.5,
        scale_ion_flux=1.0
    )

    _, phi_t_i_more_cpp, _, phi_t_i_more_py = _compare_python_reference_to_cpp(
        support_data=support_data,
        beam=beam,
        scale_electron_flux=1.0,
        scale_ion_flux=2.0
    )
    _, phi_t_i_less_cpp, _, phi_t_i_less_py = _compare_python_reference_to_cpp(
        support_data=support_data,
        beam=beam,
        scale_electron_flux=1.0,
        scale_ion_flux=0.5
    )

    # More ambient electrons => more negative target potential.
    assert phi_t_e_more_cpp < phi_t_base_cpp < phi_t_e_less_cpp
    assert phi_t_e_more_py < phi_t_base_py < phi_t_e_less_py

    # More ambient ions => more positive target potential.
    assert phi_t_i_less_cpp < phi_t_base_cpp < phi_t_i_more_cpp
    assert phi_t_i_less_py < phi_t_base_py < phi_t_i_more_py


def test_cpp_vs_python_zero_padded_plasma_bins(support_data):
    """Ensure trailing zero-filled plasma bins are ignored consistently."""
    energies, electron_flux, ion_flux, yield_see_electron, yield_see_ion, yield_backscattered = support_data
    beam = BeamParameters(energy_eb=33000.0, current_eb=4.5e-6, alpha_eb=1.0)

    valid_bins = 30
    energies_padded = np.zeros_like(energies)
    electron_flux_padded = np.zeros_like(electron_flux)
    ion_flux_padded = np.zeros_like(ion_flux)
    energies_padded[:valid_bins] = energies[:valid_bins]
    electron_flux_padded[:valid_bins] = electron_flux[:valid_bins]
    ion_flux_padded[:valid_bins] = ion_flux[:valid_bins]

    phi_servicer_cpp, phi_target_cpp = _run_cpp_module_once(
        energies_padded,
        electron_flux_padded,
        ion_flux_padded,
        yield_see_electron,
        yield_see_ion,
        yield_backscattered,
        beam
    )

    reference = SpacecraftChargingEquilibriumCppReference(
        energies[:valid_bins],
        electron_flux[:valid_bins],
        ion_flux[:valid_bins],
        yield_see_electron[:valid_bins],
        yield_see_ion[:valid_bins],
        yield_backscattered[:valid_bins]
    )
    phi_servicer_py, phi_target_py = reference.solveCoupledEquilibrium(beam)

    np.testing.assert_allclose(
        np.array([phi_servicer_cpp, phi_target_cpp]),
        np.array([phi_servicer_py, phi_target_py]),
        rtol=0.0,
        atol=1e-3
    )


@pytest.mark.parametrize("energy_eb, current_eb, alpha_eb", [
    (22000.0, 2.0e-6, 0.8),
    (33000.0, 4.5e-6, 1.0),
    (45000.0, 7.0e-6, 0.6),
    (40000.0, 5.0e-4, 0.95),
    (33000.0, 0.0, 1.0)
])
def test_cpp_vs_python_beam_parameter_cases(support_data, energy_eb, current_eb, alpha_eb):
    """Compare C++ and Python equilibrium outputs across beam parameter cases."""
    beam = BeamParameters(energy_eb=energy_eb, current_eb=current_eb, alpha_eb=alpha_eb)
    _compare_python_reference_to_cpp(
        support_data=support_data,
        beam=beam
    )


def test_cpp_requires_exactly_two_spacecraft(support_data):
    """Single-spacecraft run should fail reset with the exact-two-spacecraft precondition."""
    energies, electron_flux, ion_flux, yield_see_electron, yield_see_ion, yield_backscattered = support_data
    beam = BeamParameters(energy_eb=33000.0, current_eb=4.5e-6, alpha_eb=1.0)

    with pytest.raises(bskLogging.BasiliskError, match=r"requires exactly 2 spacecraft"):
        _run_cpp_module_one_spacecraft_once(
            energies,
            electron_flux,
            ion_flux,
            yield_see_electron,
            yield_see_ion,
            yield_backscattered,
            beam
        )


@pytest.mark.parametrize("servicer_sunlit_area, target_sunlit_area", [
    (5.0, 10.0),
    (15.0, 25.132),
    (25.132, 40.0)
])
def test_cpp_vs_python_sunlit_area_override_cases(support_data, servicer_sunlit_area, target_sunlit_area):
    """Compare C++ and Python equilibrium outputs when sunlit areas are overridden."""
    beam = BeamParameters(energy_eb=33000.0, current_eb=4.5e-6, alpha_eb=1.0)
    _compare_python_reference_to_cpp(
        support_data=support_data,
        beam=beam,
        servicer_sunlit_area=servicer_sunlit_area,
        target_sunlit_area=target_sunlit_area
    )


@pytest.mark.parametrize("energy_eb, current_eb, alpha_eb", [
    (22000.0, 2.0e-6, 0.8),
    (33000.0, 4.5e-6, 1.0),
    (45000.0, 7.0e-6, 0.6)
])
def test_target_current_breakdown_cpp_vs_python_equilibrium_beam_cases(
    support_data,
    energy_eb,
    current_eb,
    alpha_eb
):
    """Compare C++ current-message target terms to Python terms at equilibrium."""
    energies, electron_flux, ion_flux, yield_see_electron, yield_see_ion, yield_backscattered = support_data
    beam = BeamParameters(energy_eb=energy_eb, current_eb=current_eb, alpha_eb=alpha_eb)

    phi_servicer_cpp, phi_target_cpp, _, currents_target_cpp = _run_cpp_module_once(
        energies,
        electron_flux,
        ion_flux,
        yield_see_electron,
        yield_see_ion,
        yield_backscattered,
        beam,
        return_currents=True
    )

    reference = SpacecraftChargingEquilibriumCppReference(
        energies,
        electron_flux,
        ion_flux,
        yield_see_electron,
        yield_see_ion,
        yield_backscattered
    )
    py = _python_target_breakdown(
        reference=reference,
        phi_t=phi_target_cpp,
        phi_s=phi_servicer_cpp,
        beam=beam
    )
    cpp = _cpp_currents_payload_to_dict(currents_target_cpp)

    np.testing.assert_allclose(cpp["i_e"], py["i_e"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["i_i"], py["i_i"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["i_see_e"], py["i_see_e"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["i_see_i"], py["i_see_i"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["i_bs"], py["i_bs"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["i_ph"], py["i_ph"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["env_i"], py["env_i"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["beam_arrival_i"], py["beam_arrival_i"], rtol=0.0, atol=1e-9)
    np.testing.assert_allclose(cpp["beam_see_i"], py["beam_see_i"], rtol=0.0, atol=1e-9)
    np.testing.assert_allclose(cpp["beam_bs_i"], py["beam_bs_i"], rtol=0.0, atol=1e-9)
    np.testing.assert_allclose(cpp["total_i"], py["total_i"], rtol=0.0, atol=1e-6)

    # Sign sanity for target beam terms:
    # incoming beam is non-positive, emitted SEE/backscatter are non-negative.
    assert cpp["beam_arrival_i"] <= 1e-12
    assert cpp["beam_see_i"] >= -1e-12
    assert cpp["beam_bs_i"] >= -1e-12


@pytest.mark.parametrize("target_sunlit_area", [10.0, 25.132, 40.0])
def test_target_current_breakdown_cpp_vs_python_near_equilibrium(
    support_data,
    target_sunlit_area
):
    """Compare C++ current-message target terms across sunlit-area settings."""
    energies, electron_flux, ion_flux, yield_see_electron, yield_see_ion, yield_backscattered = support_data
    beam = BeamParameters(energy_eb=33000.0, current_eb=4.5e-6, alpha_eb=1.0)

    phi_servicer_cpp, phi_target_cpp, _, currents_target_cpp = _run_cpp_module_once(
        energies,
        electron_flux,
        ion_flux,
        yield_see_electron,
        yield_see_ion,
        yield_backscattered,
        beam,
        target_sunlit_area=target_sunlit_area,
        return_currents=True
    )

    reference = SpacecraftChargingEquilibriumCppReference(
        energies,
        electron_flux,
        ion_flux,
        yield_see_electron,
        yield_see_ion,
        yield_backscattered
    )
    py = _python_target_breakdown(
        reference=reference,
        phi_t=phi_target_cpp,
        phi_s=phi_servicer_cpp,
        beam=beam,
        sunlit_area=target_sunlit_area
    )
    cpp = _cpp_currents_payload_to_dict(currents_target_cpp)

    np.testing.assert_allclose(cpp["i_e"], py["i_e"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["i_i"], py["i_i"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["i_see_e"], py["i_see_e"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["i_see_i"], py["i_see_i"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["i_bs"], py["i_bs"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["i_ph"], py["i_ph"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["env_i"], py["env_i"], rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(cpp["beam_arrival_i"], py["beam_arrival_i"], rtol=0.0, atol=1e-9)
    np.testing.assert_allclose(cpp["beam_see_i"], py["beam_see_i"], rtol=0.0, atol=1e-9)
    np.testing.assert_allclose(cpp["beam_bs_i"], py["beam_bs_i"], rtol=0.0, atol=1e-9)
    np.testing.assert_allclose(cpp["total_i"], py["total_i"], rtol=0.0, atol=1e-6)

    assert cpp["beam_arrival_i"] <= 1e-12
    assert cpp["beam_see_i"] >= -1e-12
    assert cpp["beam_bs_i"] >= -1e-12


if __name__ == "__main__":
    pytest.main([__file__])
