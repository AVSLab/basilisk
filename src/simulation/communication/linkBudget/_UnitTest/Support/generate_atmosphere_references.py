# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Reproduce the atmospheric integration references in ``test_linkBudget.py``.

Run this file with a Python environment containing Basilisk and NumPy. It
prints the gas-table sources and SHA-256 hashes, a 1 m versus 0.5 m convergence
report, and replacement ``ATMOSPHERE_ABSOLUTE_CASES`` entries. ``--check`` also
verifies committed values without importing or running simulation tests.

The NumPy formulas below intentionally reproduce ``ItuRefAtmosphere.cpp`` and
the spectral formulas in ``linkBudget.cpp`` at PR #1648 commit ``466fb30``.
They preserve the existing pressure branches, altitude conversion, water-vapor
floor and per-layer sine clamp of 0.0872. They do not call the production grid
builder or attenuation integrator. This is a numerical integration reference
for the current model, not independent validation of ITU physical fidelity.
"""

import argparse
import ast
import hashlib
import json
from pathlib import Path

import numpy as np

from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path


EARTH_RADIUS = 6378136.6  # [m] astroConstants.h, REQ_EARTH
ITU_RADIUS = 6356.766  # [km] ItuRefAtmosphere.h
TOP_ALTITUDE = 100000.0  # [m]
MIN_SIN_ELEVATION = 0.0872  # [-] Preserve the rounded production clamp.
CONVERGENCE_TOLERANCE = 1.0e-5  # [dB]
REFERENCE_TOLERANCE = 1.0e-8  # [dB] Allow numerical rounding.
GROUND_ALTITUDES = (0.0, 1500.0)  # [m]
ELEVATIONS = (2.0, 4.9, 5.0, 5.1, 10.0, 90.0)  # [deg]
# Columns: frequency [Hz], production-grid absolute tolerance [dB].
FREQUENCIES = (
    (10.0e9, 1.0e-4),
    (60.0e9, 0.05),
    (118.750334e9, 0.02),
    (183.310091e9, 0.02),
)


def atmosphere(altitude):
    """Evaluate the current piecewise atmosphere independently in NumPy.

    :param altitude: One-dimensional array of altitudes in meters.
    :return: Temperature in kelvin, pressure in hectopascals, and water-vapor
        partial pressure in hectopascals, as arrays.
    """
    height = np.asarray(altitude, dtype=float)*1.0e-3  # [km]
    boundaries = np.array([0, 11, 20, 32, 47, 51, 71, 84.852])  # [km]
    gradients = np.array([-6.5, 0, 1, 2.8, 0, -2.8, -2, 0])  # [K/km]
    base_temperatures = np.concatenate((
        [288.15], 288.15 + np.cumsum(np.diff(boundaries)*gradients[:-1])
    ))  # [K]
    layer = np.searchsorted(boundaries, height, side="right") - 1
    temperature = (
        base_temperatures[layer]
        + gradients[layer]*(height - boundaries[layer])
    )  # [K]
    geometric_height = ITU_RADIUS*height/(ITU_RADIUS - height)  # [km]
    upper = layer == 7
    temperature[upper] = 186.8673  # [K]
    rising = upper & (
        (geometric_height >= 91.0) & (geometric_height < 100.0)
    )  # [km] Geometric-height limits.
    temperature[rising] = 263.1905 - 76.3232*np.sqrt(
        1 - ((geometric_height[rising] - 91.0)/19.9429)**2
    )  # [K]; geometric-height coefficients in [km].
    temperature[upper & (geometric_height >= 100.0)] = 195.08134  # [K]

    pressure = np.zeros_like(height)  # [hPa]
    pressure_constant = 34.1632  # [K/km]
    # Preserve the production branch signs, base pressures [hPa], temperatures
    # [K], heights [km], and empirical inverse scale heights [1/km].
    for index in range(7):
        selected = layer == index
        local_height = height[selected]  # [km]
        gradient = gradients[index]  # [K/km]
        if index == 0:
            pressure[selected] = 1013.25*(
                288.15/(288.15 - gradient*local_height)
            )**(-pressure_constant/gradient)  # [hPa]
        elif index == 1:
            pressure[selected] = 226.3226*np.exp(
                -0.157688*(local_height - 11.0)
            )  # [hPa]
        elif index == 2:
            pressure[selected] = 54.7498*(
                216.65/(216.65 + local_height - 20.0)
            )**(-pressure_constant)  # [hPa]
        elif index == 3:
            pressure[selected] = 8.680422*(
                228.65/(228.65 + gradient*(local_height - 32.0))
            )**(-pressure_constant/gradient)  # [hPa]
        elif index == 4:
            pressure[selected] = 1.109106*np.exp(
                -0.126226*(local_height - 47.0)
            )  # [hPa]
        elif index == 5:
            pressure[selected] = 0.6694167*(
                270.65/(270.65 - gradient*(local_height - 51.0))
            )**(-pressure_constant/gradient)  # [hPa]
        else:
            pressure[selected] = 0.03956649*(
                214.65/(214.65 - gradient*(local_height - 71.0))
            )**(-pressure_constant/gradient)  # [hPa]
    selected = upper & (geometric_height < 100.0)  # [km]
    upper_height = geometric_height[selected]  # [km]
    pressure[selected] = np.exp(
        95.571899 - 4.011801*upper_height + 6.424731e-2*upper_height**2
        - 4.789660e-4*upper_height**3 + 1.340543e-6*upper_height**4
    )  # [hPa]; empirical polynomial uses geometric height in [km].

    density = 7.5*np.exp(-geometric_height/2.0)  # [g/m^3]; scale height [km].
    vapor_pressure = density*temperature/216.7  # [hPa]
    vapor_pressure = np.maximum(vapor_pressure, 2.0e-6*pressure)  # [hPa]
    return temperature, pressure, vapor_pressure


def refractive_index(temperature, pressure, vapor_pressure):
    """Evaluate the refractive index used by the production model.

    :param temperature: Temperature in kelvin.
    :param pressure: Atmospheric pressure in hectopascals.
    :param vapor_pressure: Water-vapor partial pressure in hectopascals.
    :return: Dimensionless refractive index.
    """
    return 1 + 1.0e-6*(
        77.6*pressure/temperature - 5.6*vapor_pressure/temperature
        + 3.75e5*vapor_pressure/temperature**2
    )


def specific_attenuation(frequency, properties, oxygen, water_vapor):
    """Evaluate the existing gas spectra at each reference-grid midpoint.

    :param frequency: Radio frequency in hertz.
    :param properties: Arrays returned by :func:`atmosphere`.
    :param oxygen: Oxygen line table; frequency column in gigahertz.
    :param water_vapor: Water-vapor line table; frequency column in gigahertz.
    :return: Specific attenuation array in decibels per kilometer.
    """
    temperature, pressure, vapor_pressure = properties
    frequency_ghz = frequency/1.0e9  # [GHz]
    theta = 300.0/temperature  # [-]; reference temperature [K].
    oxygen_spectrum = np.zeros_like(temperature)
    water_spectrum = np.zeros_like(temperature)
    # Empirical P.676 coefficients use pressure in hPa and frequency in GHz,
    # matching linkBudget.cpp, including its current water-vapor line shape.
    for line_frequency, a1, a2, a3, a4, a5, a6 in oxygen:
        strength = a1*1.0e-7*pressure*theta**3*np.exp(a2*(1 - theta))
        width = a3*1.0e-4*(
            pressure*theta**(0.8 - a4) + 1.1*vapor_pressure*theta
        )  # [GHz]
        width = np.sqrt(width**2 + 2.25e-6)  # [GHz]
        correction = (
            (a5 + a6*theta)*1.0e-4*(pressure + vapor_pressure)*theta**0.8
        )
        minus = line_frequency - frequency_ghz  # [GHz]
        plus = line_frequency + frequency_ghz  # [GHz]
        shape = frequency_ghz/line_frequency*(
            (width - correction*minus)/(minus**2 + width**2)
            + (width - correction*plus)/(plus**2 + width**2)
        )
        oxygen_spectrum += strength*shape
    for line_frequency, b1, b2, b3, b4, b5, b6 in water_vapor:
        strength = b1*1.0e-1*vapor_pressure*theta**3.5*np.exp(b2*(1 - theta))
        width = b3*1.0e-4*(
            pressure*theta**b4 + b5*vapor_pressure*theta**b6
        )  # [GHz]
        width = 0.535*width + np.sqrt(
            0.217*width**2 + 2.1316e-12*line_frequency**2/theta
        )  # [GHz]
        shape = frequency_ghz/line_frequency*2*width/(
            (line_frequency - frequency_ghz)**2 + width**2
        )
        water_spectrum += strength*shape
    debye_width = 5.6e-4*(pressure + vapor_pressure)*theta**0.8  # [GHz]
    continuum = frequency_ghz*pressure*theta**2*(
        6.14e-5/(debye_width*(1 + frequency_ghz**2/debye_width**2))
        + 1.4e-12*pressure*theta**1.5/(1 + 1.9e-5*frequency_ghz**1.5)
    )
    return 0.1820*frequency_ghz*(
        oxygen_spectrum + water_spectrum + continuum
    )  # [dB/km]


def load_lines(data_file):
    """Load a registered gas table and print its provenance.

    :param data_file: Basilisk support-data identifier.
    :return: Numeric spectral table in the JSON column order.
    """
    path = Path(get_path(data_file))
    contents = path.read_bytes()
    table = json.loads(contents)
    digest = hashlib.sha256(contents).hexdigest()
    print(f"# {path.name}: {table['source']}; SHA-256 {digest}")
    return np.array([
        [row[column] for column in table["columns"]] for row in table["data"]
    ])


def integrate_grid(ground_altitude, step, oxygen, water_vapor):
    """Integrate all frequencies and elevations on an independent uniform grid.

    :param ground_altitude: Ground altitude in meters.
    :param step: Maximum midpoint-cell width in meters.
    :param oxygen: Oxygen spectral table.
    :param water_vapor: Water-vapor spectral table.
    :return: Map from ``(altitude, frequency, elevation)`` to loss in decibels.
    """
    # The final edge is exactly the model top, including for nonintegral spans.
    edges = np.append(
        np.arange(ground_altitude, TOP_ALTITUDE, step), TOP_ALTITUDE
    )  # [m]
    thickness = np.diff(edges)  # [m]
    height = edges[:-1] + thickness/2  # [m]
    properties = atmosphere(height)
    index = refractive_index(*properties)
    ground_index = refractive_index(*atmosphere([ground_altitude]))[0]
    ray_ratio = (
        (EARTH_RADIUS + ground_altitude)*ground_index
        / ((EARTH_RADIUS + height)*index)
    )
    path_lengths = {}
    for elevation in ELEVATIONS:  # [deg]
        cosine = np.clip(ray_ratio*np.cos(np.radians(elevation)), -1, 1)
        sine = np.maximum(
            np.sqrt(np.maximum(0, 1 - cosine**2)), MIN_SIN_ELEVATION
        )
        path_lengths[elevation] = thickness*1.0e-3/sine  # [km]
    results = {}
    for frequency, _ in FREQUENCIES:  # [Hz, dB]
        gamma = specific_attenuation(
            frequency, properties, oxygen, water_vapor
        )  # [dB/km]
        for elevation, path_length in path_lengths.items():
            loss = float(np.sum(gamma*path_length))  # [dB]
            results[ground_altitude, frequency, elevation] = loss
    return results


def check_committed_references(results):
    """Check committed literal expectations without importing the test module.

    :param results: Mapping of case coordinates to fine-grid attenuation in dB.
    :raises ValueError: For missing, duplicate, nonfinite or mismatched cases.
    """
    test_path = Path(__file__).resolve().parents[1] / "test_linkBudget.py"
    tree = ast.parse(test_path.read_text(encoding="utf-8"))
    cases = next(
        ast.literal_eval(node.value) for node in tree.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == "ATMOSPHERE_ABSOLUTE_CASES"
            for target in node.targets
        )
    )
    committed = {tuple(case[:3]): case[3] for case in cases}
    if len(committed) != len(cases) or committed.keys() != results.keys():
        raise ValueError("Committed cases differ from the generator matrix.")
    for case, actual in results.items():
        difference = abs(actual - committed[case])  # [dB]
        if (not np.isfinite(actual) or not np.isfinite(committed[case])
                or difference > REFERENCE_TOLERANCE):
            raise ValueError(
                f"Reference mismatch for {case}: generated {actual}, "
                f"committed {committed[case]}"
            )
    print(
        f"# All {len(results)} committed references agree within "
        f"{REFERENCE_TOLERANCE:g} dB."
    )


def main():
    """Print references and fail if convergence or checking fails."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check", action="store_true", help="Check committed references."
    )
    args = parser.parse_args()
    oxygen = load_lines(DataFile.AtmRadioFreqDampData.oxygen)
    water_vapor = load_lines(DataFile.AtmRadioFreqDampData.waterVapor)
    print(f"# NumPy {np.__version__}; layer sine clamp {MIN_SIN_ELEVATION}")
    coarse, fine = {}, {}
    coarse_step = 1.0  # [m]
    fine_step = 0.5  # [m]
    for altitude in GROUND_ALTITUDES:  # [m]
        coarse.update(integrate_grid(
            altitude, coarse_step, oxygen, water_vapor
        ))
        fine.update(integrate_grid(altitude, fine_step, oxygen, water_vapor))
    print(
        "# altitude [m], frequency [Hz], elevation [deg], "
        "1 m loss [dB], 0.5 m loss [dB], difference [dB]"
    )
    max_difference = 0.0  # [dB]
    for case, loss in fine.items():
        difference = abs(loss - coarse[case])  # [dB]
        print(f"# {case}: {coarse[case]:.15g}, {loss:.15g}, {difference:.6g}")
        if (not np.isfinite(loss) or not np.isfinite(coarse[case])
                or difference >= CONVERGENCE_TOLERANCE):
            raise ValueError(
                f"Fine-grid convergence failed for {case}: {difference:g} dB"
            )
        max_difference = max(max_difference, difference)
    print(f"# Maximum 1 m versus 0.5 m difference: {max_difference:.6g} dB")
    print("ATMOSPHERE_ABSOLUTE_CASES = [")
    for frequency, tolerance in FREQUENCIES:  # [Hz, dB]
        for altitude in GROUND_ALTITUDES:  # [m]
            for elevation in ELEVATIONS:  # [deg]
                loss = fine[altitude, frequency, elevation]  # [dB]
                print(
                    f"    ({altitude:.1f}, {frequency/1.0e9:.9g}e9, "
                    f"{elevation:.1f}, {loss:.15g}, {tolerance:g}),"
                )
    print("]")
    if args.check:
        check_committed_references(fine)


if __name__ == "__main__":
    main()
