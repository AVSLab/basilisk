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

import numpy as np

from Basilisk.simulation import motorVoltageInterface
from Basilisk.utilities.MonteCarlo.Controller import SimulationExecutor, SimulationParameters
from Basilisk.utilities.MonteCarlo.Dispersions import UniformDispersion, UniformVectorAngleDispersion


class DummyHub:
    def __init__(self):
        self.mHub = 1.0  # [kg]
        self.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
        self.unitVector = np.array([[1.0], [0.0], [0.0]])  # [-]


class DummyModel:
    def __init__(self):
        self.RNGSeed = 0
        self.hub = DummyHub()


class DummyModelWithoutSeed:
    pass


class DummyTask:
    def __init__(self):
        self.TaskModels = [DummyModel(), DummyModelWithoutSeed()]


class DummyVectorContainer:
    def __init__(self):
        self.values = [0.0, 0.0, 0.0]


class DummyDynModel:
    def __init__(self):
        self.scObject = DummyModel()


class DummySimulation:
    def __init__(self):
        self.TaskList = [DummyTask()]
        self.vectorContainer = DummyVectorContainer()
        self.dynModel = DummyDynModel()

    def get_DynModel(self):
        return self.dynModel


def test_apply_modification_updates_nested_attributes():
    """Verify Monte Carlo path strings update the nested object."""
    sim = DummySimulation()

    mass_path = "TaskList[0].TaskModels[0].hub.mHub"
    SimulationExecutor.applyModification(sim, mass_path, "25.0")

    assert sim.TaskList[0].TaskModels[0].hub.mHub == 25.0
    assert not hasattr(sim, mass_path)

    center_of_mass_path = "TaskList[0].TaskModels[0].hub.r_BcB_B"
    SimulationExecutor.applyModification(sim, center_of_mass_path, "[1.0, 2.0, 3.0]")

    assert sim.TaskList[0].TaskModels[0].hub.r_BcB_B == [1.0, 2.0, 3.0]

    indexed_path = "vectorContainer.values[1]"
    SimulationExecutor.applyModification(sim, indexed_path, "7.5")

    assert sim.vectorContainer.values == [0.0, 7.5, 0.0]

    method_path = "get_DynModel().scObject.hub.mHub"
    SimulationExecutor.applyModification(sim, method_path, "35.0")

    assert sim.get_DynModel().scObject.hub.mHub == 35.0
    assert not hasattr(sim, method_path)


def test_indexed_modification_updates_swig_copy_on_read_container():
    """Verify indexed paths write back copy-on-read SWIG containers."""
    sim = DummySimulation()
    sim.rwVoltageIO = motorVoltageInterface.MotorVoltageInterface()
    initial_gains = [0.02, 0.02, 0.02]  # [N*m/V]
    new_gain = 0.123  # [N*m/V]

    sim.rwVoltageIO.setGains(initial_gains)
    SimulationExecutor.applyModification(
        sim,
        "rwVoltageIO.voltage2TorqueGain[0]",
        str(new_gain)
    )

    assert sim.rwVoltageIO.voltage2TorqueGain[0][0] == new_gain


def test_vector_angle_dispersion_reads_nested_method_path():
    """Verify vector dispersions read nested zero-argument method paths."""
    sim = DummySimulation()
    vector_path = "get_DynModel().scObject.hub.unitVector"
    phi_bounds = [-0.001, 0.001]  # [rad]
    theta_bounds = [-0.001, 0.001]  # [rad]

    dispersion = UniformVectorAngleDispersion(
        vector_path,
        phiBoundsOffNom=phi_bounds,
        thetaBoundsOffNom=theta_bounds
    )

    dispersed_vector = dispersion.generate(sim)

    assert np.isclose(np.linalg.norm(dispersed_vector), 1.0)


def test_populate_seeds_applies_before_configure_function():
    """Verify archived RNGSeed modifications are set before configuration."""
    configured_seeds = []

    def create_sim():
        return DummySimulation()

    def configure_sim(sim):
        configured_seeds.append(sim.TaskList[0].TaskModels[0].RNGSeed)

    def execute_sim(sim):
        assert sim.TaskList[0].TaskModels[0].RNGSeed == 8675309
        assert sim.TaskList[0].TaskModels[0].hub.mHub == 42.0

    sim_params = SimulationParameters(
        creationFunction=create_sim,
        executionFunction=execute_sim,
        configureFunction=configure_sim,
        retentionPolicies=[],
        dispersions=[],
        shouldDisperseSeeds=False,
        shouldArchiveParameters=False,
        filename="",
        icfilename="",
        index=0,
        modifications={
            "TaskList[0].TaskModels[0].RNGSeed": "8675309",
            "TaskList[0].TaskModels[0].hub.mHub": "42.0",
        }
    )

    result = SimulationExecutor()([sim_params, None])

    assert result == (True, 0)
    assert configured_seeds == [8675309]


def test_uniform_dispersion_randomizes_nested_sim_parameter():
    """Verify generated dispersions update the live simulation parameter."""
    mass_path = "TaskList[0].TaskModels[0].hub.mHub"
    mass_bounds = [24.0, 26.0]  # [kg]
    observed_masses = []
    generated_masses = []

    def create_sim():
        return DummySimulation()

    def execute_sim(sim):
        observed_masses.append(sim.TaskList[0].TaskModels[0].hub.mHub)

    for run_index in range(2):
        sim_params = SimulationParameters(
            creationFunction=create_sim,
            executionFunction=execute_sim,
            configureFunction=None,
            retentionPolicies=[],
            dispersions=[UniformDispersion(mass_path, mass_bounds)],
            shouldDisperseSeeds=False,
            shouldArchiveParameters=False,
            filename="",
            icfilename="",
            index=run_index,
            modifications={}
        )

        result = SimulationExecutor()([sim_params, None])
        generated_mass = SimulationExecutor.parseModificationValue(
            sim_params.modifications[mass_path]
        )

        assert result == (True, run_index)
        generated_masses.append(generated_mass)

    assert observed_masses == generated_masses
    assert observed_masses[0] != observed_masses[1]
    for observed_mass in observed_masses:
        assert mass_bounds[0] <= observed_mass <= mass_bounds[1]
        assert observed_mass != DummyHub().mHub


def test_uniform_dispersion_randomizes_method_path_parameter():
    """Verify generated dispersions update a zero-argument method path."""
    mass_path = "get_DynModel().scObject.hub.mHub"
    mass_bounds = [34.0, 36.0]  # [kg]
    observed_masses = []
    generated_masses = []

    def create_sim():
        return DummySimulation()

    def execute_sim(sim):
        observed_masses.append(sim.get_DynModel().scObject.hub.mHub)

    for run_index in range(2):
        sim_params = SimulationParameters(
            creationFunction=create_sim,
            executionFunction=execute_sim,
            configureFunction=None,
            retentionPolicies=[],
            dispersions=[UniformDispersion(mass_path, mass_bounds)],
            shouldDisperseSeeds=False,
            shouldArchiveParameters=False,
            filename="",
            icfilename="",
            index=run_index,
            modifications={}
        )

        result = SimulationExecutor()([sim_params, None])
        generated_mass = SimulationExecutor.parseModificationValue(
            sim_params.modifications[mass_path]
        )

        assert result == (True, run_index)
        generated_masses.append(generated_mass)

    assert observed_masses == generated_masses
    assert observed_masses[0] != observed_masses[1]
    for observed_mass in observed_masses:
        assert mass_bounds[0] <= observed_mass <= mass_bounds[1]
        assert observed_mass != DummyHub().mHub


def test_disperse_seeds_only_records_seeded_models():
    """Verify random seed dispersions are recorded only for seeded models."""
    sim = DummySimulation()

    random_seeds = SimulationExecutor.disperseSeeds(sim)

    seed_path = "TaskList[0].TaskModels[0].RNGSeed"

    assert set(random_seeds.keys()) == {seed_path}
    assert isinstance(
        SimulationExecutor.parseModificationValue(random_seeds[seed_path]),
        int
    )


if __name__ == "__main__":
    test_apply_modification_updates_nested_attributes()
    test_indexed_modification_updates_swig_copy_on_read_container()
    test_vector_angle_dispersion_reads_nested_method_path()
    test_populate_seeds_applies_before_configure_function()
    test_uniform_dispersion_randomizes_nested_sim_parameter()
    test_uniform_dispersion_randomizes_method_path_parameter()
    test_disperse_seeds_only_records_seeded_models()
