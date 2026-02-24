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

"""
Fault definitions for BskSim fault-injection scenarios.

This module provides ``FaultObject`` subclasses that can be assembled into a
``faultList`` (for example, in ``scenario_FaultList.py``).

Each fault object encapsulates:

1. A trigger time (simulation nanoseconds),
2. The mutation applied to the dynamics/FSW model, and
3. A one-time status message.

Usage pattern in scenarios:

1. Instantiate one or more fault objects.
2. Add them to a list.
3. Call ``fault.addFaultToSimulation(sim, idx)`` for each element.

The base class registers each fault as an event through ``createNewEvent`` and
executes the fault action at its configured time.

Indexing conventions:

- Fault index arguments use 0-based indexing.
- Reaction wheel indices must be in the range 0..3.
- CSS sensor indices must be in the range 0..7.
- Magnetometer axis indices must be in the range 0..2.
"""

import numpy as np
from Basilisk.simulation import encoder, tempMeasurement, magnetometer
from Basilisk.utilities import macros, unitTestSupport, orbitalMotion, vizSupport


def faultTypeInList(faultList, faultType):
    return np.any([type(fault) == faultType for fault in faultList])


def getFaultFromList(faultList, faultType):
    return faultList[np.where([type(fault) == faultType for fault in faultList])]


class FaultObject:
    def __init__(self, name, time, verbose=True, **kwargs):
        self.name = name
        self.time = time
        self.verbose = verbose
        self.message = None
        self.message_printed = False

    def execute(self, simulation):
        if getattr(self, "executed", False):
            return
        self.executed = True
        raise NotImplementedError(
            f"{self.name} does not have a custom execute function!"
        )

    def print_message(self, message):
        if not self.message_printed:
            print(message)
            self.message_printed = True

    def addFaultToSimulation(self, simulation, listIdx):
        self.uniqueFaultIdx = listIdx  # Index in the faultList array.
        eventName = f"add{self.name}Fault_{listIdx}"

        def fault(self, uniqueFaultIdx=self.uniqueFaultIdx, eventName=eventName):
            self.faultList[uniqueFaultIdx].execute(self)
            self.faultList[uniqueFaultIdx].print()
            self.setEventActivity(eventName, False)

        simulation.createNewEvent(
            eventName,
            simulation.get_DynModel().processTasksTimeStep,
            True,
            conditionTime=self.time,
            actionFunction=fault,
        )


class RwPowerFault(FaultObject):
    """
    Represents a reaction wheel power fault event.

    This fault reduces the maximum power or torque output available
    from one or all reaction wheels.

    Parameters
    ----------
    time : float
        Simulation time (in nanoseconds) when the fault occurs.
    reducedLimit : float
        Power limit value (in watts).
    wheelIdx : int or str
        Index (0, 1, 2, or 3) of the affected reaction wheel(s),
        or "all" to apply to every wheel.
    name : str, optional
        Name identifier for the fault (default: "rwPowerLimited").
    """

    def __init__(self, time, reducedLimit, wheelIdx, name="rwPowerLimited"):
        super().__init__(name, time)
        self.reducedLimit = reducedLimit

        if wheelIdx == "all":
            self.wheelIdx = wheelIdx
        elif isinstance(wheelIdx, (float, int)):
            idx = int(wheelIdx)
            if 0 <= idx <= 3:
                self.wheelIdx = idx
            else:
                raise ValueError(
                    "Fault parameter 'wheelIdx' must be 0-3 or 'all'"
                )
        else:
            raise ValueError(
                "Fault parameter 'wheelIdx' must either be a number corresponding to a reaction wheel or the string 'all'"
            )

    def execute(self, simulation):
        if getattr(self, "executed", False):
            return
        self.executed = True
        dynModels = simulation.get_DynModel()
        if self.wheelIdx == "all":
            # option to trigger the fault in all wheels (not supported for all fault types)
            dynModels.RW1.P_max = self.reducedLimit
            dynModels.RW2.P_max = self.reducedLimit
            dynModels.RW3.P_max = self.reducedLimit
            dynModels.RW4.P_max = self.reducedLimit
        else:
            rwList = [dynModels.RW1, dynModels.RW2, dynModels.RW3, dynModels.RW4]
            rwList[self.wheelIdx].P_max = self.reducedLimit

    def print(self):
        if self.wheelIdx == "all":
            self.message = f"RW Power Fault: all RW's power limit reduced to {self.reducedLimit} Watts at {self.time*macros.NANO2MIN} minutes!"
        else:
            self.message = f"RW Power Fault: RW{self.wheelIdx}'s power limit reduced to {self.reducedLimit} Watts at {self.time*macros.NANO2MIN} minutes!"
        super().print_message(self.message)


class RwFrictionFault(FaultObject):
    """
    Represents a reaction wheel friction fault event.

    This fault increases the friction in one of reaction wheels,
    reducing their effective torque output.

    Parameters
    ----------
    time : float
        Simulation time (in nanoseconds) when the fault occurs.
    frictionMultiplier : float
        Multiplier applied to the reaction wheel friction.
        For example, 2.0 doubles the nominal friction.
    wheelIdx : int
        Index (0, 1, 2, or 3) of the affected reaction wheel.
    name : str, optional
        Name identifier for the fault (default: "rwFriction").
    """

    def __init__(self, time, frictionMultiplier, wheelIdx, name="rwFriction"):
        super().__init__(name, time)
        self.frictionMultiplier = frictionMultiplier
        idx = int(wheelIdx)
        if 0 <= idx <= 3:
            self.wheelIdx = idx
        else:
            raise ValueError("Fault parameter 'wheelIdx' must be 0-3")

    def execute(self, simulation):
        if getattr(self, "executed", False):
            return
        self.executed = True
        dynModels = simulation.get_DynModel()
        rwList = [dynModels.RW1, dynModels.RW2, dynModels.RW3, dynModels.RW4]
        rwList[self.wheelIdx].fCoulomb *= self.frictionMultiplier

    def print(self):
        self.message = f"RW Friction Fault: RW{self.wheelIdx}'s friction multiplied by {self.frictionMultiplier}x at {self.time*macros.NANO2MIN} minutes!"
        super().print_message(self.message)


class DebrisImpactOn(FaultObject):
    """
    Represents a debris impact fault on the spacecraft.

    This fault simulates an impact from space debris at a specified location
    on the spacecraft body. The momentum transfer due to the impact generates
    both force and torque on the spacecraft.

    Parameters
    ----------
    time : float
        Simulation time (in nanoseconds) when the impact occurs.
    debris_location : array_like
        Impact location in body coordinates [m] as a 3-element vector.
    direction : array_like
        Impact direction in body coordinates as a 3-element vector.
        The vector is normalized internally.
    mass : float
        Mass of the debris particle in kilograms (kg).
    velocityMag : float, optional
        Magnitude of the debris velocity in meters per second (m/s)
        (default: 8e3 m/s).
    name : str, optional
        Name identifier for the fault (default: "DebrisImpactOn").
    """

    def __init__(
        self,
        time,
        debris_location,
        direction,
        mass,
        velocityMag=8e3,
        name="DebrisImpactOn",
    ):
        super().__init__(name, time)

        self.location = np.asarray(debris_location, dtype=float).reshape(-1)
        if self.location.size != 3:
            raise ValueError(
                "Fault parameter 'debris_location' must be a 3-element vector"
            )

        direction = np.asarray(direction, dtype=float).reshape(-1)
        if direction.size != 3:
            raise ValueError(
                "Fault parameter 'direction' must be a 3-element vector"
            )
        direction_norm = np.linalg.norm(direction)
        if direction_norm == 0.0:
            raise ValueError("Fault parameter 'direction' must be non-zero")
        self.direction = direction / direction_norm

        self.velocityMag = velocityMag
        self.velocity = self.direction * self.velocityMag
        self.mass = mass  # kg
        self.label = "custom"

    def execute(self, simulation):
        dynModels = simulation.get_DynModel()
        dt = simulation.dynRate
        momentum = self.mass * self.velocity
        force_B = momentum / dt

        torque_B = np.cross(self.location, force_B)

        dynModels.extForceTorqueObject.extForce_B = force_B
        dynModels.extForceTorqueObject.extTorquePntB_B = torque_B

    def print(self):
        self.message = f"Debris impact case {self.label} with {self.mass} kg and {self.velocity} m/s of debris hit at {self.location} m at {self.time * macros.NANO2MIN} minutes!"
        super().print_message(self.message)


class DebrisImpactOff(FaultObject):
    """
    Represents the end of a debris impact fault.

    This fault resets the external force and torque applied by a previous
    debris impact (e.g., DebrisImpactOn) back to zero, effectively ending
    the impact event.

    Parameters
    ----------
    time : float
        Simulation time (in nanoseconds) when the impact effect ends.
    name : str, optional
        Name identifier for the fault (default: "DebrisImpactOff").
    """

    def __init__(self, time, name="DebrisImpactOff"):
        super().__init__(name, time)

    def execute(self, simulation):
        dynModels = simulation.get_DynModel()

        force_B = np.array([0, 0, 0])
        torque_B = np.array([0, 0, 0])

        dynModels.extForceTorqueObject.extForce_B = force_B
        dynModels.extForceTorqueObject.extTorquePntB_B = torque_B

    def print(self):
        self.message = "Debris impact done!"
        super().print_message(self.message)


class CssSignalFault(FaultObject):
    """
    Represents a coarse sun sensor (CSS) measurement fault.

    The types of CSS faults are as follows:
    1. The specified CSS measurement is turned off ("CSSFAULT_OFF").
    2. The CSS measurement is stuck at the current value ("CSSFAULT_STUCK_CURRENT").
    3. The CSS measurement is stuck at the maximum value ("CSSFAULT_STUCK_MAX").
    4. The CSS measurement is stuck at a random value ("CSSFAULT_STUCK_RAND").
    5. The CSS measurement is randomly altered ("CSSFAULT_RAND").

    Parameters
    ----------
    time : float
        Simulation time (in nanoseconds) when the fault occurs.
    type : str
        Type of CSS fault, one of:
        "CSSFAULT_OFF", "CSSFAULT_STUCK_CURRENT", "CSSFAULT_STUCK_MAX",
        "CSSFAULT_STUCK_RAND", or "CSSFAULT_RAND".
    cssIdx : list[int]
        Index (0, 1, 2, 3, 4, 5, 6, 7) of the affected CSS sensor.
    name : str, optional
        Name identifier for the fault (default: "cssSignalOff").
    """

    def __init__(self, time, type, cssIdx, name="cssSignalOff"):
        super().__init__(name, time)
        valid_types = {
            "CSSFAULT_OFF",
            "CSSFAULT_STUCK_CURRENT",
            "CSSFAULT_STUCK_MAX",
            "CSSFAULT_STUCK_RAND",
            "CSSFAULT_RAND",
        }
        if type not in valid_types:
            raise ValueError(
                "Fault parameter 'type' must be one of "
                f"{sorted(valid_types)}"
            )
        self.type = type
        self.cssIdx = np.array(cssIdx, dtype=int).reshape((-1))
        if np.any(self.cssIdx < 0) or np.any(self.cssIdx > 7):
            raise ValueError("Fault parameter 'cssIdx' must be in the range 0-7")

    def execute(self, simulation):
        from Basilisk.simulation import coarseSunSensor

        if getattr(self, "executed", False):
            return
        self.executed = True
        dynModels = simulation.get_DynModel()

        fault_state_map = {
            "CSSFAULT_OFF": coarseSunSensor.CSSFAULT_OFF,
            "CSSFAULT_STUCK_CURRENT": coarseSunSensor.CSSFAULT_STUCK_CURRENT,
            "CSSFAULT_STUCK_MAX": coarseSunSensor.CSSFAULT_STUCK_MAX,
            "CSSFAULT_STUCK_RAND": coarseSunSensor.CSSFAULT_STUCK_RAND,
            "CSSFAULT_RAND": coarseSunSensor.CSSFAULT_RAND,
        }
        faultState = fault_state_map[self.type]
        for idx in self.cssIdx:
            # idx needs to get wrapped in int() because it's not just an int when returned from self.cssIdx?
            print(f"Setting CSS {idx} fault state to {faultState}")
            dynModels.CSSConstellationObject.sensorList[int(idx)].faultState = faultState

    def print(self):
        self.message = f"CSS Signal Fault {self.type} executed on CSS {self.cssIdx} at {self.time*macros.NANO2MIN} minutes!"
        super().print_message(self.message)


class RWBitFlipFault(FaultObject):
    """
    Represents a reaction wheel control bit-flip fault.

    This fault simulates a scenario in which a single bit in the reaction wheel
    control axis configuration is flipped.

    Parameters
    ----------
    time : float
        Simulation time (in nanoseconds) when the fault occurs.
    bitFlipIdx : int
        Zero-based index of the bit to flip (0–15).
    RW_Idx : int
        Index of the affected reaction wheel (0–3).
    elementIdx : int
        Zero-based index of the axis to flip within the reaction wheel control (0–2).
    name : str, optional
        Name identifier for the fault (default: "RWBitFlipFault").
    """

    def __init__(self, time, bitFlipIdx, RW_Idx, elementIdx, name="RWBitFlipFault"):
        super().__init__(name, time)

        self.bitFlipIdx = int(bitFlipIdx)
        if self.bitFlipIdx < 0 or self.bitFlipIdx > 15:
            raise ValueError("Fault parameter 'bitFlipIdx' must be 0-15")
        idx = int(RW_Idx)
        if 0 <= idx <= 3:
            self.RW_Idx = idx
        else:
            raise ValueError("Fault parameter 'RW_Idx' must be 0-3")
        self.elementIdx = int(elementIdx)
        if self.elementIdx < 0 or self.elementIdx > 2:
            raise ValueError("Fault parameter 'elementIdx' must be 0-2")

    def float16_to_bits(self, f):
        # Convert float16 to raw bits
        return np.frombuffer(np.float16(f).tobytes(), dtype=np.uint16)[0]

    def bits_to_float16(self, bits):
        # Convert raw bits back to float16
        return np.frombuffer(np.uint16(bits).tobytes(), dtype=np.float16)[0]

    def flip_one_bit(self, value, bit_idx):
        return value ^ (1 << bit_idx)

    def print_binary(self, value):
        # Print the binary representation with leading zeros for 16 bits
        return format(value, "016b")

    def execute(self, simulation):
        if getattr(self, "executed", False):
            return
        self.executed = True
        dynModels = simulation.get_DynModel()
        fswModels = simulation.get_FswModel()

        rw_Idx = f"RW{self.RW_Idx + 1}"
        self.gsHat_B = getattr(dynModels, rw_Idx).gsHat_B
        self.original = self.gsHat_B[self.elementIdx][0]
        bits = self.float16_to_bits(self.original)

        # Flip just one bit
        flipped_bits = self.flip_one_bit(bits, self.bitFlipIdx)
        self.gsHat_B[self.elementIdx][0] = self.bits_to_float16(flipped_bits)
        GsHat_B = fswModels.rwMotorTorque.GsMatrix_B
        GsHat_B[self.RW_Idx * 3 + self.elementIdx] = self.gsHat_B[
            self.elementIdx
        ][0]
        fswModels.rwMotorTorque.GsMatrix_B = GsHat_B

    def print(self):
        self.message = f"RW Bit Flip Fault occurred for RW{self.RW_Idx} of element {self.elementIdx} on bit {self.bitFlipIdx} at {self.time*macros.NANO2MIN} minutes! The value changed from {self.original} (binary: {self.print_binary(self.float16_to_bits(self.original))}) to {self.gsHat_B[self.elementIdx][0]} (binary: {self.print_binary(self.float16_to_bits(self.gsHat_B[self.elementIdx][0]))})"
        super().print_message(self.message)


class RWGainBitFlipFault(FaultObject):
    """
    Represents a reaction wheel feedback gain bit-flip fault.

    This fault simulates a scenario in which a single bit in the reaction wheel
    feedback gain configuration is flipped.

    Parameters
    ----------
    time : float
        Simulation time (in nanoseconds) when the fault occurs.
    bitFlipIdx : int
        Zero-based index of the bit to flip (0–15).
    gainType : str
        Type of gain affected, one of:
        - "K"  : Proportional gain applied to MRP errors (rad/sec)
        - "P"  : Rate error feedback gain (N·m·s)
        - "Ki" : Integral feedback on rate error (N·m)
    name : str, optional
        Name identifier for the fault (default: "RWGainBitFlipFault").
    """

    def __init__(self, time, bitFlipIdx, gainType, name="RWGainBitFlipFault"):
        super().__init__(name, time)

        self.bitFlipIdx = int(bitFlipIdx)
        if self.bitFlipIdx < 0 or self.bitFlipIdx > 15:
            raise ValueError("Fault parameter 'bitFlipIdx' must be 0-15")
        self.gainType = gainType

    def float16_to_bits(self, f):
        # Convert float16 to raw bits
        return np.frombuffer(np.float16(f).tobytes(), dtype=np.uint16)[0]

    def bits_to_float16(self, bits):
        # Convert raw bits back to float16
        return np.frombuffer(np.uint16(bits).tobytes(), dtype=np.float16)[0]

    def flip_one_bit(self, value, bit_idx):
        return value ^ (1 << bit_idx)

    def print_binary(self, value):
        # Print the binary representation with leading zeros for 16 bits
        return format(value, "016b")

    def execute(self, simulation):
        if getattr(self, "executed", False):
            return
        self.executed = True
        fswModels = simulation.get_FswModel()
        self.gain = getattr(fswModels.mrpFeedbackRWs, self.gainType)
        bits = self.float16_to_bits(self.gain)

        # Flip just one bit
        flipped_bits = self.flip_one_bit(bits, self.bitFlipIdx)
        self.flipped_gain = self.bits_to_float16(flipped_bits)
        setattr(fswModels.mrpFeedbackRWs, self.gainType, float(self.flipped_gain))

    def print(self):
        self.message = f"RW Gain {self.gainType} Bit Flip Fault occurred on bit {self.bitFlipIdx} at {self.time*macros.NANO2MIN} minutes! The value changed from {self.gain} (binary: {self.print_binary(self.float16_to_bits(self.gain))}) to {self.flipped_gain} (binary: {self.print_binary(self.float16_to_bits(self.flipped_gain))})"
        super().print_message(self.message)


class MagnetometerFault(FaultObject):
    """
    Represents a magnetometer sensor fault.

    This fault simulates scenarios in which magnetometer measurements become inaccurate
    due to stuck readings or random spikes, depending on the selected fault type.
    It can affect one or more of the sensor’s three axes (x, y, z).

    Parameters
    ----------
    time : float
        Simulation time (in nanoseconds) when the fault occurs.
    faultType : str
        Type of magnetometer fault. Supported types include:
        - "MAG_FAULT_STUCK_CURRENT" : Output held at the current value.
        - "MAG_FAULT_STUCK_VALUE"   : Output fixed at a given constant value.
        - "MAG_FAULT_SPIKING"       : Output intermittently spiking.
    faultyAxis : int
        Axis affected by the fault (0–2), corresponding to x, y, z.
    stuckValue : float, optional
        Constant value used when the fault type is "MAG_FAULT_STUCK_VALUE" (default: 0.0).
    spikeProbability : float, optional
        Probability of a spike occurring at each time step when the fault type is
        "MAG_FAULT_SPIKING" (default: 0.01).
    spikeAmount : float, optional
        Magnitude of the spike added to the nominal measurement when a spike occurs
        (default: 2.0).
    name : str, optional
        Name identifier for the fault (default: "magnetometerFault").
    """

    def __init__(
        self,
        time,
        faultType,
        faultyAxis,
        stuckValue=0.0,
        spikeProbability=0.01,
        spikeAmount=2.0,
        name="magnetometerFault",
    ):
        super().__init__(name, time)

        valid_fault_types = {
            "MAG_FAULT_STUCK_CURRENT",
            "MAG_FAULT_STUCK_VALUE",
            "MAG_FAULT_SPIKING",
        }
        if faultType not in valid_fault_types:
            raise ValueError(
                "Fault parameter 'faultType' must be one of "
                f"{sorted(valid_fault_types)}"
            )
        self.faultType = faultType
        self.faultyAxis = int(faultyAxis)
        if self.faultyAxis < 0 or self.faultyAxis > 2:
            raise ValueError("Fault parameter 'faultyAxis' must be 0-2")

        # Pull optional args from kwargs with defaults
        self.stuckValue = stuckValue
        self.spikeProbability = spikeProbability
        self.spikeAmount = spikeAmount

    def execute(self, simulation):
        if getattr(self, "executed", False):
            return
        self.executed = True
        dynModels = simulation.get_DynModel()

        if self.faultType == "MAG_FAULT_STUCK_CURRENT":
            dynModels.TAM.setFaultState(
                self.faultyAxis, magnetometer.MAG_FAULT_STUCK_CURRENT
            )
        elif self.faultType == "MAG_FAULT_STUCK_VALUE":
            dynModels.TAM.setFaultState(
                self.faultyAxis, magnetometer.MAG_FAULT_STUCK_VALUE
            )
            stuckValue = [self.stuckValue] * 3
            dynModels.TAM.stuckValue = stuckValue
        elif self.faultType == "MAG_FAULT_SPIKING":
            dynModels.TAM.setFaultState(self.faultyAxis, magnetometer.MAG_FAULT_SPIKING)
            spikeProbability = [self.spikeProbability] * 3
            dynModels.TAM.spikeProbability = spikeProbability
            spikeAmount = [self.spikeAmount] * 3
            dynModels.TAM.spikeAmount = spikeAmount

    def print(self):
        if self.faultType == "MAG_FAULT_STUCK_CURRENT":
            self.message = f"Magnetometer Fault: Magnetometer{self.faultyAxis} is stuck at current value at {self.time * macros.NANO2MIN} minutes!"
        elif self.faultType == "MAG_FAULT_STUCK_VALUE":
            self.message = f"Magnetometer Fault: Magnetometer{self.faultyAxis} is stuck at value {self.stuckValue} at {self.time * macros.NANO2MIN} minutes!"
        elif self.faultType == "MAG_FAULT_SPIKING":
            self.message = f"Magnetometer Fault: Magnetometer{self.faultyAxis} is spiking with probability {self.spikeProbability} and spike amount {self.spikeAmount} at {self.time * macros.NANO2MIN} minutes!"
        else:
            self.message = f"Magnetometer Nominal: Magnetometer{self.faultyAxis} is nominal at {self.time * macros.NANO2MIN} minutes!"
        super().print_message(self.message)

class MagPolarNoise(FaultObject):
    """
    Magnetometer fault model that emulates degradation effects in Earth's polar regions.

    This fault object modifies the behavior of a three-axis magnetometer (TAM)
    based on the spacecraft's geomagnetic latitude. As the spacecraft enters
    high-latitude regions (:math:`|\\mathrm{latitude}| \\gtrsim 60\\,\\mathrm{deg}`),
    the fault severity transitions
    smoothly from nominal to polar conditions using a sigmoid weighting function.

    Depending on the selected fault type, the model can:

    - Increase sensor noise and random-walk bounds ("NOISE")
    - Inject sporadic measurement spikes ("SPIKE")
    - Apply both noise degradation and spiking behavior ("BOTH")

    The spacecraft position is obtained in the inertial frame, transformed into
    the Earth-fixed frame using SPICE-provided planet state information, and
    converted to geodetic latitude. Fault parameters are then interpolated
    between nominal and polar values based on latitude.

    Parameters
    ----------
    time : float
        Simulation time at which the fault is executed.
    faultType : str
        Type of magnetometer fault to apply. Must be one of:
        {"NOISE", "SPIKE", "BOTH"}.
    name : str, optional
        Name of the fault object (default is "magPolarFault").

    Notes
    -----

    - The latitude transition is centered at 60 degrees with a 5-degree
      smoothing width.
    - Noise and random-walk bounds are applied directly to the TAM model.
    - Spiking faults are enabled via the magnetometer fault-state interface.
    - This model assumes an Earth-centered mission and valid SPICE ephemerides.
    """

    def __init__(
        self,
        time,
        faultType,
        name="magPolarFault",
    ):
        super().__init__(name, time)

        self.faultType = faultType  # "NOISE", "SPIKE", "BOTH"

    def execute(self, simulation):
        if getattr(self, "executed", False):
            return
        self.executed = True
        dynModels = simulation.get_DynModel()
        r_BN_N = dynModels.scObject.scStateOutMsg.read().r_BN_N
        J20002Pfix = dynModels.gravFactory.spiceObject.planetStateOutMsgs[dynModels.earth].read().J20002Pfix
        r_PN_N = dynModels.gravFactory.spiceObject.planetStateOutMsgs[dynModels.earth].read().PositionVector

        r_BN_N = np.array(r_BN_N)
        r_PN_N = np.array(r_PN_N)
        C_PN = np.array(J20002Pfix)

        r_BP_N = r_BN_N - r_PN_N
        r_BP_P = C_PN @ r_BP_N
        lat_rad, _, _ = vizSupport.fixedframe2lla(
            r_BP_P,
            orbitalMotion.REQ_EARTH * 1000.0,
            orbitalMotion.RP_EARTH / orbitalMotion.REQ_EARTH,
        )
        lat_deg = np.degrees(float(lat_rad))
        S = 1 / (1 + np.exp(-(abs(lat_deg) - 60) / 5))  # 60 deg threshold, 5 deg transition

        nominalsenNoiseStd = np.array([4e-9, 4e-9, 4e-9])
        nominalwalkBounds  = np.array([15e-9, 15e-9, 15e-9])
        polarsenNoiseStd   = np.array([120e-9, 120e-9, 120e-9])
        polarwalkBounds    = np.array([600e-9, 600e-9, 600e-9])
        senNoise = (1-S)*nominalsenNoiseStd + S*polarsenNoiseStd
        walkBounds = (1-S)*nominalwalkBounds + S*polarwalkBounds
        # GaussMarkov checks new bounds against the previous noise matrix.
        # Keep bounds above 3-sigma for both current and previous noise levels
        # to avoid transient truncation warnings during latitude transitions.
        prevNoise = np.asarray(dynModels.TAM.senNoiseStd, dtype=float).reshape(-1)
        if prevNoise.size != 3:
            prevNoise = senNoise
        safetySigma = 3.1
        walkBounds = np.maximum(walkBounds, safetySigma * np.maximum(senNoise, prevNoise))

        nominalSpikeRate = 0.005
        polarSpikeRate  = 0.1
        spikeAmount = 2.0

        if self.faultType == "NOISE":
            dynModels.TAM.senNoiseStd = senNoise.tolist()
            dynModels.TAM.walkBounds  = walkBounds.tolist()
        elif self.faultType == "SPIKE":
            spikeRate = (1-S)*nominalSpikeRate + S*polarSpikeRate
            dynModels.TAM.setFaultState(0, magnetometer.MAG_FAULT_SPIKING)
            dynModels.TAM.setFaultState(1, magnetometer.MAG_FAULT_SPIKING)
            dynModels.TAM.setFaultState(2, magnetometer.MAG_FAULT_SPIKING)

            dynModels.TAM.spikeProbability = [spikeRate]*3
            dynModels.TAM.spikeAmount = [spikeAmount]*3
        elif self.faultType == "BOTH":
            dynModels.TAM.senNoiseStd = senNoise.tolist()
            dynModels.TAM.walkBounds  = walkBounds.tolist()

            spikeRate = (1-S)*nominalSpikeRate + S*polarSpikeRate
            dynModels.TAM.setFaultState(0, magnetometer.MAG_FAULT_SPIKING)
            dynModels.TAM.setFaultState(1, magnetometer.MAG_FAULT_SPIKING)
            dynModels.TAM.setFaultState(2, magnetometer.MAG_FAULT_SPIKING)

            dynModels.TAM.spikeProbability = [spikeRate]*3
            dynModels.TAM.spikeAmount = [spikeAmount]*3
        else:
            raise ValueError("Invalid fault type. Must be 'NOISE', 'SPIKE', or 'BOTH'.")


    def print(self):
        pass
        # self.message = ""
        # super().print_message(self.message)
