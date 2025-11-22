from typing import Literal

import pytest
import numpy as np
import numpy.testing as npt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import orbitalMotion
from Basilisk.simulation import pointMassGravityModel
from Basilisk.simulation import exponentialAtmosphere

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJAerodynamicDrag
    from Basilisk.simulation import NBodyGravity

    couldImportMujoco = True
except:
    couldImportMujoco = False

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_aerodynamicDrag():
    """
    Unit test for classes that inherit from MeanRevertingNoise.
    """
    dt = 1 # s

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create the MJScene from a simple cannonball body
    scene = mujoco.MJScene("<mujoco/>") # empty scene, no multi-body dynamics
    scSim.AddModelToTask("test", scene)

    density = 2 # kg/m^3
    cd = 1.5
    area = 2.75 # m^2
    r_CP_S = [1, 0, 0] # m

    v_SN_N = [0, 0.5, 0] # m/s
    sigma_SN = [0.1, 0.2, 0.3]

    atmoMsg = messaging.AtmoPropsMsg()
    atmoMsg.write(messaging.AtmoPropsMsgPayload(neutralDensity=density))

    dragGeometryMsg = messaging.DragGeometryMsg()
    dragGeometryMsg.write(messaging.DragGeometryMsgPayload(
        projectedArea = area,
        dragCoeff = cd,
        r_CP_S = r_CP_S
    ))

    siteFrameMsg = messaging.SCStatesMsg()
    siteFrameMsg.write(messaging.SCStatesMsgPayload(
        sigma_BN = sigma_SN,
        v_BN_N = v_SN_N
    ))

    drag = MJAerodynamicDrag.AerodynamicDrag()
    drag.dragGeometryInMsg.subscribeTo(dragGeometryMsg)
    drag.atmoDensInMsg.subscribeTo(atmoMsg)
    drag.referenceFrameStateInMsg.subscribeTo(siteFrameMsg)
    scene.AddModelToDynamicsTask(drag)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(dt))
    scSim.ExecuteSimulation()

    force_S = drag.forceOutMsg.read().force_S
    torque_S = drag.torqueOutMsg.read().torque_S

    # Expected
    # Rotate velocity from N to S using MRPs
    C_BN = np.array(rbk.MRP2C(sigma_SN))  # maps N components to B=S components
    v_S = C_BN.dot(v_SN_N)
    v_mag = np.linalg.norm(v_SN_N)  # drag magnitude depends on speed, frame invariant
    if v_mag <= 1e-12:
        F_exp = np.zeros(3)
        T_exp = np.zeros(3)
    else:
        F_mag = 0.5 * density * (v_mag ** 2) * cd * area
        v_hat_S = v_S / np.linalg.norm(v_S)
        F_exp = -F_mag * v_hat_S
        T_exp = np.cross(r_CP_S, F_exp)

    npt.assert_allclose(force_S, F_exp, rtol=0.0, atol=1e-12)
    npt.assert_allclose(torque_S, T_exp, rtol=0.0, atol=1e-12)

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_orbit(orbitCase: Literal["LPO", "LTO"], planetCase: Literal["earth", "mars"], showPlots = False):
    def cannonballDragBFrame(dragCoeff, density, area, velInertial, sigmaBN):
        speed = np.linalg.norm(velInertial)
        if speed <= 0.0:
            return np.zeros(3)

        dragDirInertial = -velInertial / speed
        dcmBN = rbk.MRP2C(sigmaBN)
        dragDirBody = dcmBN.dot(dragDirInertial)

        return 0.5 * dragCoeff * density * area * speed**2 * dragDirBody


    scSim = SimulationBaseClass.SimBaseClass()
    processName = "dragProcess"
    taskName = "dragTask"
    dynamicsProcess = scSim.CreateNewProcess(processName)
    stepNanos = macros.sec2nano(1.0)
    dynamicsProcess.addTask(scSim.CreateNewTask(taskName, stepNanos))

    bodyName = "satellite"
    dragSceneXml = fr"""
        <mujoco>
            <worldbody>
                <body name="{bodyName}">
                    <joint name="chief_x" axis="1 0 0" type="slide"/>
                    <joint name="chief_y" axis="0 1 0" type="slide"/>
                    <joint name="chief_z" axis="0 0 1" type="slide"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                </body>
            </worldbody>
        </mujoco>
    """
    scene = mujoco.MJScene(dragSceneXml)
    scene.extraEoMCall = True
    scSim.AddModelToTask(taskName, scene)

    mjBody = scene.getBody(bodyName)

    planetData = simIncludeGravBody.BODY_DATA[planetCase]
    gravitationalParameter = planetData.mu
    planetRadius = planetData.radEquator

    gravityModel = NBodyGravity.NBodyGravity()
    gravityModel.ModelTag = "gravity"
    scene.AddModelToDynamicsTask(gravityModel)

    pointMassModel = pointMassGravityModel.PointMassGravityModel()
    pointMassModel.muBody = gravitationalParameter
    gravityModel.addGravitySource(planetCase, pointMassModel, True)
    gravityModel.addGravityTarget(bodyName, mjBody)

    if planetCase == "earth":
        baseDensity = 1.217
        scaleHeight = 8500.0
    else:
        baseDensity = 0.020
        scaleHeight = 11100.0

    atmoModel = exponentialAtmosphere.ExponentialAtmosphere()
    atmoModel.ModelTag = "ExpAtmo"
    atmoModel.planetRadius = planetRadius
    atmoModel.baseDensity = baseDensity
    atmoModel.scaleHeight = scaleHeight
    scene.AddModelToDynamicsTask(atmoModel)

    comStateMsg = mjBody.getCenterOfMass().stateOutMsg
    atmoModel.addSpacecraftToModel(comStateMsg)

    projectedArea = 10.0
    dragCoeff = 2.2

    dragGeometryMsg = messaging.DragGeometryMsg()
    dragGeometryMsg.write(
        messaging.DragGeometryMsgPayload(
            projectedArea=projectedArea,
            dragCoeff=dragCoeff,
            r_CP_S=[1.0, 0.0, 0.0],
        )
    )

    dragModel = MJAerodynamicDrag.AerodynamicDrag()
    dragModel.ModelTag = "drag"
    scene.AddModelToDynamicsTask(dragModel)
    forceTorqueDrag: mujoco.MJForceTorqueActuator = dragModel.applyTo(mjBody)
    dragModel.atmoDensInMsg.subscribeTo(atmoModel.envOutMsgs[0])
    dragModel.dragGeometryInMsg.subscribeTo(dragGeometryMsg)

    if orbitCase == "LPO":
        orbAltMin = 300e3 # m
        orbAltMax = orbAltMin
    elif orbitCase == "LTO":
        orbAltMin = 300e3 # m
        orbAltMax = 800e3 # m

    rMin = planetRadius + orbAltMin
    rMax = planetRadius + orbAltMax

    elements = orbitalMotion.ClassicElements()
    elements.a = (rMin+rMax)/2.0
    elements.e = 1.0 - rMin/elements.a
    elements.i = 0.0
    elements.Omega = 0.0
    elements.omega = 0.0
    elements.f = 0.0

    meanMotion = np.sqrt(gravitationalParameter / elements.a**3)
    orbitPeriod = 2.0 * np.pi / meanMotion
    finalTimeNanos = macros.sec2nano(orbitPeriod)

    stateRecorder = comStateMsg.recorder()
    atmoRecorder = atmoModel.envOutMsgs[0].recorder()
    dragForceRecorder = forceTorqueDrag.forceInMsg.recorder()

    scSim.AddModelToTask(taskName, stateRecorder)
    scSim.AddModelToTask(taskName, atmoRecorder)
    scSim.AddModelToTask(taskName, dragForceRecorder)

    scSim.InitializeSimulation()

    positionInertial, velocityInertial = orbitalMotion.elem2rv(gravitationalParameter, elements)
    mjBody.setPosition(positionInertial)
    mjBody.setVelocity(velocityInertial)

    scSim.ConfigureStopTime(finalTimeNanos)
    scSim.ExecuteSimulation()

    positionData = stateRecorder.r_BN_N
    velocityData = stateRecorder.v_BN_N
    attitudeData = stateRecorder.sigma_BN
    densityData = atmoRecorder.neutralDensity
    dragForceData = dragForceRecorder.force_S

    numSteps = dragForceData.shape[0]
    referenceDrag = np.zeros_like(dragForceData)

    for timeIndex in range(numSteps):
        referenceDrag[timeIndex, :] = cannonballDragBFrame(
            dragCoeff=dragCoeff,
            density=densityData[timeIndex],
            area=projectedArea,
            velInertial=velocityData[timeIndex, :],
            sigmaBN=attitudeData[timeIndex, :],
        )

    npt.assert_allclose(
        dragForceData,
        referenceDrag,
        rtol=0,
        atol=1e-12,
    )

    if showPlots:
        import matplotlib.pyplot as plt

        timesSec = stateRecorder.times() * macros.NANO2SEC
        timesOrbit = timesSec / orbitPeriod

        # Inertial position components
        plt.figure()
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style="plain")
        for idx in range(3):
            plt.plot(
                timesOrbit,
                positionData[:, idx] / 1e3,
                label=f"$r_{{BN,{idx}}}$",
            )
        plt.legend(loc="lower right")
        plt.xlabel("Time [orbits]")
        plt.ylabel("Inertial position [km]")

        # Orbit in perifocal frame
        elementsInit = orbitalMotion.rv2elem(gravitationalParameter, positionInertial, velocityInertial)
        bParam = elementsInit.a * np.sqrt(1.0 - elementsInit.e**2)
        pParam = elementsInit.a * (1.0 - elementsInit.e**2)

        plt.figure(figsize=tuple(np.array((1.0, bParam / elementsInit.a)) * 4.75), dpi=100)
        axisLimits = np.array(
            [-elementsInit.rApoap, elementsInit.rPeriap, -bParam, bParam]
        ) / 1e3 * 1.25
        plt.axis(axisLimits)

        fig = plt.gcf()
        ax = fig.gca()
        planetColor = "#008800"
        planetRadiusKm = planetRadius / 1e3
        ax.add_artist(plt.Circle((0.0, 0.0), planetRadiusKm, color=planetColor))

        rDataList = []
        fDataList = []
        for idx in range(positionData.shape[0]):
            elementsStep = orbitalMotion.rv2elem(
                gravitationalParameter,
                positionData[idx, 0:3],
                velocityData[idx, 0:3],
            )
            rDataList.append(elementsStep.rmag)
            fDataList.append(elementsStep.f + elementsStep.omega - elementsInit.omega)

        rDataArray = np.array(rDataList)
        fDataArray = np.array(fDataList)

        plt.plot(
            rDataArray * np.cos(fDataArray) / 1e3,
            rDataArray * np.sin(fDataArray) / 1e3,
            color="#aa0000",
            linewidth=3.0,
        )

        fGrid = np.linspace(0.0, 2.0 * np.pi, 200)
        rGrid = pParam / (1.0 + elementsInit.e * np.cos(fGrid))
        plt.plot(
            rGrid * np.cos(fGrid) / 1e3,
            rGrid * np.sin(fGrid) / 1e3,
            "--",
            color="#555555",
        )
        plt.xlabel("$i_e$ coord [km]")
        plt.ylabel("$i_p$ coord [km]")
        plt.grid()

        # Semi major axis vs time
        plt.figure()
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style="plain")
        smaData = []
        for idx in range(positionData.shape[0]):
            elementsStep = orbitalMotion.rv2elem(
                gravitationalParameter,
                positionData[idx, 0:3],
                velocityData[idx, 0:3],
            )
            smaData.append(elementsStep.a / 1e3)
        smaData = np.array(smaData)
        plt.plot(timesOrbit, smaData, color="#aa0000")
        plt.xlabel("Time [orbits]")
        plt.ylabel("SMA [km]")

        # Density vs time
        plt.figure()
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style="sci")
        timesAtmoSec = atmoRecorder.times() * macros.NANO2SEC
        plt.plot(timesAtmoSec, densityData)
        plt.title("Density vs time")
        plt.xlabel("Time [s]")
        plt.ylabel("Density [kg/m^3]")

        # Drag vs reference
        plt.figure()
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style="plain")
        for idx in range(3):
            plt.plot(timesOrbit, dragForceData[:, idx], label=f"F_mod_{idx}")
            plt.plot(timesOrbit, referenceDrag[:, idx], "--", label=f"F_ref_{idx}")
        plt.xlabel("Time [orbits]")
        plt.ylabel("Drag force [N]")
        plt.legend()
        plt.title("Drag force vs reference")

        plt.show()


if __name__ == "__main__":
    assert couldImportMujoco
    # test_aerodynamicDrag()
    test_orbit("LTO","earth",True)
