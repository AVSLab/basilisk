from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.architecture import messaging, bskLogging

# Plugin defined module
from custom_atm import customExponentialAtmosphere


# Plugin defined message
from custom_atm.messaging import CustomAtmStatusMsgPayload, CustomAtmStatusMsg


def main():
    sim = SimulationBaseClass.SimBaseClass()
    sim.bskLogger.setLogLevel(bskLogging.BSK_INFORMATION)

    proc = sim.CreateNewProcess("proc")
    task = "task"
    dt = macros.sec2nano(1.0)
    proc.addTask(sim.CreateNewTask(task, dt))

    atmosphere = customExponentialAtmosphere.CustomExponentialAtmosphere()

    atmosphere.planetRadius = 6371e3
    atmosphere.envMinReach = -1.0
    atmosphere.envMaxReach = -1.0

    atmosphere.baseDensity = 1.225
    atmosphere.scaleHeight = 8500.0
    atmosphere.localTemp = 293.0

    sim.AddModelToTask(task, atmosphere)

    # Custom message setup
    status_pl = CustomAtmStatusMsgPayload()
    status_pl.density = atmosphere.baseDensity
    status_pl.scaleHeight = atmosphere.scaleHeight
    status_pl.modelValid = 1

    status_msg = CustomAtmStatusMsg().write(status_pl)
    atmosphere.connectAtmStatus(status_msg)

    sc_pl = messaging.SCStatesMsgPayload()
    sc_pl.r_BN_N = [atmosphere.planetRadius + 400e3, 0.0, 0.0]
    sc_pl.v_BN_N = [0.0, 0.0, 0.0]

    sc_msg = messaging.SCStatesMsg().write(sc_pl)

    atmosphere.addSpacecraftToModel(sc_msg)

    log = atmosphere.envOutMsgs[0].recorder()
    sim.AddModelToTask(task, log)

    sim.InitializeSimulation()
    print("Recorder length initially:", len(log.neutralDensity))

    for k in range(5):
        # Just to show that the plugin overwrites this value
        status_pl.density = 1.225 * (1.0 + 0.2 * k)
        status_msg.write(status_pl)

        sim.ConfigureStopTime((k + 1) * dt)
        sim.ExecuteSimulation()

        print(
            f"Step {k}: "
            f"status density={status_pl.density:.3f}, "
            f"neutralDensity={log.neutralDensity[-1]:.3e}"
        )
        print("Recorder length now:", len(log.neutralDensity))


if __name__ == "__main__":
    main()
