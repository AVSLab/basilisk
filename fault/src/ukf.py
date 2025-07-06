import numpy as np
from Basilisk import messaging
from Basilisk.fswAlgorithms import inertialUKF

from src.parameters import NoiseParameters, FilterData

class UKFSetup:
    def __init__(self, simulationTimeStep, fswRwParamMsg, rwStateEffector):
        self.simulationTimeStep = simulationTimeStep
        self.fswRwParamMsg = fswRwParamMsg
        self.rwStateEffector = rwStateEffector

        self.noiseParams = NoiseParameters()
        self.filterData = FilterData()

    def create_and_setup_filter(self, scSim, simTaskName):
        attEstimator = inertialUKF.inertialUKF()
        scSim.AddModelToTask(simTaskName, attEstimator)

        attEstimator.alpha = self.filterData.alpha
        attEstimator.beta = self.filterData.beta
        attEstimator.kappa = self.filterData.kappa
        attEstimator.switchMag = self.filterData.switchMag

        attEstimator.stateInit = self.filterData.stateInit
        attEstimator.covarInit = self.filterData.covarInit
        attEstimator.qNoise = self.filterData.qNoise

        gyroInMsg = messaging.AccDataMsg()
        attEstimator.gyrBuffInMsg.subscribeTo(gyroInMsg)

        attEstimator.rwParamsInMsg.subscribeTo(self.fswRwParamMsg)
        attEstimator.rwSpeedsInMsg.subscribeTo(self.rwStateEffector.rwSpeedOutMsg)

        attEstimatorLog = attEstimator.logger(["covar", "state"], self.simulationTimeStep)
        scSim.AddModelToTask(simTaskName, attEstimatorLog)

        return attEstimator, attEstimatorLog
