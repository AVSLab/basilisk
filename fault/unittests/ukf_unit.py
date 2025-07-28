import unittest

import numpy as np
from src.ukf import UKFSetup

class DummyMsg:
    def subscribeTo(self, msg):
        self.subscribed_msg = msg

class DummyEstimator:
    def __init__(self):
        self.gyrBuffInMsg = DummyMsg()
        self.rwParamsInMsg = DummyMsg()
        self.rwSpeedsInMsg = DummyMsg()
        self.alpha = None
        self.beta = None
        self.kappa = None
        self.switchMag = None
        self.stateInit = None
        self.covarInit = None
        self.qNoise = None

    def logger(self, args, timestep):
        self.logger_args = args
        self.logger_timestep = timestep
        return self._logger

    _logger = object()

class DummyScSim:
    def __init__(self):
        self.models_added = []

    def AddModelToTask(self, taskName, model):
        self.models_added.append((taskName, model))

class DummyRwStateEffector:
    def __init__(self):
        self.rwSpeedOutMsg = object()

class DummyFswRwParamMsg:
    pass

class UKFSetupTest(UKFSetup):
    # Override to return dummy estimator instead of real inertialUKF
    def create_and_setup_filter(self, scSim, simTaskName):
        attEstimator = DummyEstimator()
        scSim.AddModelToTask(simTaskName, attEstimator)

        attEstimator.alpha = self.filterData.alpha
        attEstimator.beta = self.filterData.beta
        attEstimator.kappa = self.filterData.kappa
        attEstimator.switchMag = self.filterData.switchMag

        attEstimator.stateInit = self.filterData.stateInit
        attEstimator.covarInit = self.filterData.covarInit
        attEstimator.qNoise = self.filterData.qNoise

        gyroInMsg = object()
        attEstimator.gyrBuffInMsg.subscribeTo(gyroInMsg)

        attEstimator.rwParamsInMsg.subscribeTo(self.fswRwParamMsg)
        attEstimator.rwSpeedsInMsg.subscribeTo(self.rwStateEffector.rwSpeedOutMsg)

        attEstimatorLog = attEstimator.logger(["covar", "state"], self.simulationTimeStep)
        scSim.AddModelToTask(simTaskName, attEstimatorLog)

        return attEstimator, attEstimatorLog

class TestUKFSetupIntegration(unittest.TestCase):
    def setUp(self):
        self.simTimeStep = 0.1
        self.rwStateEffector = DummyRwStateEffector()
        self.fswRwParamMsg = DummyFswRwParamMsg()
        self.scSim = DummyScSim()
        self.simTaskName = "fswTask"

        self.ukfSetup = UKFSetupTest(self.simTimeStep, self.fswRwParamMsg, self.rwStateEffector)

    def test_create_and_setup_filter(self):
        attEstimator, attEstimatorLog = self.ukfSetup.create_and_setup_filter(self.scSim, self.simTaskName)

        # Check if models were added to the task
        models = [model for task, model in self.scSim.models_added if task == self.simTaskName]

        self.assertIn(attEstimator, models)
        self.assertIn(attEstimatorLog, models)

        # Check if subscriptions are done
        self.assertTrue(hasattr(attEstimator.gyrBuffInMsg, 'subscribed_msg'))
        self.assertIs(attEstimator.rwParamsInMsg.subscribed_msg, self.fswRwParamMsg)
        self.assertIs(attEstimator.rwSpeedsInMsg.subscribed_msg, self.rwStateEffector.rwSpeedOutMsg)

        # Check that filter parameters are set (not None)
        self.assertIsNotNone(attEstimator.alpha)
        self.assertIsNotNone(attEstimator.beta)
        self.assertIsNotNone(attEstimator.kappa)
        self.assertIsNotNone(attEstimator.switchMag)
        self.assertIsNotNone(attEstimator.stateInit)
        self.assertIsNotNone(attEstimator.covarInit)
        self.assertIsNotNone(attEstimator.qNoise)

if __name__ == "__main__":
    unittest.main()
