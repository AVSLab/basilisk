import unittest
from unittest.mock import MagicMock, patch, call

from src.ukf import UKFSetup

class TestUKFSetupWithMocks(unittest.TestCase):

    def setUp(self):
        self.simulationTimeStep = 0.1
        self.fswRwParamMsg = MagicMock(name="FswRwParamMsg")
        self.rwStateEffector = MagicMock(name="RwStateEffector")
        self.rwStateEffector.rwSpeedOutMsg = MagicMock(name="RwSpeedOutMsg")

        self.scSim = MagicMock(name="SimInterface")
        self.simTaskName = "fswTask"

        self.ukf_setup = UKFSetup(self.simulationTimeStep, self.fswRwParamMsg, self.rwStateEffector)

    @patch('src.ukf.inertialUKF.inertialUKF', autospec=True)
    def test_create_setup_and_update_filter(self, MockInertialUKF):
        # Arrange: mock the filter instance and its methods/attributes
        mock_estimator = MagicMock(name="InertialUKFInstance")
        MockInertialUKF.return_value = mock_estimator

        # mock logger returned by .logger()
        mock_logger = MagicMock(name="LoggerInstance")
        mock_estimator.logger.return_value = mock_logger

        # Mock messages for subscribing
        mock_estimator.gyrBuffInMsg = MagicMock(name="GyrBuffInMsg")
        mock_estimator.rwParamsInMsg = MagicMock(name="RwParamsInMsg")
        mock_estimator.rwSpeedsInMsg = MagicMock(name="RwSpeedsInMsg")

        # Act: create and setup filter
        attEstimator, attEstimatorLog = self.ukf_setup.create_and_setup_filter(self.scSim, self.simTaskName)

        # Assert: creation calls
        MockInertialUKF.assert_called_once()
        mock_estimator.logger.assert_called_once_with(["covar", "state"], self.simulationTimeStep)

        # Assert: messages subscribe called properly
        mock_estimator.gyrBuffInMsg.subscribeTo.assert_called_once()
        mock_estimator.rwParamsInMsg.subscribeTo.assert_called_once_with(self.fswRwParamMsg)
        mock_estimator.rwSpeedsInMsg.subscribeTo.assert_called_once_with(self.rwStateEffector.rwSpeedOutMsg)

        # Assert: filter and logger are added to sim task
        calls = self.scSim.AddModelToTask.call_args_list
        # We expect calls adding both estimator and logger
        self.assertIn(call(self.simTaskName, mock_estimator), calls)
        self.assertIn(call(self.simTaskName, mock_logger), calls)

        # Assert returned objects
        self.assertIs(attEstimator, mock_estimator)
        self.assertIs(attEstimatorLog, mock_logger)

        # Simulate filter update call (assuming you have such method)
        if hasattr(attEstimator, 'update'):
            gyro_data = MagicMock(name="GyroMeasurement")
            attEstimator.update(gyro_data)
            attEstimator.update.assert_called_once_with(gyro_data)

if __name__ == "__main__":
    unittest.main()
