import unittest
from src import fault


class TestFID(unittest.TestCase):
    def test_run_fid_returns_expected_tuple(self):
        """
        This test runs the actual FID.run_fid function with sweep_window=1,
        and asserts that the result is a tuple of expected structure:
        (fail_rate: float, avg_delay: float, u_hist: list)
        """
        result = fault.FID.run_fid(sweep_window=1)

        self.assertIsInstance(result, tuple, "Result should be a tuple")
        self.assertEqual(len(result), 3, "Tuple should have 3 elements")

        fail_rate, avg_delay, u_hist = result

        self.assertIsInstance(fail_rate, float, "fail_rate should be float")
        self.assertIsInstance(avg_delay, float, "avg_delay should be float")
        self.assertIsInstance(u_hist, list, "u_hist should be list")


if __name__ == '__main__':
    unittest.main()
