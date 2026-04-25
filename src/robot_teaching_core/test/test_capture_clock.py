import unittest

from teaching_core.capture.clock import ClockCalibrator


class ClockCalibratorTest(unittest.TestCase):
    def test_calibrates_remote_timestamp_into_local_time(self):
        calibrator = ClockCalibrator(window_size=3)

        corrected = calibrator.calibrate(t1_remote=10.0, t2_local=10.125)

        self.assertAlmostEqual(corrected, 10.125)
        self.assertTrue(calibrator.is_calibrated)
        self.assertAlmostEqual(
            calibrator.get_current_offset(now_fn=lambda: 10.5),
            0.125,
        )

    def test_large_clock_jump_resets_window(self):
        calibrator = ClockCalibrator(window_size=3)
        calibrator.calibrate(10.0, 10.100)
        calibrator.calibrate(10.1, 10.205)

        corrected_after_jump = calibrator.calibrate(10.2, 11.000)

        self.assertAlmostEqual(corrected_after_jump, 11.000)
        self.assertEqual(len(calibrator.delay_history), 1)
        self.assertAlmostEqual(calibrator.delay_history[0], 0.8)


if __name__ == "__main__":
    unittest.main()
