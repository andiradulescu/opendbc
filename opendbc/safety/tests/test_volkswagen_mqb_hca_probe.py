#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests import test_volkswagen_mqb
from opendbc.safety.tests.libsafety import libsafety_py


class TestVolkswagenMqbHcaProbe(unittest.TestCase):
  def setUp(self):
    self.case = test_volkswagen_mqb.TestVolkswagenMqbStockSafety()
    self.case.setUp()
    self.safety = libsafety_py.libsafety

  def reset(self, probe=False, torque=300, driver=0, speed_kph=10.8):
    param = 4 if probe else 0
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagen, param)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.case._reset_speed_measurement(speed_kph)
    self.case._reset_torque_driver_measurement(driver)
    self.case._set_prev_torque(torque)
    self.safety.set_timer(0)

  def tx(self, torque, timer_us):
    self.safety.set_timer(timer_us)
    return self.case._tx(self.case._torque_cmd_msg(torque))

  def test_capability_absent_is_stock_300(self):
    for sign in (-1, 1):
      self.reset(False, 300 * sign)
      self.assertFalse(self.tx(304 * sign, 20_000))
      # Stock violation semantics reset steering history, so a full-scale command cannot follow immediately.
      self.assertFalse(self.tx(300 * sign, 40_000))
      self.assertTrue(self.tx(0, 60_000))

  def test_capability_allows_bounded_ramp(self):
    for sign in (-1, 1):
      self.reset(True, 300 * sign)
      timer = 20_000
      for torque in (304, 308, 312, 316, 320):
        self.assertTrue(self.tx(torque * sign, timer))
        timer += 20_000
      self.assertFalse(self.tx(321 * sign, timer))

  def test_speed_and_driver_gates_do_not_advance_history(self):
    for speed_kph, driver in ((0.0, 0), (21.6, 0), (10.8, 51), (10.8, -51)):
      self.reset(True, 300, driver=driver, speed_kph=speed_kph)
      self.assertFalse(self.tx(304, 20_000))
      # Make the gate valid without resetting safety. If the rejected 304 did not advance history, 304 is the next valid +4 step.
      self.case._reset_speed_measurement(10.8)
      self.case._reset_torque_driver_measurement(0)
      self.assertTrue(self.tx(304, 40_000))

  def test_frame_budget_is_one_shot_and_recovery_is_allowed(self):
    self.reset(True, 300)
    timer = 20_000
    for torque in (304, 308, 312, 316, 320, 320, 320, 320, 320, 320):
      self.assertTrue(self.tx(torque, timer))
      timer += 20_000
    self.assertFalse(self.tx(320, timer))
    # Once used, reducing toward stock is allowed; returning <=300 closes the probe permanently.
    self.assertTrue(self.tx(310, timer + 20_000))
    self.assertTrue(self.tx(300, timer + 40_000))
    self.case._set_prev_torque(300)
    self.assertFalse(self.tx(304, timer + 60_000))

  def test_time_budget(self):
    self.reset(True, 300)
    self.assertTrue(self.tx(304, 20_000))
    # Window elapsed: same/increasing experimental torque is blocked without changing history.
    self.assertFalse(self.tx(308, 300_001))
    # If the blocked 308 had advanced history, 294 would exceed the 10 cNm rate-down allowance from 308.
    self.assertTrue(self.tx(294, 320_000))

  def test_default_driver_boundary_remains_unchanged(self):
    for sign in (-1, 1):
      self.reset(False, 270 * sign, driver=-90 * sign)
      self.assertTrue(self.tx(270 * sign, 20_000))
      self.reset(False, 271 * sign, driver=-90 * sign)
      self.assertFalse(self.tx(271 * sign, 20_000))


if __name__ == "__main__":
  unittest.main()
