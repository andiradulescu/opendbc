"""Diagnostics must not widen any MQB steering authority or accept unrelated UDS."""
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.safety.tests import common, test_volkswagen_mqb


class TestMqbDiagnosticIsolation(unittest.TestCase):
  def setUp(self):
    self.case = test_volkswagen_mqb.TestVolkswagenMqbStockSafety()
    self.case.setUp()
    self.safety = self.case.safety

  def reset(self, param, torque=300, driver=0, speed=18):
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagen, param)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.case._reset_speed_measurement(speed)
    self.case._reset_torque_driver_measurement(driver)
    self.case._set_prev_torque(torque)

  def test_over_300_rejected_at_all_test_speeds(self):
    for param in (0, VolkswagenSafetyFlags.LONG_CONTROL):
      for speed in (0, 1.8, 18, 36, 72):
        for sign in (-1, 1):
          for torque in (301, 304, 312, 320, 350):
            with self.subTest(param=param, speed=speed, sign=sign, torque=torque):
              self.reset(param, 300 * sign, speed=speed)
              self.assertFalse(self.case._tx(self.case._torque_cmd_msg(torque * sign)))

  def test_original_driver_boundary_both_directions(self):
    for param in (0, VolkswagenSafetyFlags.LONG_CONTROL):
      for sign in (-1, 1):
        self.reset(param, 270 * sign, -90 * sign)
        self.assertTrue(self.case._tx(self.case._torque_cmd_msg(270 * sign)))
        self.reset(param, 271 * sign, -90 * sign)
        self.assertFalse(self.case._tx(self.case._torque_cmd_msg(271 * sign)))

  def test_rejected_commands_do_not_authorize_a_later_jump(self):
    self.reset(0, 300, speed=72)
    for torque in (304, 308, 312, 316, 320):
      self.assertFalse(self.case._tx(self.case._torque_cmd_msg(torque)))
    self.case._reset_speed_measurement(18)
    self.assertFalse(self.case._tx(self.case._torque_cmd_msg(320)))
    self.assertTrue(self.case._tx(self.case._torque_cmd_msg(0)))

  def test_exact_diagnostic_payload_only(self):
    allowed = (bytes.fromhex("0322180b00000000"), bytes.fromhex("0322182300000000"))
    for param in (0, VolkswagenSafetyFlags.LONG_CONTROL):
      self.reset(param)
      for controls_allowed in (False, True):
        self.safety.set_controls_allowed(controls_allowed)
        for data in allowed:
          for bus in range(4):
            self.assertEqual(bus == 1, bool(self.safety.safety_tx_hook(common.make_msg(bus, 0x712, dat=data))))
          for size in range(8):
            self.assertFalse(self.safety.safety_tx_hook(common.make_msg(1, 0x712, dat=data[:size])))
          for i in range(8):
            for bit in range(8):
              changed = bytearray(data)
              changed[i] ^= 1 << bit
              self.assertFalse(self.safety.safety_tx_hook(common.make_msg(1, 0x712, dat=bytes(changed))))
        for data in ("02104f0000000000", "0322111000000000", "032e180b00000000", "023e000000000000", "3000000000000000"):
          self.assertFalse(self.safety.safety_tx_hook(common.make_msg(1, 0x712, dat=bytes.fromhex(data))))


if __name__ == "__main__":
  unittest.main()
