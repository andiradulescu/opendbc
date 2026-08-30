#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.safety.tests import common
from opendbc.safety.tests.libsafety import libsafety_py


EPS_DIAG_ADDR = 0x712
EPS_DIAG_BUS = 1


class TestVolkswagenMqbEpsDiagnostics(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety

  @staticmethod
  def _msg(did, service=0x22, length=3, padding=b"\x00\x00\x00\x00", bus=EPS_DIAG_BUS):
    dat = bytes([length, service, did >> 8, did & 0xFF]) + padding
    return common.make_msg(bus, EPS_DIAG_ADDR, dat=dat)

  def test_read_only_did_whitelist(self):
    for safety_param in (0, VolkswagenSafetyFlags.LONG_CONTROL):
      with self.subTest(safety_param=safety_param):
        self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagen, safety_param)
        self.safety.init_tests()

        self.assertTrue(self.safety.safety_tx_hook(self._msg(0x180B)))
        self.assertTrue(self.safety.safety_tx_hook(self._msg(0x1823)))

        self.assertFalse(self.safety.safety_tx_hook(self._msg(0x180C)))
        self.assertFalse(self.safety.safety_tx_hook(self._msg(0x180B, service=0x2E)))
        self.assertFalse(self.safety.safety_tx_hook(self._msg(0x180B, length=4)))
        self.assertFalse(self.safety.safety_tx_hook(self._msg(0x180B, padding=b"\x01\x00\x00\x00")))
        self.assertFalse(self.safety.safety_tx_hook(self._msg(0x180B, bus=0)))


if __name__ == "__main__":
  unittest.main()
