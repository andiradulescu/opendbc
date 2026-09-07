#!/usr/bin/env python3
import unittest

from opendbc.car.volkswagen.carcontroller import HCAProbe


class Params:
  STEER_DELTA_UP = 4
  STEER_DELTA_DOWN = 10


class TestHCAProbe(unittest.TestCase):
  def setUp(self):
    self.probe = HCAProbe(Params(), True)

  def update(self, nominal=300, output_last=300, armed=True, now=0, enabled=True, lat_active=True, speed=3.0,
             driver=0, fault_temp=False, fault_perm=False):
    return self.probe.update(nominal, output_last, armed, now, enabled, lat_active, speed, driver, fault_temp, fault_perm)

  def test_unarmed_is_exact_nominal(self):
    for nominal in (-300, -123, 0, 123, 300):
      self.assertEqual(nominal, self.update(nominal=nominal, output_last=nominal, armed=False))

  def test_one_shot_ramp_and_recovery(self):
    output = 300
    commands = []
    now = 1_000_000_000
    for _ in range(10):
      output = self.update(output_last=output, now=now)
      commands.append(output)
      now += 20_000_000
    self.assertEqual([304, 308, 312, 316, 320, 320, 320, 320, 320, 320], commands)
    self.assertEqual(10, self.probe.frames)

    # Frame budget expires: recover at the normal rate-down limit, then mark the one-shot done.
    output = self.update(output_last=output, now=now)
    self.assertEqual(310, output)
    output = self.update(output_last=output, now=now + 20_000_000)
    self.assertEqual(300, output)
    self.assertTrue(self.probe.done)
    self.assertEqual(300, self.update(output_last=300, now=now + 40_000_000))
    # A new arm edge in the same controller process cannot repeat the one-shot.
    self.assertEqual(300, self.update(output_last=300, armed=False, now=now + 60_000_000))
    self.assertEqual(300, self.update(output_last=300, armed=True, now=now + 80_000_000))

  def test_fault_driver_speed_and_disarm_force_recovery(self):
    for kwargs in ({"driver": 51}, {"speed": 0.5}, {"speed": 5.1}, {"fault_temp": True}, {"fault_perm": True}, {"enabled": False}, {"lat_active": False}, {"armed": False}):
      self.probe = HCAProbe(Params(), True)
      # Start at 304.
      self.assertEqual(304, self.update(output_last=300, now=1_000_000_000))
      self.assertEqual(300, self.update(output_last=304, now=1_020_000_000, **kwargs))
      self.assertTrue(self.probe.done)

  def test_lat_disengage_recovers_at_rate_down_to_zero(self):
    self.assertEqual(304, self.update(output_last=300, now=1_000_000_000))
    out = 304
    expected = list(range(294, -1, -10)) + [0]
    got = []
    now = 1_020_000_000
    for _ in expected:
      out = self.update(nominal=0, output_last=out, now=now, lat_active=False)
      got.append(out)
      now += 20_000_000
    self.assertEqual(expected, got)
    self.assertTrue(self.probe.done)

  def test_arm_deadline_is_controller_local(self):
    self.assertEqual(304, self.update(output_last=300, now=1_000_000_000))
    # A stale host arm cannot keep the controller probe alive.
    self.assertEqual(300, self.update(output_last=304, now=16_000_000_001))
    self.assertTrue(self.probe.done)

  def test_direction_change_recovers_before_following_nominal(self):
    self.assertEqual(304, self.update(output_last=300, now=1_000_000_000))
    self.assertEqual(294, self.update(nominal=-300, output_last=304, now=1_020_000_000))

  def test_unsupported_platform_never_changes_command(self):
    probe = HCAProbe(Params(), False)
    self.assertEqual(300, probe.update(300, 300, True, 1_000_000_000, True, True, 3.0, 0, False, False))


if __name__ == "__main__":
  unittest.main()
