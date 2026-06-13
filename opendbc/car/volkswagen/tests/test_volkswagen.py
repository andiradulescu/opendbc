import random
import re
import unittest

from opendbc.car import DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.carcontroller import AccelResumeRamp, HCAMitigation
from opendbc.car.volkswagen.values import CAR, CarControllerParams as CCP, FW_QUERY_CONFIG, WMI
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenHCAMitigation(unittest.TestCase):
  STUCK_TORQUE_FRAMES = round(CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP))

  def test_same_torque_mitigation(self):
    """Same-torque nudge fires at the threshold, in the correct direction, and resets cleanly."""
    hca_mitigation = HCAMitigation(CCP)

    for actuator_value in (-CCP.STEER_MAX, -1, 0, 1, CCP.STEER_MAX):
      hca_mitigation.update(0, 0)  # Reset mitigation state
      for frame in range(self.STUCK_TORQUE_FRAMES + 2):
        should_nudge = actuator_value != 0 and frame == self.STUCK_TORQUE_FRAMES
        expected_torque = actuator_value - (1, -1)[actuator_value < 0] if should_nudge else actuator_value
        assert hca_mitigation.update(actuator_value, actuator_value) == expected_torque, f"{frame=}"

class TestVolkswagenAccelResumeRamp(unittest.TestCase):
  MAX_DELTA = CCP.ACCEL_RESUME_JERK * DT_CTRL * CCP.ACC_CONTROL_STEP

  def test_passthrough_while_inactive(self):
    """While not regulating, the command is untouched and the ramp tracks the vehicle for the next resume."""
    ramp = AccelResumeRamp(CCP)
    for a_ego in (-1.0, 0.0, 0.5):
      assert ramp.update(0.0, False, a_ego) == 0.0

  def test_resume_ramps_from_vehicle_not_step(self):
    """On resume after an override, the command ramps from the vehicle's accel toward the target, jerk-limited."""
    ramp = AccelResumeRamp(CCP)
    a_ego, target = 0.17, -0.45  # the 00000063 seg4 handover
    ramp.update(0.0, False, a_ego)  # driver gas override: seed from the vehicle

    first = ramp.update(target, True, a_ego)
    assert first != target, "resume stepped straight to the target"
    assert abs(first - a_ego) <= self.MAX_DELTA + 1e-9, "first resume frame exceeded the jerk limit"

    prev, reached = first, False
    for _ in range(50):
      out = ramp.update(target, True, a_ego)
      assert abs(out - prev) <= self.MAX_DELTA + 1e-9, "ramp exceeded the jerk limit mid-catch-up"
      prev = out
      if abs(out - target) < 1e-9:
        reached = True
        break
    assert reached, "ramp never caught up to the target"

  def test_symmetric_accel_up(self):
    """The ramp limits the accel direction too, not just decel."""
    ramp = AccelResumeRamp(CCP)
    a_ego, target = -0.5, 1.5
    ramp.update(0.0, False, a_ego)
    first = ramp.update(target, True, a_ego)
    assert first != target
    assert abs(first - a_ego) <= self.MAX_DELTA + 1e-9

  def test_no_limit_after_catchup(self):
    """Once caught up, normal braking/accel passes through unlimited (safety: no delayed braking)."""
    ramp = AccelResumeRamp(CCP)
    ramp.update(0.0, False, 0.0)
    for _ in range(100):  # drive long enough to catch up to a steady target
      ramp.update(0.0, True, 0.0)
    # a genuine hard-braking step during sustained control must not be jerk-limited
    assert ramp.update(-3.5, True, 0.0) == -3.5

class TestVolkswagenPlatformConfigs(unittest.TestCase):
  def test_spare_part_fw_pattern(self):
    # Relied on for determining if a FW is likely VW
    for platform, ecus in FW_VERSIONS.items():
      with self.subTest(platform=platform.value):
        for fws in ecus.values():
          for fw in fws:
            assert SPARE_PART_FW_PATTERN.match(fw) is not None, f"Bad FW: {fw}"

  def test_chassis_codes(self):
    for platform in CAR:
      with self.subTest(platform=platform.value):
        assert len(platform.config.wmis) > 0, "WMIs not set"
        assert len(platform.config.chassis_codes) > 0, "Chassis codes not set"
        assert all(CHASSIS_CODE_PATTERN.match(cc) for cc in
                   platform.config.chassis_codes), "Bad chassis codes"

        # No two platforms should share chassis codes
        for comp in CAR:
          if platform == comp:
            continue
          assert set() == platform.config.chassis_codes & comp.config.chassis_codes, \
                           f"Shared chassis codes: {comp}"

  def test_custom_fuzzy_fingerprinting(self):
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus[Ecu.fwdRadar, 0x757, None]})

    for platform in CAR:
      with self.subTest(platform=platform.name):
        for wmi in WMI:
          for chassis_code in platform.config.chassis_codes | {"00"}:
            vin = ["0"] * 17
            vin[0:3] = wmi
            vin[6:8] = chassis_code
            vin = "".join(vin)

            # Check a few FW cases - expected, unexpected
            for radar_fw in random.sample(all_radar_fw, 5) + [b'\xf1\x875Q0907572G \xf1\x890571', b'\xf1\x877H9907572AA\xf1\x890396']:
              should_match = ((wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes) and
                              radar_fw in all_radar_fw)

              live_fws = {(0x757, None): [radar_fw]}
              matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

              expected_matches = {platform} if should_match else set()
              assert expected_matches == matches, "Bad match"
