#!/usr/bin/env python3
import os
import time
from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import RadarInterfaceBase

RADAR_A_MSGS = list(range(0x310, 0x36F, 3))
RADAR_B_MSGS = list(range(0x311, 0x36F, 3))

def _create_radard_can_parser(car_fingerprint):

    msg_a_n = len(RADAR_A_MSGS)
    msg_b_n = len(RADAR_B_MSGS)

    signals = list(
        zip(
            ["LongDist"] * msg_a_n
            + ["LatDist"] * msg_a_n
            + ["LongSpeed"] * msg_a_n
            + ["LongAccel"] * msg_a_n
            + ["Valid"] * msg_a_n
            + ["Tracked"] * msg_a_n
            + ["Meas"] * msg_a_n
            + ["ProbExist"] * msg_a_n
            + ["Index"] * msg_a_n
            + ["ProbObstacle"] * msg_a_n
            + ["LatSpeed"] * msg_b_n
            + ["Index2"] * msg_b_n
            + ["Class"] * msg_b_n
            + ["ProbClass"] * msg_b_n
            + ["Length"] * msg_b_n
            + ["dZ"] * msg_b_n
            + ["MovingState"] * msg_b_n,
            RADAR_A_MSGS * 10 + RADAR_B_MSGS * 7,
            [255.0] * msg_a_n
            + [0.0] * msg_a_n
            + [0.0] * msg_a_n
            + [0.0] * msg_a_n
            + [0] * msg_a_n
            + [0] * msg_a_n
            + [0] * msg_a_n
            + [0.0] * msg_a_n
            + [0] * msg_a_n
            + [0.0] * msg_a_n
            + [0.0] * msg_b_n
            + [0] * msg_b_n
            + [0] * msg_b_n
            + [0.0] * msg_b_n
            + [0.0] * msg_b_n
            + [0.0] * msg_b_n
            + [0] * msg_b_n,
        )
    )

    checks = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [6] * (msg_a_n + msg_b_n)))

    return CANParser(DBC[car_fingerprint]['radar'], signals, checks, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.track_id = 0
    self.radar_ts = CP.radarTimeStep

    self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}

    self.rcp = _create_radar_can_parser(CP.carFingerprint)
    self.trigger_msg = self.RADAR_B_MSGS[-1]
    self.updated_messages = set()

  def update(self, can_strings):
    # radard at 20Hz and return no points
    if self.radar_off_can:
      return super().update(None)
          
      vls = self.rcp.update_strings(can_strings)
      self.updated_messages.update(vls)

      if self.trigger_msg not in self.updated_messages:
        return None

      rr = self._update(self.updated_messages)
      self.updated_messages.clear()
      
      return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors
    return ret
