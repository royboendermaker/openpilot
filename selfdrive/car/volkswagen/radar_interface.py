#!/usr/bin/env python3
from opendbc.can.parser import CANParser
from cereal import car
from selfdrive.car.interfaces import RadarInterfaceBase

def _create_radar_can_parser(CP):
  RADAR_A_MSGS = list(range(0x310, 0x36F))
  RADAR_B_MSGS = list(range(0x311, 0x36F))

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

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.track_id = 0
    self.radar_ts = CP.radarTimeStep

    self.RADAR_A_MSGS = list(range(0x310, 0x36F))
    self.RADAR_B_MSGS = list(range(0x311, 0x36F))

    self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}

    self.rcp = _create_radar_can_parser(CP.carFingerprint)
    self.trigger_msg = self.RADAR_B_MSGS[-1]
    self.updated_messages = set()


  def update(self, can_strings):
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

    for ii in sorted(updated_messages):
      if ii in self.RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]

        if cpt['LONG_DIST'] >= 255 or cpt['NEW_TRACK']:
          self.valid_cnt[ii] = 0    # reset counter
        if cpt['VALID'] and cpt['LONG_DIST'] < 255:
          self.valid_cnt[ii] += 1
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)

        score = self.rcp.vl[ii+16]['SCORE']
        # print ii, self.valid_cnt[ii], score, cpt['VALID'], cpt['LONG_DIST'], cpt['LAT_DIST']

        # radar point only valid if it's a valid measurement and score is above 50
        if cpt['VALID'] or (score > 50 and cpt['LONG_DIST'] < 255 and self.valid_cnt[ii] > 0):
          if ii not in self.pts or cpt['NEW_TRACK']:
            self.pts[ii] = car.RadarData.RadarPoint.new_message()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          self.pts[ii].dRel = cpt['LONG_DIST']  # from front of car
          self.pts[ii].yRel = -cpt['LAT_DIST']  # in car frame's y axis, left is positive
          self.pts[ii].vRel = cpt['REL_SPEED']
          self.pts[ii].aRel = float('nan')
          self.pts[ii].yvRel = float('nan')
          self.pts[ii].measured = bool(cpt['VALID'])
        else:
          if ii in self.pts:
            del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
