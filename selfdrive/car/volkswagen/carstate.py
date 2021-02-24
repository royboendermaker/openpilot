import numpy as np
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volkswagen.values import DBC, CANBUS, NWL, TRANS, GEAR, BUTTON_STATES, CarControllerParams

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    
    if True: #set this to False if you are not PQ
      self.shifter_values = can_define.dv["Getriebe_1"]['Waehlhebelposition__Getriebe_1_']
      if CP.enableGasInterceptor:
        self.openpilot_enabled = False

  def update(self, pt_cp):
    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds.fl = pt_cp.vl["Bremse_3"]['Radgeschw__VL_4_1'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["Bremse_3"]['Radgeschw__VR_4_1'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["Bremse_3"]['Radgeschw__HL_4_1'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["Bremse_3"]['Radgeschw__HR_4_1'] * CV.KPH_TO_MS

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)


    ret.standstill = ret.vEgoRaw < 0.1

    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    # the sign/direction in a separate signal so they must be recombined.
    ret.steeringAngle = pt_cp.vl["Lenkhilfe_3"]['LH3_BLW'] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]['LH3_BLWSign'])]
    ret.steeringRate = pt_cp.vl["Lenkwinkel_1"]['Lenkradwinkel_Geschwindigkeit'] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]['Lenkradwinkel_Geschwindigkeit_S'])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]['LH3_LM'] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]['LH3_LMSign'])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["Bremse_5"]['Giergeschwindigkeit'] * (1, -1)[int(pt_cp.vl["Bremse_5"]['Vorzeichen_der_Giergeschwindigk'])] * CV.DEG_TO_RAD

    # Update gas, brakes, and gearshift.
    if not self.CP.enableGasInterceptor:
      ret.gas = pt_cp.vl["Motor_3"]['Fahrpedal_Rohsignal'] / 100.0
      ret.gasPressed = ret.gas > 0
    else:
      ret.gas = (cam_cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cam_cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
      ret.gasPressed = ret.gas > 468

    ret.brake = pt_cp.vl["Bremse_5"]['Bremsdruck'] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]['Bremstestschalter'])
    ret.brakeLights = bool(pt_cp.vl["Motor_2"]['Bremslichtschalter'])

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = bool(pt_cp.vl["Kombi_1"]['Bremsinfo'])  # FIXME: need to include an EPB check as well
    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]['ESP_Passiv_getastet'])

    # Update gear and/or clutch position data.
    if trans_type == TRANS.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["Getriebe_1"]['Waehlhebelposition__Getriebe_1_'], None))
    elif trans_type == TRANS.manual:
      ret.clutchPressed = not pt_cp.vl["Motor_1"]['Kupplungsschalter']
      reverse_light = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Rueckfahr'])
      if reverse_light:
        ret.gearShifter = GEAR.reverse
      else:
        ret.gearShifter = GEAR.drive

    # Update door and trunk/hatch lid open status.
    # TODO: need to locate signals for other three doors if possible
    ret.doorOpen = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Fa_Tuerkont'])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = not bool(pt_cp.vl["Airbag_1"]["Gurtschalter_Fahrer"])

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    self.displayMetricUnits = not pt_cp.vl["Einheiten_1"]["MFA_v_Einheit_02"]

    self.ldw_lane_warning_left = False
    self.ldw_lane_warning_right = False
    self.ldw_side_dlc_tlc = False
    self.ldw_dlc = False
    self.ldw_tlc = False

    # Update ACC radar status.
    # FIXME: This is unfinished and not fully correct, need to improve further
    ret.cruiseState.available = bool(pt_cp.vl["GRA_Neu"]['GRA_Hauptschalt'])
    ret.cruiseState.enabled = True if pt_cp.vl["Motor_2"]['GRA_Status'] in [1, 2] else False

    # Set override flag for openpilot enabled state.
    if self.CP.enableGasInterceptor and pt_cp.vl["Motor_2"]['GRA_Status'] in [1, 2]:
      self.openpilot_enabled = True

    # Check if Gas or Brake pressed and cancel override
    if self.CP.enableGasInterceptor and (ret.gasPressed or ret.brakePressed):
      self.openpilot_enabled = False

    # Override openpilot enabled if gas interceptor installed
    if self.CP.enableGasInterceptor and self.openpilot_enabled:
      ret.cruiseState.enabled = True

    # Update ACC setpoint. When the setpoint reads as 255, the driver has not
    # yet established an ACC setpoint, so treat it as zero.
    ret.cruiseState.speed = pt_cp.vl["Motor_2"]['Soll_Geschwindigkeit_bei_GRA_Be'] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 70:  # 255 kph in m/s == no current setpoint
      ret.cruiseState.speed = 0

    # Update control button states for turn signals and ACC controls.
    self.buttonStates["accelCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Up_kurz']) or bool(pt_cp.vl["GRA_Neu"]['GRA_Up_lang'])
    self.buttonStates["decelCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Down_kurz']) or bool(pt_cp.vl["GRA_Neu"]['GRA_Down_lang'])
    self.buttonStates["cancel"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Abbrechen'])
    self.buttonStates["setCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Neu_Setzen'])
    self.buttonStates["resumeCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Recall'])
    self.buttonStates["gapAdjustCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Zeitluecke'])
    ret.leftBlinker = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Blinker_li'])
    ret.rightBlinker = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Blinker_re'])

    # Read ACC hardware button type configuration info that has to pass thru
    # to the radar. Ends up being different for steering wheel buttons vs
    # third stalk type controls.
    # TODO: Check to see what info we need to passthru and spoof on PQ
    self.graHauptschalter = pt_cp.vl["GRA_Neu"]['GRA_Hauptschalt']
    self.graSenderCoding = pt_cp.vl["GRA_Neu"]['GRA_Sender']
    self.graTypHauptschalter = False
    self.graButtonTypeInfo = False
    self.graTipStufe2 = False
    # Pick up the GRA_ACC_01 CAN message counter so we can sync to it for
    # later cruise-control button spamming.
    # FIXME: will need msg counter and checksum algo to spoof GRA_neu
    self.graMsgBusCounter = pt_cp.vl["GRA_Neu"]['GRA_Neu_Zaehler']

    # Check to make sure the electric power steering rack is configured to
    # accept and respond to HCA_01 messages and has not encountered a fault.
    self.steeringFault = pt_cp.vl["Lenkhilfe_2"]['LH2_Sta_HCA'] not in [1, 3, 5, 7]

    # Read ABS pump for checking in ACC braking is working.
    if self.CP.enableGasInterceptor:
      self.ABSWorking = pt_cp.vl["Bremse_8"]["BR8_Sta_ADR_BR"]
      self.currentSpeed = ret.vEgo

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("LH3_BLW", "Lenkhilfe_3", 0),                # Absolute steering angle
      ("LH3_BLWSign", "Lenkhilfe_3", 0),            # Steering angle sign
      ("LH3_LM", "Lenkhilfe_3", 0),                 # Absolute driver torque input
      ("LH3_LMSign", "Lenkhilfe_3", 0),             # Driver torque input sign
      ("LH2_Sta_HCA", "Lenkhilfe_2", 0),            # Steering rack HCA status
      ("Lenkradwinkel_Geschwindigkeit", "Lenkwinkel_1", 0),  # Absolute steering rate
      ("Lenkradwinkel_Geschwindigkeit_S", "Lenkwinkel_1", 0),  # Steering rate sign
      ("Radgeschw__VL_4_1", "Bremse_3", 0),         # ABS wheel speed, front left
      ("Radgeschw__VR_4_1", "Bremse_3", 0),         # ABS wheel speed, front right
      ("Radgeschw__HL_4_1", "Bremse_3", 0),         # ABS wheel speed, rear left
      ("Radgeschw__HR_4_1", "Bremse_3", 0),         # ABS wheel speed, rear right
      ("Giergeschwindigkeit", "Bremse_5", 0),       # Absolute yaw rate
      ("Vorzeichen_der_Giergeschwindigk", "Bremse_5", 0),  # Yaw rate sign
      ("GK1_Fa_Tuerkont", "Gate_Komf_1", 0),     # Door open, driver
      # TODO: locate passenger and rear door states
      ("GK1_Blinker_li", "Gate_Komf_1", 0),         # Left turn signal on
      ("GK1_Blinker_re", "Gate_Komf_1", 0),         # Right turn signal on
      ("Gurtschalter_Fahrer", "Airbag_1", 0),       # Seatbelt status, driver
      ("Gurtschalter_Beifahrer", "Airbag_1", 0),    # Seatbelt status, passenger
      ("Bremstestschalter", "Motor_2", 0),          # Brake pedal pressed (brake light test switch)
      ("Bremslichtschalter", "Motor_2", 0),         # Brakes applied (brake light switch)
      ("Bremsdruck", "Bremse_5", 0),                # Brake pressure applied
      ("Vorzeichen_Bremsdruck", "Bremse_5", 0),     # Brake pressure applied sign (???)
      ("Fahrpedal_Rohsignal", "Motor_3", 0),        # Accelerator pedal value
      ("ESP_Passiv_getastet", "Bremse_1", 0),       # Stability control disabled
      ("MFA_v_Einheit_02", "Einheiten_1", 0),       # MPH vs KMH speed display
      ("Bremsinfo", "Kombi_1", 0),                  # Manual handbrake applied
      ("GRA_Status", "Motor_2", 0),                 # ACC engagement status
      ("Soll_Geschwindigkeit_bei_GRA_Be", "Motor_2", 0),  # ACC speed setpoint from ECU??? check this
      ("GRA_Hauptschalt", "GRA_Neu", 0),              # ACC button, on/off
      ("GRA_Abbrechen", "GRA_Neu", 0),                  # ACC button, cancel
      ("GRA_Neu_Setzen", "GRA_Neu", 0),                     # ACC button, set
      ("GRA_Up_lang", "GRA_Neu", 0),                # ACC button, increase or accel, long press
      ("GRA_Down_lang", "GRA_Neu", 0),              # ACC button, decrease or decel, long press
      ("GRA_Up_kurz", "GRA_Neu", 0),                # ACC button, increase or accel, short press
      ("GRA_Down_kurz", "GRA_Neu", 0),              # ACC button, decrease or decel, short press
      ("GRA_Recall", "GRA_Neu", 0),                 # ACC button, resume
      ("GRA_Zeitluecke", "GRA_Neu", 0),             # ACC button, time gap adj
      ("GRA_Neu_Zaehler", "GRA_Neu", 0),            # ACC button, time gap adj
      ("GRA_Sender", "GRA_Neu", 0),                 # GRA Sender Coding
      ("BR8_Sta_ADR_BR", "Bremse_8", 0),           # ABS Pump actively braking for ACC
    ]

    checks = [
      # sig_address, frequency
      ("Bremse_3", 100),          # From J104 ABS/ESP controller
      ("Lenkhilfe_3", 100),       # From J500 Steering Assist with integrated sensors
      ("Lenkwinkel_1", 100),      # From J500 Steering Assist with integrated sensors
      ("Motor_3", 100),           # From J623 Engine control module
      ("Airbag_1", 50),           # From J234 Airbag control module
      ("Bremse_5", 50),           # From J104 ABS/ESP controller
      ("Bremse_8", 50),           # From J??? ABS/ACC controller
      ("GRA_Neu", 50),            # From J??? steering wheel control buttons
      ("Kombi_1", 50),            # From J285 Instrument cluster
      ("Motor_2", 50),            # From J623 Engine control module
      ("Lenkhilfe_2", 20),        # From J500 Steering Assist with integrated sensors
      ("Gate_Komf_1", 10),     # From J533 CAN gateway
      ("Einheiten_1", 1),         # From J??? cluster or gateway
    ]

    if CP.transmissionType == TRANS.automatic:
      signals += [("Waehlhebelposition__Getriebe_1_", "Getriebe_1", 0)]  # Auto trans gear selector position
      checks += [("Getriebe_1", 100)]  # From J743 Auto transmission control module
    elif CP.transmissionType == TRANS.manual:
      signals += [("Kupplungsschalter", "Motor_1", 0),  # Clutch switch
                  ("GK1_Rueckfahr", "Gate_Komf_1", 0)]  # Reverse light from BCM
      checks += [("Motor_1", 100)]  # From J623 Engine control module

    if CP.networkLocation == NWL.fwdCamera:
      # The ACC radar is here on CANBUS.pt
      signals += [("ACA_V_Wunsch", "ACC_GRA_Anziege", 0)]  # ACC set speed
      checks += [("ACC_GRA_Anziege", 25)]  # From J428 ACC radar control module

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, CANBUS.pt)

  @staticmethod
  def get_cam_can_parser(CP):
    # TODO: Need to monitor LKAS camera, if present, for TLC/DLC/warning signals for passthru to SWA
    signals = []
    checks = []

    if CP.networkLocation == NWL.gateway:
      # The ACC radar is here on CANBUS.cam
      signals += [("ACA_V_Wunsch", "ACC_GRA_Anziege", 0)]  # ACC set speed
      checks += [("ACC_GRA_Anziege", 25)]  # From J428 ACC radar control module

    if CP.enableGasInterceptor:
      signals += [("INTERCEPTOR_GAS", "GAS_SENSOR", 0), ("INTERCEPTOR_GAS2", "GAS_SENSOR", 0)]
      checks += [("GAS_SENSOR", 50)]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, CANBUS.cam)