# CAN controls for MQB platform Volkswagen, Audi, Skoda and SEAT.
# PQ35/PQ46/NMS, and any future MLB, to come later.

# ----------------------------------------------------------------------- #
#                                                                         #
# CAN message packing for PQ35/PQ46/NMS vehicles                          #
#                                                                         #
# ----------------------------------------------------------------------- #

def create_pq_steering_control(packer, bus, apply_steer, idx, lkas_enabled):
  values = {
    "HCA_Zaehler": idx,
    "LM_Offset": abs(apply_steer),
    "LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_Status": 7 if (lkas_enabled and apply_steer != 0) else 3,
    "Vib_Freq": 16,
  }

  dat = packer.make_can_msg("HCA_1", bus, values)[2]
  values["HCA_Checksumme"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4]
  return packer.make_can_msg("HCA_1", bus, values)

def create_pq_braking_control(packer, bus, apply_brake, idx, brake_enabled, brake_pre_enable, stopping_wish):
  values = {
    "PQ_MOB_COUNTER": idx,
    "MOB_Bremsmom": abs(apply_brake),
    "MOB_Bremsstgr": abs(apply_brake),
    "MOB_Standby": 1 if (brake_enabled) else 0,
    "MOB_Freigabe": 1 if (brake_enabled and brake_pre_enable) else 0,
    "MOB_Anhaltewunsch": 1 if stopping_wish else 0,
  }

  dat = packer.make_can_msg("MOB_1", bus, values)[2]
  values["PQ_MOB_CHECKSUM"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5]
  return packer.make_can_msg("MOB_1", bus, values)

def create_pq_awv_control(packer, bus, idx, led_orange, led_green, abs_working):
  values = {
    "AWV_2_Fehler" : 1 if led_orange else 0,
    "AWV_2_Status" : 1 if led_green else 0,
    "AWV_Zaehler": idx,
    "AWV_Text": abs_working,
    "AWV_Infoton": 1 if (abs_working == 5) else 0,
  }

  dat = packer.make_can_msg("mAWV", bus, values)[2]
  values["AWV_Checksumme"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4]
  return packer.make_can_msg("mAWV", bus, values)

def create_pq_pedal_control(packer, bus, apply_gas, idx):
  # Common gas pedal msg generator
  enable = apply_gas > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    apply_gas = apply_gas * 1125.
    if (apply_gas < 227):
      apply_gas = 227
    values["GAS_COMMAND"] = apply_gas
    values["GAS_COMMAND2"] = apply_gas

  dat = packer.make_can_msg("GAS_COMMAND", bus, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", bus, values)

def create_pq_hud_control(packer, bus, hca_enabled, steering_pressed, hud_alert, leftLaneVisible, rightLaneVisible,
                           ldw_lane_warning_left, ldw_lane_warning_right, ldw_side_dlc_tlc, ldw_dlc, ldw_tlc):
  if hca_enabled:
    leftlanehud = 3 if leftLaneVisible else 1
    rightlanehud = 3 if rightLaneVisible else 1
  else:
    leftlanehud = 2 if leftLaneVisible else 1
    rightlanehud = 2 if rightLaneVisible else 1

  values = {
    "Right_Lane_Status": rightlanehud,
    "Left_Lane_Status": leftlanehud,
    "SET_ME_X1": 1,
    "Kombi_Lamp_Orange": 1 if hca_enabled and steering_pressed else 0,
    "Kombi_Lamp_Green": 1 if hca_enabled and not steering_pressed else 0,
  }
  return packer.make_can_msg("LDW_1", bus, values)

def create_pq_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Neu_Zaehler": idx,
    "GRA_Sender": CS.graSenderCoding,
    "GRA_Abbrechen": 1 if (buttonStatesToSend["cancel"] or CS.buttonStates["cancel"]) else 0,
    "GRA_Hauptschalt": CS.graHauptschalter,
  }

  dat = packer.make_can_msg("GRA_Neu", bus, values)[2]
  values["GRA_Checksum"] = dat[1] ^ dat[2] ^ dat[3]
  return packer.make_can_msg("GRA_Neu", bus, values)