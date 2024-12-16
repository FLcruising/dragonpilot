def create_steering_control(packer, bus, apply_steer, lkas_enabled):
  values = {
    "LM_Offset": abs(apply_steer),
    "LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_Status": 7 if (lkas_enabled and apply_steer != 0) else 3,
    "Vib_Freq": 16,
  }

  return packer.make_can_msg("HCA_1", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control):
  values = ldw_stock_values.copy()

  values.update({
    "LDW_Lampe_gelb": 1 if enabled and steering_pressed else 0,
    "LDW_Lampe_gruen": 1 if enabled and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible,
    "LDW_Lernmodus_rechts": 3 if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible,
    "LDW_Textbits": hud_alert,
  })

  return packer.make_can_msg("LDW_Status", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, counter, cancel=False, resume=False):
  values = gra_stock_values.copy()

  values.update({
    "COUNTER": counter,
    "GRA_Abbrechen": cancel,
    "GRA_Recall": resume,
  })

  return packer.make_can_msg("GRA_Neu", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  if long_active:
    acc_control = 1
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    hud_status = 6
  elif long_active:
    hud_status = 3
  elif main_switch_on:
    hud_status = 2
  else:
    hud_status = 0

  return hud_status


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  commands = []

  values = {
    "ACS_Sta_ADR": acc_control,
    "ACS_StSt_Info": acc_enabled,
    "ACS_Typ_ACC": acc_type,
    "ACS_Anhaltewunsch": acc_type == 1 and stopping,
    "ACS_FreigSollB": acc_enabled,
    "ACS_Sollbeschl": accel if acc_enabled else 3.01,
    "ACS_zul_Regelabw": 0.2 if acc_enabled else 1.27,
    "ACS_max_AendGrad": 2.0 if acc_enabled else 5.08,
  }

  commands.append(packer.make_can_msg("ACC_System", bus, values))

  return commands


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance):
  values = {
    "ACA_StaACC": acc_hud_status,
    "ACA_Zeitluecke": 2,
    "ACA_V_Wunsch": set_speed if set_speed < 250 else 255,
    "ACA_gemZeitl": lead_distance,
    "ACA_PrioDisp": 3,
    # TODO: ACA_ID_StaACC, ACA_AnzDisplay, ACA_kmh_mph, ACA_Aend_Zeitluecke
    # TODO: ACA_kmh_mph handling probably needed to resolve rounding errors in displayed setpoint
  }

  return packer.make_can_msg("ACC_GRA_Anzeige", bus, values)


    # *** Below here is for OEM+ behavior modification of OEM ACC *** #
    # Modify Motor_2, Bremse_8, Bremse_11
    # When EP1_Freigabe_Ver == 1 (OEM behavior observed)
    #  Bremse_8 : BR8_Sta_ACC_Anf   1 --> 0
    #  Bremse_8 : BR8_Verz_EPB_akt  0 --> 1
    #  Bremse_8 : BR8_StaBrSyst     1 --> 0
    #  Motor_2  : GRA_Status        1 --> 0

def filter_motor2(packer, bus, motor2_car, active):  # bus 0 --> 2
  values = motor2_car
  if active:
    values.update({
      "GRA_Status": 1,
    })
  return packer.make_can_msg("Motor_2", bus, values)

def filter_bremse8(packer, bus, bremse8_car, active):  # bus 0 --> 2
  values = bremse8_car
  if active:
    values.update({
      "BR8_Sta_ACC_Anf": 1,
      "BR8_Verz_EPB_akt": 0,
      "BR8_StaBrSyst": 1,
    })
  return packer.make_can_msg("Bremse_8", bus, values)

def filter_bremse11(packer, bus, bremse11_car, stopped):  # bus 0 --> 2
  values = bremse11_car
  values.update({
    "B11_HydHalten": 1 if stopped else 0,
  })
  return packer.make_can_msg("Bremse_11", bus, values)

def filter_epb1(packer, bus, stopped):  # bus 0 --> 2
  values = {
    "EP1_Verzoegerung": 0,
    "EP1_Freigabe_Ver": 0,
    "EP1_Bremslicht": 0,
    "EP1_HydrHalten": 1 if stopped else 0,
    "EP1_AutoHold_aktiv": 1,
  }
  return packer.make_can_msg("EPB_1", bus, values)

def filter_ACC_System(packer, bus, acc_car, epb_freigabe):  # bus 2 --> 0
  values = acc_car
  if epb_freigabe:
    values.update({
      "ACS_Sta_ADR": 0,
      "ACS_StSt_Info": 0,
      "ACS_FreigSollB": 0,
      "ACS_Sollbeschl": 3.01,
    })
  return packer.make_can_msg("ACC_System", bus, values)

def filter_ACC_Anzeige(packer, bus, anz_car, blind):  # bus 2 --> 0
  values = anz_car
  if blind:
    values.update({
      "ACA_Fahrerhinw": 0,
      "ACA_Akustik2": 0,
    })
  return packer.make_can_msg("ACC_GRA_Anzeige", bus, values)

def filter_GRA_Neu(packer, bus, gra_car, resume):  # bus 2 --> 0
  values = gra_car
  if resume:
    values.update({
      "GRA_Recall": 1,
    })
  return packer.make_can_msg("GRA_Neu", bus, values)

def create_epb_control(packer, bus, apply_brake, epb_enabled):  # bus 1
  values = {
    "EP1_Fehler_Sta": 0,
    "EP1_Sta_EPB": 0,
    "EP1_Spannkraft": 0,
    "EP1_Schalterinfo": 0,
    "EP1_Fkt_Lampe": 0,
    "EP1_Verzoegerung": apply_brake,                        #Brake request in m/s2
    "EP1_Freigabe_Ver": 1 if epb_enabled else 0,            #Allow braking pressure to build.
    "EP1_Bremslicht": 1 if apply_brake != 0 else 0,         #Enable brake lights
    "EP1_HydrHalten": 1 if epb_enabled else 0,
    "EP1_AutoHold_aktiv": 1,                                #Signal indicating EPB is available
  }
  return packer.make_can_msg("EPB_1", bus, values)