from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_driver_steer_torque_limits
from openpilot.selfdrive.car.volkswagen import mqbcan, pqcan
from openpilot.selfdrive.car.volkswagen.values import CANBUS, PQ_CARS, CarControllerParams, VolkswagenFlags

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

def limit_jerk(accel, prev_accel, max_jerk, dt):
  max_delta_accel = max_jerk * dt
  delta_accel = max(-max_delta_accel, min(accel - prev_accel, max_delta_accel))
  return prev_accel + delta_accel

def EPB_handler(CS, self, ACS_Sta_ADR, ACS_Sollbeschl, vEgo, stopping):
  if ACS_Sta_ADR == 1 and ACS_Sollbeschl < 0 and vEgo <= (18 * CV.KPH_TO_MS):
      if not self.EPB_enable:  # First frame of EPB entry
          self.EPB_counter = 0
          self.EPB_brake = 0
          self.EPB_enable = 1
          self.EPB_brake_last = ACS_Sollbeschl
      else:
          self.EPB_brake = limit_jerk(-4, self.EPB_brake_last, 0.7, 0.02) if stopping else ACS_Sollbeschl
          self.EPB_brake_last = self.EPB_brake
      self.EPB_counter += 1
  else:
      if self.EPB_enable and self.EPB_counter < 10:  # Keep EPB_enable active for 10 frames
          self.EPB_counter += 1
      else:
          self.EPB_brake = 0
          self.EPB_enable = 0

  if CS.out.gasPressed or CS.out.brakePressed:
    if self.EPB_enable:
      self.ACC_anz_blind = 1
    self.EPB_brake = 0
    self.EPB_enable = 0
    self.EPB_enable_prev = 0
    self.EPB_enable_2old = 0

  if self.ACC_anz_blind and self.ACC_anz_blind_counter < 150:
    self.ACC_anz_blind_counter += 1
  else:
    self.ACC_anz_blind = 0
    self.ACC_anz_blind_counter = 0

  # Update EPB historical states and calculate EPB_active
  self.EPB_active = int((self.EPB_enable_2old and not self.EPB_enable) or self.EPB_enable)
  self.EPB_enable_2old = self.EPB_enable_prev
  self.EPB_enable_prev = self.EPB_enable

  return self.EPB_enable, self.EPB_brake, self.EPB_active


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.CCS = pqcan if CP.carFingerprint in PQ_CARS else mqbcan
    self.packer_pt = CANPacker(dbc_name)

    self.apply_steer_last = 0
    self.gra_acc_counter_last = None
    self.bremse8_counter_last = None
    self.bremse11_counter_last = None
    self.acc_sys_counter_last = None
    self.acc_anz_counter_last = None
    self.ACC_anz_blind = 0
    self.ACC_anz_blind_counter = 0
    self.frame = 0
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.accel_last = 0
    self.frame = 0
    self.EPB_brake = 0
    self.EPB_brake_last = 0
    self.EPB_enable = 0
    self.EPB_enable_prev = 0
    self.EPB_enable_2old = 0
    self.EPB_active = 0
    self.EPB_counter = 0
    self.accel_diff = 0
    self.long_deviation = 0
    self.long_jerklimit = 0
    self.stopped = 0
    self.stopping = 0

  def update(self, CC, CS, ext_bus, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      # Logic to avoid HCA state 4 "refused":
      #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
      #   * Don't steer at standstill
      #   * Don't send > 3.00 Newton-meters torque
      #   * Don't send the same torque for > 6 seconds
      #   * Don't send uninterrupted steering for > 360 seconds
      # MQB racks reset the uninterrupted steering timer after a single frame
      # of HCA disabled; this is done whenever output happens to be zero.

      if CC.latActive:
        new_steer = int(round(actuators.steer * self.CCP.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)
        self.hca_frame_timer_running += self.CCP.STEER_STEP
        if self.apply_steer_last == apply_steer:
          self.hca_frame_same_torque += self.CCP.STEER_STEP
          if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
            apply_steer -= (1, -1)[apply_steer < 0]
            self.hca_frame_same_torque = 0
        else:
          self.hca_frame_same_torque = 0
        hca_enabled = abs(apply_steer) > 0
      else:
        hca_enabled = False
        apply_steer = 0

      if not hca_enabled:
        self.hca_frame_timer_running = 0

      self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
      self.apply_steer_last = apply_steer
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = clip(apply_steer * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX)
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, CANBUS.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0
      stopping = actuators.longControlState == LongCtrlState.stopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
      can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.longActive, accel,
                                                         acc_control, stopping, starting, CS.esp_hold_confirmation))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.enabled,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, set_speed,
                                                       lead_distance))

    # **** Stock ACC Button Controls **************************************** #

    if self.CP.openpilotLongitudinalControl:
      gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
      if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
        can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, ext_bus, CS.gra_stock_values,
                                                            cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))
    
    # *** Below here is for OEM+ behavior modification of OEM ACC *** #
    # Modify Motor_2, Bremse_8, Bremse_11
    if self.CCS == pqcan and not self.CP.openpilotLongitudinalControl:
      self.stopping = CS.acc_sys_stock["ACS_Anhaltewunsch"] and (CS.out.vEgoRaw <= 1 or self.stopping)
      self.stopped = self.EPB_enable and (CS.out.vEgoRaw == 0 or (self.stopping and self.stopped))

      if CS.acc_sys_stock["COUNTER"] != self.acc_sys_counter_last:
        EPB_handler(CS, self, CS.acc_sys_stock["ACS_Sta_ADR"], CS.acc_sys_stock["ACS_Sollbeschl"], CS.out.vEgoRaw, self.stopping)
        can_sends.append(self.CCS.filter_ACC_System(self.packer_pt, CANBUS.pt, CS.acc_sys_stock, self.EPB_active))
        can_sends.append(self.CCS.create_epb_control(self.packer_pt, CANBUS.br, self.EPB_brake, self.EPB_enable))
        can_sends.append(self.CCS.filter_epb1(self.packer_pt, CANBUS.cam, self.stopped))  # in custom module, filter the gateway fwd EPB msg
      if CS.acc_anz_stock["COUNTER"] != self.acc_anz_counter_last:
        can_sends.append(self.CCS.filter_ACC_Anzeige(self.packer_pt, CANBUS.pt, CS.acc_anz_stock, self.ACC_anz_blind))
      if self.frame % 2 or CS.motor2_stock != getattr(self, 'motor2_last', CS.motor2_stock):  # 50hz / 20ms
        can_sends.append(self.CCS.filter_motor2(self.packer_pt, CANBUS.cam, CS.motor2_stock, self.EPB_active))
      if CS.bremse8_stock["COUNTER"] != self.bremse8_counter_last:
        can_sends.append(self.CCS.filter_bremse8(self.packer_pt, CANBUS.cam, CS.bremse8_stock, self.EPB_active))
      if CS.bremse11_stock["COUNTER"] != self.bremse11_counter_last:
        can_sends.append(self.CCS.filter_bremse11(self.packer_pt, CANBUS.cam, CS.bremse11_stock, self.stopped))
      if CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last:
        can_sends.append(self.CCS.filter_GRA_Neu(self.packer_pt, CANBUS.cam, CS.gra_stock_values, resume = self.stopped and (self.frame % 100 < 50)))

      self.motor2_last = CS.motor2_stock
      self.acc_sys_counter_last = CS.acc_sys_stock["COUNTER"]
      self.acc_anz_counter_last = CS.acc_anz_stock["COUNTER"]
      self.bremse8_counter_last = CS.bremse8_stock["COUNTER"]
      self.bremse11_counter_last = CS.bremse11_stock["COUNTER"]

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends, self.eps_timer_soft_disable_alert
