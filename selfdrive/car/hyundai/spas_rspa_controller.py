# This is the work of JPR
from cereal import car
from common.params import Params
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from common.realtime import DT_CTRL
from selfdrive.car.hyundai.values import CAR, CHECKSUM, FEATURES, EV_HYBRID_CAR
import crcmod
hyundai_checksum = crcmod.mkCrcFun(0x11D, initCrc=0xFD, rev=False, xorOut=0xdf)

###### SPAS ###### - JPR
STEER_ANG_MAX = 450 # SPAS Max Angle
ANGLE_DELTA_BP = [0., 10., 20.]
ANGLE_DELTA_V = [1.19, 1.14, 1.09]    # windup limit
ANGLE_DELTA_VU = [1.29, 1.19, 1.14]   # unwind limit
TQ = 290 # = TQ / 100 = NM is unit of measure for wheel.
SPAS_SWITCH = 30 * CV.MPH_TO_MS # MPH - lowered Bc of model and overlearn steerRatio
STEER_MAX_OFFSET = 105 # How far from MAX LKAS torque to engage Dynamic SPAS when under 60mph.
###### SPAS #######

EventName = car.CarEvent.EventName

class SpasRspaController:
  def __init__(self):
    self.last_apply_angle = 0.0
    self.en_spas = 2
    self.mdps11_stat_last = 0
    self.lkas_active = False
    self.spas_active = False
    self.dynamicSpas = Params().get_bool('DynamicSpas')
    self.ratelimit = 2.3 # Starting point - JPR
    self.rate = 0
    self.lastSteeringAngleDeg = 0
    self.cut_timer = 0
    self.SteeringTempUnavailable = False
    self.ens_rspa = 0
  
  @staticmethod
  def create_rspa11(packer, car_fingerprint, frame, en_rspa, bus, enabled, accel, stopping, gaspressed):
    values = {
      "CF_RSPA_State": en_rspa, # Assuming like SPAS state logic somehow...
      "CF_RSPA_Act": 0, # Maybe which gear to be in?
      "CF_RSPA_DecCmd": 0, # Wanna apply brakes? clip(accel, -2, 0)?
      "CF_RSPA_Trgt_Spd": 0, # Probably not needed bc either speed spoofed or using ACC. Depends on how testing goes...
      "CF_RSPA_StopReq": 1 if enabled and stopping and not gaspressed else 0,
      "CR_RSPA_EPB_Req": 0, # Electronic Parking Brake
      "CF_RSPA_ACC_ACT": 0, # Accel to target speed?
      "CF_RSPA_AliveCounter": frame % 0x200, #Probably same or similar Alive Counter too SPAS
      "CF_RSPA_CRC": 0,
    }
    # Handle RSPA CRC
    dat = packer.make_can_msg("RSPA11", 0, values)[2]
    if car_fingerprint in CHECKSUM["crc8"]:
      dat = dat[:6]
      values["CF_RSPA_CRC"] = hyundai_checksum(dat)
    else:
      values["CF_RSPA_CRC"] = sum(dat[:6]) % 256
    return packer.make_can_msg("RSPA11", bus, values)

  def create_spas11(packer, car_fingerprint, frame, en_spas, apply_steer, bus):
    values = {
      "CF_Spas_Stat": en_spas,
      "CF_Spas_TestMode": 0,
      "CR_Spas_StrAngCmd": apply_steer,
      "CF_Spas_BeepAlarm": 0,
      "CF_Spas_Mode_Seq": 2,
      "CF_Spas_AliveCnt": frame % 0x200,
      "CF_Spas_Chksum": 0,
      "CF_Spas_PasVol": 0,
    }
    dat = packer.make_can_msg("SPAS11", 0, values)[2]
    if car_fingerprint in CHECKSUM["crc8"]:
      dat = dat[:6]
      values["CF_Spas_Chksum"] = hyundai_checksum(dat)
    else:
      values["CF_Spas_Chksum"] = sum(dat[:6]) % 256
    return packer.make_can_msg("SPAS11", bus, values)

  def create_spas12(bus):
    return [1268, 0, b"\x00\x00\x00\x00\x00\x00\x00\x00", bus]
  
  def create_ems_366(packer, ems_366, enabled):
    values = ems_366
    if enabled:
      values["VS"] = 1
    return packer.make_can_msg("EMS_366", 1, values)

  def create_ems11(packer, ems11, enabled):
    values = ems11
    if enabled:
      values["VS"] = 1
    return packer.make_can_msg("EMS11", 1, values)

  def create_eems11(packer, eems11, enabled):
    values = eems11
    if enabled:
      values["Accel_Pedal_Pos"] = 1
      values["CR_Vcu_AccPedDep_Pos"] = 1
    return packer.make_can_msg("E_EMS11", 1, values)

  def create_clu11(packer, clu11, enabled):
    values = clu11
    if enabled:
      values["CF_Clu_Vanz"] = 1
    return packer.make_can_msg("CLU11", 1, values)

  def inject_events(self, events):
    if self.SteeringTempUnavailable:
        events.add(EventName.steerTempUnavailable)

  def RSPA_Controller(self, c, CS, frame, packer, car_fingerprint, can_sends, accel, stopping):
    if CS.rspa_enabled:
      if (frame % 2) == 0: # Not sure rough guess for now... Will know when see cabana. - JPR
        can_sends.append(SpasRspaController.create_rspa11(packer, car_fingerprint, frame, self.en_rspa, CS.mdps_bus, c.active, accel, stopping, CS.out.gasPressed))

  def SPAS_Controller(self, c, CS, actuators, frame, maxTQ, packer, car_fingerprint, emsType, apply_steer, turnsignalcut, can_sends):
    self.packer = packer
    self.car_fingerprint = car_fingerprint
    if CS.spas_enabled:
      # Keep Track of SPAS State, Steering wheel rate, and other metrics. - JPR
      self.rate = abs(CS.out.steeringAngleDeg - self.lastSteeringAngleDeg)
      apply_angle = clip(actuators.steeringAngleDeg, -1*(STEER_ANG_MAX), STEER_ANG_MAX)
      apply_diff = abs(apply_angle - CS.out.steeringAngleDeg)
      spas_active = c.active and CS.out.vEgo < 26.82 and (CS.out.vEgo < SPAS_SWITCH or apply_diff > 3.2 and self.dynamicSpas and not CS.out.steeringPressed or abs(apply_angle) > 3. and self.spas_active or maxTQ - STEER_MAX_OFFSET < apply_steer and self.dynamicSpas)      
      if (frame % 2) == 0: # Run this at same speed as the SPAS11 message BC thats how fast the steering updates. - JPR
        if spas_active and apply_diff > 1.75: # Rate limit for when steering angle is not apply_angle or "engage" rate. - JPR
          self.ratelimit += 0.03 # Increase each cycle - JPR
          rate_limit = max(self.ratelimit, 10) # Make sure not to go past +-10 on rate - JPR
          #print("apply_diff is greater than 1.5 : rate limit :", rate_limit)
          apply_angle = clip(apply_angle, CS.out.steeringAngleDeg - rate_limit, CS.out.steeringAngleDeg + rate_limit)
        elif spas_active: # Normal Operation Rate Limiter. - JPR
          self.ratelimit = 2.3 # Reset it back - JPR
          if self.last_apply_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_apply_angle):
            rate_limit = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_V)
          else:
            rate_limit = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
          apply_angle = clip(apply_angle, self.last_apply_angle - rate_limit, self.last_apply_angle + rate_limit)
        else:
          apply_angle = CS.mdps11_strang

      if (CS.out.steeringPressedSPAS or self.rate > 1.4): # Reset SPAS cut timer if steeringPressedSPAS is True or if the steering wheel is moving fast. - JPR
        self.cut_timer = 0
        
      if CS.out.steeringPressedSPAS or self.cut_timer <= 85:# Keep SPAS cut for 50 cycles after steering pressed to prevent unintentional fighting. - JPR
        spas_active = False
        self.cut_timer += 1
    
      if turnsignalcut:
        spas_active = False

      if CS.out.steeringPressedSPAS or self.cut_timer < 85 and self.rate > 5:# Keep SPAS cut for 50 cycles after steering pressed to prevent unintentional fighting. - JPR
        spas_active = False
        self.cut_timer += 1

      self.last_apply_angle = apply_angle

      ############### SPAS STATES ############## - JPR
      # State 1 : Start
      # State 2 : New Request
      # State 3 : Ready to Assist(Steer)
      # State 4 : Hand Shake between OpenPilot and MDPS ECU
      # State 5 : Assisting (Steering)
      # State 6 : Failed to Assist (Steer)
      # State 7 : Cancel
      # State 8 : Failed to get ready to Assist (Steer)
      # ---------------------------------------------------
      if spas_active and (CS.mdps11_stat == 4 or CS.mdps11_stat == 5 or CS.mdps11_stat == 3): # Spoof Speed on mdps11_stat 3, 4 and 5. - JPR
        spas_active_stat = True
      else:
        spas_active_stat = False
          
      if emsType == 1:
        can_sends.append(SpasRspaController.create_ems_366(self.packer, CS.ems_366, spas_active_stat))
        if Params().get_bool('SPASDebug'):
          print("EMS_366")
      elif emsType == 2:
        can_sends.append(SpasRspaController.create_ems11(self.packer, CS.ems11, spas_active_stat))
        if Params().get_bool('SPASDebug'):
          print("EMS_11")
      elif emsType == 3:
        can_sends.append(SpasRspaController.create_eems11(self.packer, CS.eems11, spas_active_stat))
        if Params().get_bool('SPASDebug'):
          print("E_EMS11")
      elif emsType == 4:
        can_sends.append(SpasRspaController.create_clu11(self.packer, CS.clu11, spas_active_stat))
        if Params().get_bool('SPASDebug'):
          print("CLU11")
      elif emsType == 0:
        print("Please add a car parameter called ret.emsType = (your EMS type) in interface.py : EMS_366 = 1 : EMS_11 = 2 : E_EMS11 = 3")

      if (frame % 2) == 0:
        ####### !!!! DO NOT MODIFY SPAS STATE MACHINE - JPR !!!! #######
        if CS.mdps11_stat == 7 and not self.mdps11_stat_last == 7:
          self.en_spas = 7 # Acknowledge that MDPS is in State 7 - JPR

        if CS.mdps11_stat == 7 and self.mdps11_stat_last == 7:
          self.en_spas = 3 # Tell MDPS to get ready for next steer. - JPR

        if CS.mdps11_stat == 2 and spas_active:
          self.en_spas = 3 # Switch to State 3, and get Ready to Assist(Steer). - JPR

        if CS.mdps11_stat == 3 and spas_active:
          self.en_spas = 4 # Handshake Between MDPS and OpenPilot. - JPR

        if CS.mdps11_stat == 4: 
          self.en_spas = 5 # Ask MDPS to Steer using Angle. - JPR

        if CS.mdps11_stat == 5 and not spas_active:
          self.en_spas = 7 # Disengage/Cancel SPAS - JPR

        if CS.mdps11_stat == 6:
          self.en_spas = 2 # Failed to Assist and Steer, Set state back to 2 for a new request. - JPR

        if CS.mdps11_stat == 8:
          self.en_spas = 2 #MDPS ECU Fails to get into state 3 and ready for state 5. - JPR
      
        if CS.mdps11_stat == 6 or CS.mdps11_stat == 8: # Monitor MDPS SPAS error states and send them to inject_events. - JPR
          self.SteeringTempUnavailable = True
        else:
          self.SteeringTempUnavailable = False

        if not spas_active:
          apply_angle = CS.mdps11_strang

        can_sends.append(SpasRspaController.create_spas11(self.packer, self.car_fingerprint, (frame // 2), self.en_spas, apply_angle, CS.mdps_bus))

      if Params().get_bool('SPASDebug'): # SPAS debugging - JPR
        print("MDPS SPAS State: ", CS.mdps11_stat) # SPAS STATE DEBUG
        print("OP SPAS State: ", self.en_spas) # OpenPilot Ask MDPS to switch to state.
        print("spas_active:", spas_active)
        print("apply angle:", apply_angle)
        print("driver torque:", CS.out.steeringWheelTorque)

      # SPAS12 20Hz
      if (frame % 5) == 0:
        can_sends.append(SpasRspaController.create_spas12(CS.mdps_bus))

      self.mdps11_stat_last = CS.mdps11_stat
      self.spas_active = spas_active
      self.lastSteeringAngleDeg = CS.out.steeringAngleDeg
