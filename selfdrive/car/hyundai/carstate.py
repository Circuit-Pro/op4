from cereal import car
from selfdrive.car.hyundai.values import DBC, STEER_THRESHOLD, FEATURES, CAR, HYBRID_CAR, EV_HYBRID_CAR
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.conversions import Conversions as CV
from common.params import Params
from common.numpy_fast import interp, clip

GearShifter = car.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    else:  # preferred and elect gear methods use same definition
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    #Auto detection for setup
    self.no_radar = CP.sccBus == -1
    self.mdps_bus = CP.mdpsBus
    self.sas_bus = CP.sasBus
    self.scc_bus = CP.sccBus
    self.has_scc13 = CP.hasScc13 or CP.carFingerprint in FEATURES["has_scc13"]
    self.has_scc14 = CP.hasScc14 or CP.carFingerprint in FEATURES["has_scc14"]
    self.has_lfa_hda = CP.hasLfaHda
    self.leftBlinker = False
    self.rightBlinker = False
    self.lkas_button_on = True
    self.cruise_main_button = 0
    self.mdps_error_cnt = 0
    self.cruise_unavail_cnt = 0

    self.apply_steer = 0.
    self.steeringTorqueEps = 0

    # scc smoother
    self.acc_mode = False
    self.cruise_gap = 1
    self.brake_pressed = False
    self.gas_pressed = False
    self.standstill = False
    self.cruiseState_enabled = False
    self.cruiseState_speed = 0

    self.use_cluster_speed = Params().get_bool('UseClusterSpeed')
    self.long_control_enabled = Params().get_bool('LongControlEnabled')

    self.spas_enabled = CP.spasEnabled
    self.rspa_enabled = CP.rspaEnabled
    self.mdps11_stat = 0

    # Wheel Momentum/Rate Factor. - JPR
    self.angle_delta_bp = [0., 5., 10., 20., 30, 40, 50, 60, 70] # How Fast is SAS11 is reporting the rate in deg. - JPR
    self.angle_delta_v = [1., 1.1, 1.15, 1.2, 1.25, 1.3, 1.35, 1.4, 1.45]  # How much the rate factor should be. - JPR

  def update(self, cp, cp2, cp_cam):
    cp_mdps = cp2 if self.mdps_bus else cp
    cp_sas = cp2 if self.sas_bus else cp
    cp_scc = cp2 if self.scc_bus == 1 else cp_cam if self.scc_bus == 2 else cp

    self.prev_cruise_buttons = self.cruise_buttons
    self.prev_cruise_main_button = self.cruise_main_button
    self.prev_left_blinker = self.leftBlinker
    self.prev_right_blinker = self.rightBlinker
    self.prev_lkas_button = self.lkas_button_on
    ret = car.CarState.new_message()

    if self.spas_enabled:
      RATE_FACTOR = clip(interp(abs(cp_sas.vl["SAS11"]['SAS_Speed']), self.angle_delta_bp, self.angle_delta_v), 1.0, 1.45) # Don't Let angle factor get above 1.45! - JPR

    ret.doorOpen = any([cp.vl["CGW1"]["CF_Gway_DrvDrSw"], cp.vl["CGW1"]["CF_Gway_AstDrSw"],
                        cp.vl["CGW2"]["CF_Gway_RLDrSw"], cp.vl["CGW2"]["CF_Gway_RRDrSw"]])

    ret.seatbeltUnlatched = cp.vl["CGW1"]["CF_Gway_DrvSeatBeltSw"] == 0

    self.speed_conv_to_ms = CV.MPH_TO_MS if cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"] == 1 else CV.KPH_TO_MS

    cluSpeed = cp.vl["CLU11"]["CF_Clu_Vanz"]
    decimal = cp.vl["CLU11"]["CF_Clu_VanzDecimal"]
    if 0. < decimal < 0.5:
      cluSpeed += decimal

    ret.cluSpeedMs = cluSpeed * self.speed_conv_to_ms

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHL_SPD11"]["WHL_SPD_FL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_FR"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RR"],
    )

    vEgoRawWheel = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    vEgoWheel, aEgoWheel = self.update_speed_kf(vEgoRawWheel)

    vEgoRawClu = cluSpeed * self.speed_conv_to_ms
    vEgoClu, aEgoClu = self.update_clu_speed_kf(vEgoRawClu)

    if self.use_cluster_speed:
      ret.vEgoRaw = vEgoRawClu
      ret.vEgo = vEgoClu
      ret.aEgo = aEgoClu
    else:
      ret.vEgoRaw = vEgoRawWheel
      ret.vEgo = vEgoWheel
      ret.aEgo = aEgoWheel

    ret.vCluRatio = (vEgoWheel / vEgoClu) if (vEgoClu > 3. and vEgoWheel > 3.) else 1.0

    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = cp_sas.vl["SAS11"]['SAS_Angle']
    ret.steeringRateDeg = cp_sas.vl["SAS11"]['SAS_Speed']
    
    ret.yawRate = cp.vl["ESP12"]['YAW_RATE']
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["CGW1"]['CF_Gway_TurnSigLh'],
                                                            cp.vl["CGW1"]['CF_Gway_TurnSigRh'])
    ret.steeringTorque = cp_mdps.vl["MDPS12"]['CR_Mdps_StrColTq']
    ret.steeringTorqueEps = cp_mdps.vl["MDPS12"]['CR_Mdps_OutTq'] / 10
    ret.steeringWheelTorque = cp_mdps.vl["MDPS11"]['CR_Mdps_DrvTq'] 

    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steeringPressedSPAS = abs(ret.steeringTorque) > STEER_THRESHOLD + (220 * RATE_FACTOR) if self.mdps11_stat == 5 else abs(ret.steeringTorque) > STEER_THRESHOLD
    if Params().get_bool('SPASDebug'):
      print("Rate Factor  : ", RATE_FACTOR)

    if not ret.standstill and cp_mdps.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0:
      self.mdps_error_cnt += 1
    else:
      self.mdps_error_cnt = 0

    ret.steerFaultTemporary = self.mdps_error_cnt > 50

    if self.CP.enableAutoHold:
      ret.autoHold = cp.vl["ESP11"]["AVH_STAT"]

    # cruise state
    if self.CP.radarDisable:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.available = cp.vl["TCS13"]["ACCEnable"] == 0
      ret.cruiseState.enabled = cp.vl["TCS13"]["ACC_REQ"] != 3
      ret.cruiseState.standstill = False
      ret.cruiseState.enabledAcc = ret.cruiseState.enabled
      print("cruiseState.enabled", cp.vl["TCS13"]["ACC_REQ"])
      print("cruiseState.available", cp.vl["TCS13"]["ACCEnable"])
    else:
      ret.cruiseState.enabled = (cp_scc.vl["SCC12"]["ACCMode"] != 0) if not self.no_radar else \
                                      cp.vl["LVR12"]["CF_Lvr_CruiseSet"] != 0
      ret.cruiseState.available = (cp_scc.vl["SCC11"]["MainMode_ACC"] != 0) if not self.no_radar else \
                                      cp.vl["EMS16"]["CRUISE_LAMP_M"] != 0
      ret.cruiseState.standstill = cp_scc.vl["SCC11"]["SCCInfoDisplay"] == 4. if not self.no_radar else False

      ret.cruiseState.enabledAcc = ret.cruiseState.enabled

      if ret.cruiseState.enabled:
        ret.cruiseState.speed = cp_scc.vl["SCC11"]["VSetDis"] * self.speed_conv_to_ms if not self.no_radar else \
                                         cp.vl["LVR12"]["CF_Lvr_CruiseSet"] * self.speed_conv_to_ms
      else:
        ret.cruiseState.speed = 0
    self.cruise_main_button = cp.vl["CLU11"]["CF_Clu_CruiseSwMain"]
    self.cruise_buttons = cp.vl["CLU11"]["CF_Clu_CruiseSwState"]

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["TCS13"]["DriverBraking"] != 0
    ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2  # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
    ret.parkingBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
    #ret.parkingBrake = cp.vl["CGW1"]["CF_Gway_ParkBrakeSw"]

    # TODO: Check this
    ret.brakeLights = bool(cp.vl["TCS13"]["BrakeLight"] or ret.brakePressed)
    ret.gasPressed = cp.vl["TCS13"]["DriverOverride"] == 1

    if self.CP.carFingerprint in EV_HYBRID_CAR:
      if self.CP.carFingerprint in HYBRID_CAR:
        ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
      else:
        ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.

    if self.CP.hasEms:
      ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])

    #ret.cruiseState.speedLimit = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"] * 0.62137119223733 if Params().get_bool("HyundaiNaviSL") else 0.
    # TODO: refactor gear parsing in function
    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      gear = cp.vl["TCU12"]["CUR_GR"]
    elif self.CP.carFingerprint in FEATURES["use_elect_gears"]:
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    if not self.CP.radarDisable:
      if self.CP.carFingerprint in FEATURES["use_fca"]:
        ret.stockAeb = cp.vl["FCA11"]["FCA_CmdAct"] != 0
        ret.stockFcw = cp.vl["FCA11"]["CF_VSM_Warn"] == 2
      else:
        ret.stockAeb = cp.vl["SCC12"]["AEB_CmdAct"] != 0
        ret.stockFcw = cp.vl["SCC12"]["CF_VSM_Warn"] == 2

    # Blind Spot Detection and Lane Change Assist signals
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0
    else:
      ret.leftBlindspot = False
      ret.rightBlindspot = False

    # save the entire LKAS11, CLU11, SCC12 and MDPS12
    self.lkas11 = cp_cam.vl["LKAS11"]
    self.clu11 = cp.vl["CLU11"]
    self.scc11 = cp_scc.vl["SCC11"]
    self.scc12 = cp_scc.vl["SCC12"]
    self.mdps12 = cp_mdps.vl["MDPS12"]
    self.park_brake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
    self.brake_error = cp.vl["TCS13"]["ACCEnable"] != 0 # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED
    self.lfahda_mfc = cp_cam.vl["LFAHDA_MFC"]
    self.steer_state = cp_mdps.vl["MDPS12"]["CF_Mdps_ToiActive"] #0 NOT ACTIVE, 1 ACTIVE
    self.cruise_unavail_cnt += 1 if cp.vl["TCS13"]["CF_VSM_Avail"] != 1 and cp.vl["TCS13"]["ACCEnable"] != 0 else -self.cruise_unavail_cnt
    self.cruise_unavail = self.cruise_unavail_cnt > 100

    self.lead_distance = cp_scc.vl["SCC11"]["ACC_ObjDist"] if not self.no_radar else 0
    if self.has_scc13:
      self.scc13 = cp_scc.vl["SCC13"]
    if self.has_scc14:
      self.scc14 = cp_scc.vl["SCC14"]
    
    #if self.rspa_enabled: # RSPA - JPR

    if self.spas_enabled: # SPAS - JPR
      self.ems_366 = cp.vl["EMS_366"]
      self.ems11 = cp.vl["EMS11"]
      self.eems11 = cp.vl["E_EMS11"]
      self.mdps11_strang = cp_mdps.vl["MDPS11"]["CR_Mdps_StrAng"]
      self.mdps11_stat_last = self.mdps11_stat
      self.mdps11_stat = cp_mdps.vl["MDPS11"]["CF_Mdps_Stat"]
      ret.mdps11Stat = cp_mdps.vl["MDPS11"]["CF_Mdps_Stat"]
      
    self.lkas_error = cp_cam.vl["LKAS11"]["CF_Lkas_LdwsSysState"] == 7
    if not self.lkas_error and self.car_fingerprint not in [CAR.SONATA,CAR.PALISADE,
                    CAR.SONATA_HEV, CAR.SANTA_FE, CAR.KONA_EV, CAR.NIRO_EV, CAR.KONA]:
      self.lkas_button_on = bool(cp_cam.vl["LKAS11"]["CF_Lkas_LdwsSysState"])


    # scc smoother
    driver_override = cp.vl["TCS13"]["DriverOverride"]
    self.acc_mode = ret.cruiseState.enabled if self.CP.radarDisable else cp_scc.vl["SCC12"]["ACCMode"] != 0
    self.cruise_gap = 3 #cp_scc.vl["SCC11"]["TauGapSet"] #if not self.no_radar else 1
    self.gas_pressed = ret.gasPressed or driver_override == 1
    self.brake_pressed = ret.brakePressed or driver_override == 2
    self.standstill = ret.standstill or ret.cruiseState.standstill
    self.cruiseState_enabled = ret.cruiseState.enabled
    self.cruiseState_speed = ret.cruiseState.speed
    ret.cruiseGap = self.cruise_gap

    tpms_unit = cp.vl["TPMS11"]["UNIT"] * 0.725 if int(cp.vl["TPMS11"]["UNIT"]) > 0 else 1.
    ret.tpms.fl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FL"]
    ret.tpms.fr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FR"]
    ret.tpms.rl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RL"]
    ret.tpms.rr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RR"]

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("WHL_SPD_FL", "WHL_SPD11", 0),
      ("WHL_SPD_FR", "WHL_SPD11", 0),
      ("WHL_SPD_RL", "WHL_SPD11", 0),
      ("WHL_SPD_RR", "WHL_SPD11", 0),

      ("YAW_RATE", "ESP12", 0),

      ("CF_Gway_DrvSeatBeltInd", "CGW4", 1),

      ("CF_Gway_DrvSeatBeltSw", "CGW1", 0),
      ("CF_Gway_DrvDrSw", "CGW1", 0),       # Driver Door
      ("CF_Gway_AstDrSw", "CGW1", 0),       # Passenger door
      ("CF_Gway_RLDrSw", "CGW2", 0),        # Rear reft door
      ("CF_Gway_RRDrSw", "CGW2", 0),        # Rear right door
      ("CF_Gway_TurnSigLh", "CGW1", 0),
      ("CF_Gway_TurnSigRh", "CGW1", 0),
      ("CF_Gway_ParkBrakeSw", "CGW1", 0),   # Parking Brake

      ("CYL_PRES", "ESP12", 0),

      ("CF_Clu_CruiseSwState", "CLU11", 0),
      ("CF_Clu_CruiseSwMain", "CLU11", 0),
      ("CF_Clu_SldMainSW", "CLU11", 0),
      ("CF_Clu_ParityBit1", "CLU11", 0),
      ("CF_Clu_VanzDecimal" , "CLU11", 0),
      ("CF_Clu_Vanz", "CLU11", 0),
      ("CF_Clu_SPEED_UNIT", "CLU11", 0),
      ("CF_Clu_DetentOut", "CLU11", 0),
      ("CF_Clu_RheostatLevel", "CLU11", 0),
      ("CF_Clu_CluInfo", "CLU11", 0),
      ("CF_Clu_AmpInfo", "CLU11", 0),
      ("CF_Clu_AliveCnt1", "CLU11", 0),

      ("ACCEnable", "TCS13", 0),
      ("ACC_REQ", "TCS13", 0),
      ("BrakeLight", "TCS13", 0),
      ("DriverBraking", "TCS13", 0),
      ("DriverOverride", "TCS13", 0), # scc smoother
      ("CF_VSM_Avail", "TCS13", 0),
      ("StandStill", "TCS13", 0),
      ("PBRAKE_ACT", "TCS13", 0),

      ("ESC_Off_Step", "TCS15", 0),
      ("AVH_LAMP", "TCS15", 0),

      #("CF_Lvr_GearInf", "LVR11", 0),        # Transmission Gear (0 = N or P, 1-8 = Fwd, 14 = Rev)

      ("UNIT", "TPMS11", 0),
      ("PRESSURE_FL", "TPMS11", 0),
      ("PRESSURE_FR", "TPMS11", 0),
      ("PRESSURE_RL", "TPMS11", 0),
      ("PRESSURE_RR", "TPMS11", 0),
    ]
    if CP.sccBus == 0 and CP.pcmCruise:
      signals += [
        ("MainMode_ACC", "SCC11", 1),
        ("SCCInfoDisplay", "SCC11", 0),
        ("AliveCounterACC", "SCC11", 0),
        ("VSetDis", "SCC11", 30),
        ("ObjValid", "SCC11", 0),
        ("DriverAlertDisplay", "SCC11", 0),
        ("TauGapSet", "SCC11", 4),
        ("ACC_ObjStatus", "SCC11", 0),
        ("ACC_ObjLatPos", "SCC11", 0),
        ("ACC_ObjDist", "SCC11", 150), #TK211X value is 204.6
        ("ACC_ObjRelSpd", "SCC11", 0),
        ("Navi_SCC_Curve_Status", "SCC11", 0),
        ("Navi_SCC_Curve_Act", "SCC11", 0),
        ("Navi_SCC_Camera_Act", "SCC11", 0),
        ("Navi_SCC_Camera_Status", "SCC11", 2),

        ("ACCMode", "SCC12", 0),
        ("CF_VSM_Prefill", "SCC12", 0),
        ("CF_VSM_DecCmdAct", "SCC12", 0),
        ("CF_VSM_HBACmd", "SCC12", 0),
        ("CF_VSM_Warn", "SCC12", 0),
        ("CF_VSM_Stat", "SCC12", 0),
        ("CF_VSM_BeltCmd", "SCC12", 0),
        ("ACCFailInfo", "SCC12", 0),
        ("StopReq", "SCC12", 0),
        ("CR_VSM_DecCmd", "SCC12", 0),
        ("aReqRaw", "SCC12", 0), #aReqMax
        ("TakeOverReq", "SCC12", 0),
        ("PreFill", "SCC12", 0),
        ("aReqValue", "SCC12", 0), #aReqMin
        ("CF_VSM_ConfMode", "SCC12", 1),
        ("AEB_Failinfo", "SCC12", 0),
        ("AEB_Status", "SCC12", 2),
        ("AEB_CmdAct", "SCC12", 0),
        ("AEB_StopReq", "SCC12", 0),
        ("CR_VSM_Alive", "SCC12", 0),
        ("CR_VSM_ChkSum", "SCC12", 0),

        ("SCCDrvModeRValue", "SCC13", 2),
        ("SCC_Equip", "SCC13", 1),
        ("AebDrvSetStatus", "SCC13", 0),

        ("JerkUpperLimit", "SCC14", 0),
        ("JerkLowerLimit", "SCC14", 0),
        ("SCCMode2", "SCC14", 0),
        ("ComfortBandUpper", "SCC14", 0),
        ("ComfortBandLower", "SCC14", 0),

        ("UNIT", "TPMS11"),
        ("PRESSURE_FL", "TPMS11"),
        ("PRESSURE_FR", "TPMS11"),
        ("PRESSURE_RL", "TPMS11"),
        ("PRESSURE_RR", "TPMS11"),
      ]

    checks = [
      # address, frequency
      ("TCS13", 50),
      ("TCS15", 10),
      ("CLU11", 50),
      ("ESP12", 100),
      ("CGW1", 10),
      ("CGW2", 5),
      ("CGW4", 5),
      ("WHL_SPD11", 50),
    ]

    if not CP.radarDisable:
      signals += [
      ("MainMode_ACC", "SCC11", 1),
      ("SCCInfoDisplay", "SCC11", 0),
      ("AliveCounterACC", "SCC11", 0),
      ("VSetDis", "SCC11", 30),
      ("ObjValid", "SCC11", 0),
      ("DriverAlertDisplay", "SCC11", 0),
      ("TauGapSet", "SCC11", 4),
      ("ACC_ObjStatus", "SCC11", 0),
      ("ACC_ObjLatPos", "SCC11", 0),
      ("ACC_ObjDist", "SCC11", 150), #TK211X value is 204.6
      ("ACC_ObjRelSpd", "SCC11", 0),
      ("Navi_SCC_Curve_Status", "SCC11", 0),
      ("Navi_SCC_Curve_Act", "SCC11", 0),
      ("Navi_SCC_Camera_Act", "SCC11", 0),
      ("Navi_SCC_Camera_Status", "SCC11", 2),
      ("ACCMode", "SCC12", 0),
      ("CF_VSM_Prefill", "SCC12", 0),
      ("CF_VSM_DecCmdAct", "SCC12", 0),
      ("CF_VSM_HBACmd", "SCC12", 0),
      ("CF_VSM_Warn", "SCC12", 0),
      ("CF_VSM_Stat", "SCC12", 0),
      ("CF_VSM_BeltCmd", "SCC12", 0),
      ("ACCFailInfo", "SCC12", 0),
      ("StopReq", "SCC12", 0),
      ("CR_VSM_DecCmd", "SCC12", 0),
      ("aReqRaw", "SCC12", 0), #aReqMax
      ("TakeOverReq", "SCC12", 0),
      ("PreFill", "SCC12", 0),
      ("aReqValue", "SCC12", 0), #aReqMin
      ("CF_VSM_ConfMode", "SCC12", 1),
      ("AEB_Failinfo", "SCC12", 0),
      ("AEB_Status", "SCC12", 2),
      ("AEB_CmdAct", "SCC12", 0),
      ("AEB_StopReq", "SCC12", 0),
      ("CR_VSM_Alive", "SCC12", 0),
      ("CR_VSM_ChkSum", "SCC12", 0),
      ("SCCDrvModeRValue", "SCC13", 2),
      ("SCC_Equip", "SCC13", 1),
      ("AebDrvSetStatus", "SCC13", 0),
      ("JerkUpperLimit", "SCC14", 0),
      ("JerkLowerLimit", "SCC14", 0),
      ("SCCMode2", "SCC14", 0),
      ("ComfortBandUpper", "SCC14", 0),
      ("ComfortBandLower", "SCC14", 0),
      ]

    if CP.sccBus == 0 and CP.pcmCruise and not CP.radarDisable:
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
    if CP.mdpsBus == 0:
      signals += [
        ("CR_Mdps_StrColTq", "MDPS12", 0),
        ("CF_Mdps_Def", "MDPS12", 0),
        ("CF_Mdps_ToiActive", "MDPS12", 0),
        ("CF_Mdps_ToiUnavail", "MDPS12", 0),
        ("CF_Mdps_ToiFlt", "MDPS12", 0),
        ("CF_Mdps_MsgCount2", "MDPS12", 0),
        ("CF_Mdps_Chksum2", "MDPS12", 0),
        ("CF_Mdps_SErr", "MDPS12", 0),
        ("CR_Mdps_StrTq", "MDPS12", 0),
        ("CF_Mdps_FailStat", "MDPS12", 0),
        ("CR_Mdps_OutTq", "MDPS12", 0),
        ("CR_Mdps_DrvTq", "MDPS11", 0),
      ]
      checks += [
        ("MDPS12", 50),
        ("MDPS11", 100),
      ]
      if CP.spasEnabled:
        signals += [
          ("CR_Mdps_StrAng", "MDPS11", 0),
          ("CF_Mdps_Stat", "MDPS11", 0),
        ]
        checks += [("MDPS11", 100)]
    elif CP.mdpsBus == 1: # If MDPS bus is 1 do this for bus 0. I.E. Going to spoof! - JPR
      if CP.spasEnabled:        
        if CP.emsType == 1:
          signals += [
            ("TQI_1", "EMS_366", 0),
            ("N", "EMS_366", 0),
            ("TQI_2", "EMS_366", 0),
            ("VS", "EMS_366", 0),
            ("SWI_IGK", "EMS_366", 0),
          ]
          checks += [("EMS_366", 100)]
        elif CP.emsType == 2:
          signals += [
            ("SWI_IGK", "EMS11", 0),
            ("F_N_ENG", "EMS11", 0),
            ("ACK_TCS", "EMS11", 0),
            ("PUC_STAT", "EMS11", 0),
            ("TQ_COR_STAT", "EMS11", 0),
            ("RLY_AC", "EMS11", 0),
            ("F_SUB_TQI", "EMS11", 0),
            ("TQI_ACOR", "EMS11", 0),
            ("N", "EMS11", 0),
            ("TQI", "EMS11", 0),
            ("TQFR", "EMS11", 0),
            ("VS", "EMS11", 0),
            ("RATIO_TQI_BAS_MAX_STND", "EMS11", 0),
          ]
          checks += [("EMS11", 100)]
        elif CP.emsType == 3:
          signals += [
            ("Brake_Pedal_Pos", "E_EMS11", 0),
            ("IG_Reactive_Stat", "E_EMS11", 0),
            ("Gear_Change", "E_EMS11", 0),
            ("Cruise_Limit_Status", "E_EMS11", 0),
            ("Cruise_Limit_Target", "E_EMS11", 0),
            ("Accel_Pedal_Pos", "E_EMS11", 0),
            ("CR_Vcu_AccPedDep_Pos", "E_EMS11", 0),
          ]
          checks += [("E_EMS11", 100)]

    if CP.sasBus == 0:
      signals += [
        ("SAS_Angle", "SAS11"),
        ("SAS_Speed", "SAS11"),
      ]
      checks += [
        ("SAS11", 100)
      ]
    if CP.sccBus == -1:
      signals += [
        ("CRUISE_LAMP_M", "EMS16"),
        ("CF_Lvr_CruiseSet", "LVR12"),
    ]
    if CP.carFingerprint in FEATURES["use_cluster_gears"]:
      signals += [
        ("CF_Clu_Gear", "CLU15"),
      ]
    elif CP.carFingerprint in FEATURES["use_tcu_gears"]:
      signals += [
        ("CUR_GR", "TCU12"),
      ]
    elif CP.carFingerprint in FEATURES["use_elect_gears"]:
      signals += [
        ("Elect_Gear_Shifter", "ELECT_GEAR"),
    ]
    else:
      signals += [
        ("CF_Lvr_Gear","LVR12"),
      ]

    if CP.carFingerprint in EV_HYBRID_CAR:
      if CP.carFingerprint in HYBRID_CAR:
        signals += [
          ("CR_Vcu_AccPedDep_Pos", "E_EMS11")
        ]
      else:
        signals += [
          ("Accel_Pedal_Pos", "E_EMS11"),
        ]
      checks += [
        ("E_EMS11", 50),
      ]

    else:
      signals += [
        ("PV_AV_CAN", "EMS12"),
        ("CF_Ems_AclAct", "EMS16"),
      ]
      checks += [
        ("EMS12", 100),
        ("EMS16", 100),
      ]

    if CP.carFingerprint in FEATURES["use_fca"]:
      signals += [
        ("FCA_CmdAct", "FCA11"),
        ("CF_VSM_Warn", "FCA11"),
      ]


    if CP.carFingerprint in [CAR.SANTA_FE, CAR.SANTA_FE_2022]:
      checks.remove(("TCS13", 50))


    if CP.enableBsm:
      signals += [
        ("CF_Lca_IndLeft", "LCA11"),
        ("CF_Lca_IndRight", "LCA11"),
      ]
      checks += [("LCA11", 50)]

    if CP.enableAutoHold:
      signals += [
        ("AVH_STAT", "ESP11"),
        ("LDM_STAT", "ESP11"),
      ]
      checks += [("ESP11", 50)]

    if Params().get_bool("HyundaiNaviSL"):
      signals += [
        ("SpeedLim_Nav_Clu", "Navi_HU", 0),
      ]
      checks += [
        ("Navi_HU", 5)
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0, enforce_checks=False)

  @staticmethod
  def get_can2_parser(CP):
    signals = []
    checks = []
    if CP.mdpsBus == 1:
      if CP.spasEnabled: # Need Check this and get signal from bus 1 if MDPS on bus 1. - JPR
        signals += [
          ("CR_Mdps_StrAng", "MDPS11", 0),
          ("CF_Mdps_Stat", "MDPS11", 0),
        ]
      signals += [
        ("CR_Mdps_StrColTq", "MDPS12", 0),
        ("CF_Mdps_Def", "MDPS12", 0),
        ("CF_Mdps_ToiActive", "MDPS12", 0),
        ("CF_Mdps_ToiUnavail", "MDPS12", 0),
        ("CF_Mdps_ToiFlt", "MDPS12", 0),
        ("CF_Mdps_MsgCount2", "MDPS12", 0),
        ("CF_Mdps_Chksum2", "MDPS12", 0),
        ("CF_Mdps_SErr", "MDPS12", 0),
        ("CR_Mdps_StrTq", "MDPS12", 0),
        ("CF_Mdps_FailStat", "MDPS12", 0),
        ("CR_Mdps_OutTq", "MDPS12", 0),
        ("CR_Mdps_DrvTq", "MDPS11", 0),
      ]
      checks += [
        ("MDPS12", 50),
        ("MDPS11", 100),
      ]

    if CP.sasBus == 1:
      signals += [
        ("SAS_Angle", "SAS11"),
        ("SAS_Speed", "SAS11"),
      ]
      checks += [
        ("SAS11", 100)
      ]

    if CP.sccBus == 1:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("AliveCounterACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("ObjValid", "SCC11"),
        ("DriverAlertDisplay", "SCC11"),
        ("TauGapSet", "SCC11"),
        ("ACC_ObjStatus", "SCC11"),
        ("ACC_ObjLatPos", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("ACC_ObjRelSpd", "SCC11"),
        ("Navi_SCC_Curve_Status", "SCC11"),
        ("Navi_SCC_Curve_Act", "SCC11"),
        ("Navi_SCC_Camera_Act", "SCC11"),
        ("Navi_SCC_Camera_Status", "SCC11"),


        ("ACCMode", "SCC12"),
        ("CF_VSM_Prefill", "SCC12"),
        ("CF_VSM_DecCmdAct", "SCC12"),
        ("CF_VSM_HBACmd", "SCC12"),
        ("CF_VSM_Warn", "SCC12"),
        ("CF_VSM_Stat", "SCC12"),
        ("CF_VSM_BeltCmd", "SCC12"),
        ("ACCFailInfo", "SCC12"),
        ("StopReq", "SCC12"),
        ("CR_VSM_DecCmd", "SCC12"),
        ("aReqRaw", "SCC12"), #aReqMax
        ("TakeOverReq", "SCC12"),
        ("PreFill", "SCC12"),
        ("aReqValue", "SCC12"), #aReqMin
        ("CF_VSM_ConfMode", "SCC12"),
        ("AEB_Failinfo", "SCC12"),
        ("AEB_Status", "SCC12"),
        ("AEB_CmdAct", "SCC12"),
        ("AEB_StopReq", "SCC12"),
        ("CR_VSM_Alive", "SCC12"),
        ("CR_VSM_ChkSum", "SCC12"),

        ("SCCDrvModeRValue", "SCC13"),
        ("SCC_Equip", "SCC13"),
        ("AebDrvSetStatus", "SCC13"),

        ("JerkUpperLimit", "SCC14"),
        ("JerkLowerLimit", "SCC14"),
        ("SCCMode2", "SCC14"),
        ("ComfortBandUpper", "SCC14"),
        ("ComfortBandLower", "SCC14"),

      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 1, enforce_checks=False)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("CF_Lkas_LdwsActivemode", "LKAS11"),
      ("CF_Lkas_LdwsSysState", "LKAS11"),
      ("CF_Lkas_SysWarning", "LKAS11"),
      ("CF_Lkas_LdwsLHWarning", "LKAS11"),
      ("CF_Lkas_LdwsRHWarning", "LKAS11"),
      ("CF_Lkas_HbaLamp", "LKAS11"),
      ("CF_Lkas_FcwBasReq", "LKAS11"),
      ("CF_Lkas_ToiFlt", "LKAS11"),
      ("CF_Lkas_HbaSysState", "LKAS11"),
      ("CF_Lkas_FcwOpt", "LKAS11"),
      ("CF_Lkas_HbaOpt", "LKAS11"),
      ("CF_Lkas_FcwSysState", "LKAS11"),
      ("CF_Lkas_FcwCollisionWarning", "LKAS11"),
      ("CF_Lkas_MsgCount", "LKAS11"),
      ("CF_Lkas_FusionState", "LKAS11"),
      ("CF_Lkas_FcwOpt_USM", "LKAS11"),
      ("CF_Lkas_LdwsOpt_USM", "LKAS11"),
    ]

    checks = [
      ("LKAS11", 100)
    ]


    if CP.sccBus == 2:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("AliveCounterACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("ObjValid", "SCC11"),
        ("DriverAlertDisplay", "SCC11"),
        ("TauGapSet", "SCC11"),
        ("ACC_ObjStatus", "SCC11"),
        ("ACC_ObjLatPos", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("ACC_ObjRelSpd", "SCC11"),
        ("Navi_SCC_Curve_Status", "SCC11"),
        ("Navi_SCC_Curve_Act", "SCC11"),
        ("Navi_SCC_Camera_Act", "SCC11"),
        ("Navi_SCC_Camera_Status", "SCC11"),

        ("ACCMode", "SCC12"),
        ("CF_VSM_Prefill", "SCC12"),
        ("CF_VSM_DecCmdAct", "SCC12"),
        ("CF_VSM_HBACmd", "SCC12"),
        ("CF_VSM_Warn", "SCC12"),
        ("CF_VSM_Stat", "SCC12"),
        ("CF_VSM_BeltCmd", "SCC12"),
        ("ACCFailInfo", "SCC12"),
        ("StopReq", "SCC12"),
        ("CR_VSM_DecCmd", "SCC12"),
        ("aReqRaw", "SCC12"), #aReqMax
        ("TakeOverReq", "SCC12"),
        ("PreFill", "SCC12"),
        ("aReqValue", "SCC12"), #aReqMin
        ("CF_VSM_ConfMode", "SCC12"),
        ("AEB_Failinfo", "SCC12"),
        ("AEB_Status", "SCC12"),
        ("AEB_CmdAct", "SCC12"),
        ("AEB_StopReq", "SCC12"),
        ("CR_VSM_Alive", "SCC12"),
        ("CR_VSM_ChkSum", "SCC12"),

        ("SCCDrvModeRValue", "SCC13"),
        ("SCC_Equip", "SCC13"),
        ("AebDrvSetStatus", "SCC13"),

        ("JerkUpperLimit", "SCC14"),
        ("JerkLowerLimit", "SCC14"),
        ("SCCMode2", "SCC14"),
        ("ComfortBandUpper", "SCC14"),
        ("ComfortBandLower", "SCC14"),
      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]

      if CP.hasLfaHda:
        signals += [
          ("HDA_USM", "LFAHDA_MFC"),
          ("HDA_Active", "LFAHDA_MFC"),
          ("HDA_Icon_State", "LFAHDA_MFC"),
          ("HDA_LdwSysState", "LFAHDA_MFC"),
          ("HDA_Icon_Wheel", "LFAHDA_MFC"),
        ]
        checks += [("LFAHDA_MFC", 20)]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2, enforce_checks=False)
