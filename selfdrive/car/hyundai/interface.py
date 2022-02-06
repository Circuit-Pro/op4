#!/usr/bin/env python3
import os
import numpy as np
from cereal import car
from panda import Panda
from common.numpy_fast import interp
from selfdrive.config import Conversions as CV
from selfdrive.car.hyundai.values import CAR, Buttons, CarControllerParams, LEGACY_SAFETY_MODE_CAR
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from common.params import Params
from selfdrive.car.disable_ecu import disable_ecu

GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.cp2 = self.CS.get_can2_parser(CP)
    self.mad_mode_enabled = Params().get_bool('MadModeEnabled')

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):

    v_current_kph = current_speed * CV.MS_TO_KPH

    gas_max_bp = [0., 10., 20., 50., 70., 130.]
    gas_max_v = [CarControllerParams.ACCEL_MAX, 2., 1.8, 1.5, 1., 0.55, 0.33]

    return CarControllerParams.ACCEL_MIN, interp(v_current_kph, gas_max_bp, gas_max_v)

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.openpilotLongitudinalControl = Params().get_bool('LongControlEnabled') or Params().get_bool('RadarDisableEnabled') or Params().get_bool('DisableRadar')
    ret.radarDisable = Params().get_bool('DisableRadar')
    ret.radarDisableOld = Params().get_bool('RadarDisableEnabled')
    ret.radarDisablePossible = Params().get_bool('RadarDisableEnabled') or Params().get_bool('DisableRadar')
    
    ret.carName = "hyundai"
    # these cars require a special panda safety mode due to missing counters and checksums in the messages
    #if candidate in LEGACY_SAFETY_MODE_CAR:
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]
    #else:
    #  ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundai, 0)]

    tire_stiffness_factor = 1.
    if Params().get_bool('SteerLockout'):
      ret.maxSteeringAngleDeg = 1000
      ret.steerLockout = False
    else:
      ret.steerLockout = True
      ret.maxSteeringAngleDeg = 90
    UseLQR = Params().get_bool('UseLQR')
    # lateral LQR global hyundai
    if UseLQR:
      ret.lateralTuning.init('lqr')
      ret.lateralTuning.lqr.scale = 1600.
      ret.lateralTuning.lqr.ki = 0.01
      ret.lateralTuning.lqr.dcGain = 0.0027

      ret.lateralTuning.lqr.a = [0., 1., -0.22619643, 1.21822268]
      ret.lateralTuning.lqr.b = [-1.92006585e-04, 3.95603032e-05]
      ret.lateralTuning.lqr.c = [1., 0.]
      ret.lateralTuning.lqr.k = [-110., 451.]
      ret.lateralTuning.lqr.l = [0.33, 0.318]

    ret.steerRatio = 16.5
    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 0.4

    ret.steerLimitTimer = 2.5
    ret.steerMaxBP = [0.]
    ret.steerMaxV = [2.]
    ret.emsType = 0

   #Longitudinal Tune and logic for car tune
    if candidate is not CAR.GENESIS_G70 or CAR.STINGER or CAR.GENESIS or CAR.GENESIS_G80 or CAR.KONA or CAR.KONA_EV or CAR.NIRO_EV or CAR.GENESIS_EQ900 or CAR.GENESIS_G90: #Tune for untuned cars
      # Donfyffe stock tune for untuned cars
      if not UseLQR:
        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [0.]
        ret.lateralTuning.indi.innerLoopGainV = [3.1]
        ret.lateralTuning.indi.outerLoopGainBP = [0.]
        ret.lateralTuning.indi.outerLoopGainV = [2.5]
        ret.lateralTuning.indi.timeConstantBP = [0.]
        ret.lateralTuning.indi.timeConstantV = [1.4]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.]

      ret.longitudinalTuning.kpBP = [0., 10.*CV.KPH_TO_MS, 20.*CV.KPH_TO_MS, 40.*CV.KPH_TO_MS, 70.*CV.KPH_TO_MS, 100.*CV.KPH_TO_MS, 130.*CV.KPH_TO_MS]
      ret.longitudinalTuning.kpV = [1.2, 0.93, 0.8, 0.68, 0.59, 0.51, 0.43]
      ret.longitudinalTuning.kiBP = [0., 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kiV = [0.06, 0.03]
	
    ret.longitudinalTuning.deadzoneBP = [0., 30.*CV.KPH_TO_MS]
    ret.longitudinalTuning.deadzoneV = [0., 0.05]
    ret.longitudinalActuatorDelayLowerBound = 0.15
    ret.longitudinalActuatorDelayUpperBound = 0.2

    ret.stopAccel = -2.5
    ret.stoppingDecelRate = 0.73  # brake_travel/s while trying to stop
    ret.vEgoStopping = 0.8
    ret.vEgoStarting = 0.5  # needs to be >= vEgoStopping to avoid state transition oscillation

    # genesis
    if candidate == CAR.GENESIS:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Genesis.png img_spinner_comma.png")
      ret.mass = 1900. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.centerToFront = ret.wheelbase * 0.4
      if not Params().get_bool('UseSMDPSHarness'):
        ret.minSteerSpeed = 60 * CV.KPH_TO_MS
      ret.steerRatio = 16.5
      tire_stiffness_factor = 0.85
      ret.maxSteeringAngleDeg = 90.
      ret.longitudinalTuning.kpBP = [0., 10.*CV.KPH_TO_MS, 20.*CV.KPH_TO_MS, 40.*CV.KPH_TO_MS, 70.*CV.KPH_TO_MS, 100.*CV.KPH_TO_MS, 130.*CV.KPH_TO_MS]
      ret.longitudinalTuning.kpV = [1.2, 0.93, 0.8, 0.68, 0.59, 0.51, 0.43]
      ret.longitudinalTuning.kiBP = [0., 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kiV = [0.06, 0.03]
      ret.emsType = 1   
      if not UseLQR:
        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [0.]
        ret.lateralTuning.indi.innerLoopGainV = [3.1]
        ret.lateralTuning.indi.outerLoopGainBP = [0.]
        ret.lateralTuning.indi.outerLoopGainV = [2.5]
        ret.lateralTuning.indi.timeConstantBP = [0.]
        ret.lateralTuning.indi.timeConstantV = [1.4]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.]

    elif candidate == CAR.GENESIS_G70:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Genesis.png img_spinner_comma.png")
      ret.emsType = 1 
      tire_stiffness_factor = 0.85
      ret.steerRatio = 13.56
      ret.mass = 1640. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.centerToFront = ret.wheelbase * 0.4
      if not UseLQR:
        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [0.]
        ret.lateralTuning.indi.innerLoopGainV = [3.65]
        ret.lateralTuning.indi.outerLoopGainBP = [0.]
        ret.lateralTuning.indi.outerLoopGainV = [2.5]
        ret.lateralTuning.indi.timeConstantBP = [0.]
        ret.lateralTuning.indi.timeConstantV = [1.4]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.]

      tire_stiffness_factor = 0.85
      ret.steerRatio = 13.56
      ret.mass = 1640. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.centerToFront = ret.wheelbase * 0.4
      ret.longitudinalTuning.kpBP = [0, 10. * CV.KPH_TO_MS, 20. * CV.KPH_TO_MS, 40. * CV.KPH_TO_MS, 70. * CV.KPH_TO_MS, 100. * CV.KPH_TO_MS, 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kpV = [0.6, 0.58, 0.55, 0.48, 0.45, 0.40, 0.35]
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.015]

    elif candidate == CAR.GENESIS_G80:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Genesis.png img_spinner_comma.png")
      ret.mass = 1855. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.centerToFront = ret.wheelbase * 0.4
      ret.steerRatio = 16.5
      tire_stiffness_factor = 0.85
      ret.emsType = 1 
      if not UseLQR:
        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [0.]
        ret.lateralTuning.indi.innerLoopGainV = [3.1]
        ret.lateralTuning.indi.outerLoopGainBP = [0.]
        ret.lateralTuning.indi.outerLoopGainV = [2.5]
        ret.lateralTuning.indi.timeConstantBP = [0.]
        ret.lateralTuning.indi.timeConstantV = [1.4]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.]
      
      tire_stiffness_factor = 0.85
      ret.longitudinalTuning.kpBP = [0., 10.*CV.KPH_TO_MS, 20.*CV.KPH_TO_MS, 40.*CV.KPH_TO_MS, 70.*CV.KPH_TO_MS, 100.*CV.KPH_TO_MS, 130.*CV.KPH_TO_MS]
      ret.longitudinalTuning.kpV = [1.2, 0.93, 0.8, 0.68, 0.59, 0.51, 0.43]
      ret.longitudinalTuning.kiBP = [0., 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kiV = [0.06, 0.03]

    elif candidate == CAR.GENESIS_EQ900:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Genesis.png img_spinner_comma.png")
      ret.mass = 2200
      ret.wheelbase = 3.15
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 0.85
      ret.emsType = 1 
      if not UseLQR:
        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [0.]
        ret.lateralTuning.indi.innerLoopGainV = [3.1]
        ret.lateralTuning.indi.outerLoopGainBP = [0.]
        ret.lateralTuning.indi.outerLoopGainV = [2.5]
        ret.lateralTuning.indi.timeConstantBP = [0.]
        ret.lateralTuning.indi.timeConstantV = [1.4]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.]
      ret.longitudinalTuning.kpBP = [0., 10.*CV.KPH_TO_MS, 20.*CV.KPH_TO_MS, 40.*CV.KPH_TO_MS, 70.*CV.KPH_TO_MS, 100.*CV.KPH_TO_MS, 130.*CV.KPH_TO_MS]
      ret.longitudinalTuning.kpV = [1.2, 0.93, 0.8, 0.68, 0.59, 0.51, 0.43]
      ret.longitudinalTuning.kiBP = [0., 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kiV = [0.06, 0.03]

    elif candidate == CAR.GENESIS_EQ900_L:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Genesis.png img_spinner_comma.png")
      ret.mass = 2290
      ret.wheelbase = 3.45
      ret.centerToFront = ret.wheelbase * 0.4
      ret.emsType = 1 
      tire_stiffness_factor = 0.85
      ret.longitudinalTuning.kpBP = [0., 10.*CV.KPH_TO_MS, 20.*CV.KPH_TO_MS, 40.*CV.KPH_TO_MS, 70.*CV.KPH_TO_MS, 100.*CV.KPH_TO_MS, 130.*CV.KPH_TO_MS]
      ret.longitudinalTuning.kpV = [1.2, 0.93, 0.8, 0.68, 0.59, 0.51, 0.43]
      ret.longitudinalTuning.kiBP = [0., 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kiV = [0.06, 0.03]
      if not UseLQR:
        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [0.]
        ret.lateralTuning.indi.innerLoopGainV = [3.1]
        ret.lateralTuning.indi.outerLoopGainBP = [0.]
        ret.lateralTuning.indi.outerLoopGainV = [2.5]
        ret.lateralTuning.indi.timeConstantBP = [0.]
        ret.lateralTuning.indi.timeConstantV = [1.4]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.]

    elif candidate == CAR.GENESIS_G90:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Genesis.png img_spinner_comma.png")
      ret.mass = 2150
      ret.wheelbase = 3.16
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 0.85
      ret.emsType = 1 
      ret.longitudinalTuning.kpBP = [0., 10.*CV.KPH_TO_MS, 20.*CV.KPH_TO_MS, 40.*CV.KPH_TO_MS, 70.*CV.KPH_TO_MS, 100.*CV.KPH_TO_MS, 130.*CV.KPH_TO_MS]
      ret.longitudinalTuning.kpV = [1.2, 0.93, 0.8, 0.68, 0.59, 0.51, 0.43]
      ret.longitudinalTuning.kiBP = [0., 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kiV = [0.06, 0.03] 
      if not UseLQR:
        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [0.]
        ret.lateralTuning.indi.innerLoopGainV = [3.1]
        ret.lateralTuning.indi.outerLoopGainBP = [0.]
        ret.lateralTuning.indi.outerLoopGainV = [2.5]
        ret.lateralTuning.indi.timeConstantBP = [0.]
        ret.lateralTuning.indi.timeConstantV = [1.4]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.]

    # hyundai
    elif candidate in [CAR.SANTA_FE]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = 1694 + STD_CARGO_KG
      ret.wheelbase = 2.766
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      tire_stiffness_factor = 0.65
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.SANTA_FE_2022, CAR.SANTA_FE_HEV_2022]:
      ret.mass = 1750 + STD_CARGO_KG
      ret.wheelbase = 2.766
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 0.65
    elif candidate in [CAR.SONATA, CAR.SONATA_HEV, CAR.SONATA21_HEV]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = 1513. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 0.65
    elif candidate in [CAR.SONATA19, CAR.SONATA19_HEV]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = 4497. * CV.LB_TO_KG
      ret.wheelbase = 2.804
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.SONATA_LF_TURBO:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = 1590. + STD_CARGO_KG
      ret.wheelbase = 2.805
      tire_stiffness_factor = 0.65
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.PALISADE:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = 1999. + STD_CARGO_KG
      ret.wheelbase = 2.90
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.ELANTRA, CAR.ELANTRA_GT_I30]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      tire_stiffness_factor = 0.7
      ret.steerRatio = 15.4            # 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535
      ret.centerToFront = ret.wheelbase * 0.4
      if not Params().get_bool('UseSMDPSHarness'):
        ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.ELANTRA_2021:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = (2800. * CV.LB_TO_KG) + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      tire_stiffness_factor = 0.65
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.ELANTRA_HEV_2021:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = (3017. * CV.LB_TO_KG) + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 13.27 * 1.15  # 15% higher at the center seems reasonable
      tire_stiffness_factor = 0.65
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.KONA:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73 * 1.15  # Spec
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 0.7
      ret.emsType = 1 
      if not UseLQR:
        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [0.]
        ret.lateralTuning.indi.innerLoopGainV = [3.1]
        ret.lateralTuning.indi.outerLoopGainBP = [0.]
        ret.lateralTuning.indi.outerLoopGainV = [2.5]
        ret.lateralTuning.indi.timeConstantBP = [0.]
        ret.lateralTuning.indi.timeConstantV = [1.4]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.]

       #Tune To base Kona tune off of.
      ret.longitudinalTuning.kpBP = [0, 10. * CV.KPH_TO_MS, 20. * CV.KPH_TO_MS, 40. * CV.KPH_TO_MS, 70. * CV.KPH_TO_MS, 100. * CV.KPH_TO_MS, 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kpV = [1.20, 1.1, 1.05, 1.0, 0.95, 0.90, 0.85]
      ret.longitudinalTuning.kiBP = [0, 130.*CV.KPH_TO_MS]
      ret.longitudinalTuning.kiV = [0.07, 0.03]


    elif candidate in [CAR.KONA_HEV, CAR.KONA_EV]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = 1395. + STD_CARGO_KG
      ret.wheelbase = 2.6
      ret.steerRatio = 13.73  # Spec
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
      ret.emsType = 3
      if not UseLQR:
        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [0.]
        ret.lateralTuning.indi.innerLoopGainV = [3.1]
        ret.lateralTuning.indi.outerLoopGainBP = [0.]
        ret.lateralTuning.indi.outerLoopGainV = [2.5]
        ret.lateralTuning.indi.timeConstantBP = [0.]
        ret.lateralTuning.indi.timeConstantV = [1.4]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.]

 #Tune To base Kona EV tune off of.
      ret.longitudinalTuning.kpBP = [0, 10. * CV.KPH_TO_MS, 20. * CV.KPH_TO_MS, 40. * CV.KPH_TO_MS, 70. * CV.KPH_TO_MS, 100. * CV.KPH_TO_MS, 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kpV = [1.20, 1.1, 1.05, 1.0, 0.95, 0.90, 0.85]
      ret.longitudinalTuning.kiBP = [0, 130.*CV.KPH_TO_MS]
      ret.longitudinalTuning.kiV = [0.07, 0.03]

    elif candidate in [CAR.IONIQ, CAR.IONIQ_EV_LTD, CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.IONIQ_HEV]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = 1490. + STD_CARGO_KG
      ret.steerRatio = 13.73  # Spec
      ret.wheelbase = 2.7
      tire_stiffness_factor = 0.385
      ret.emsType = 3
      if candidate not in [CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV] and not Params().get_bool('UseSMDPSHarness'):
        ret.minSteerSpeed = 32 * CV.MPH_TO_MS
      ret.centerToFront = ret.wheelbase * 0.4

    elif candidate in [CAR.GRANDEUR_IG, CAR.GRANDEUR_IG_HEV]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      tire_stiffness_factor = 0.8
      ret.mass = 1640. + STD_CARGO_KG
      ret.wheelbase = 2.845
      ret.centerToFront = ret.wheelbase * 0.385
      ret.steerRatio = 17.
    elif candidate in [CAR.GRANDEUR_IG_FL, CAR.GRANDEUR_IG_FL_HEV]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      tire_stiffness_factor = 0.8
      ret.mass = 1725. + STD_CARGO_KG
      ret.wheelbase = 2.885
      ret.centerToFront = ret.wheelbase * 0.385
      ret.steerRatio = 17.
    elif candidate == CAR.VELOSTER:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      tire_stiffness_factor = 0.9
      ret.steerRatio = 13.75 * 1.15
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.TUCSON_TL_SCC:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Hyundai.png img_spinner_comma.png")
      ret.mass = 1594. + STD_CARGO_KG #1730
      ret.wheelbase = 2.67
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    # kia
    elif candidate == CAR.SORENTO:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Kia.png img_spinner_comma.png")
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.1   # 10% higher at the center seems reasonable
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.K5, CAR.K5_HEV, CAR.KIA_K5_2021]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Kia.png img_spinner_comma.png")
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      tire_stiffness_factor = 0.7
      ret.steerRatio = 13.75
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.K5_2021]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Kia.png img_spinner_comma.png")
      ret.mass = 3228. * CV.LB_TO_KG
      ret.wheelbase = 2.85
      tire_stiffness_factor = 0.7
    elif candidate == CAR.STINGER:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Stinger.png img_spinner_comma.png")
      tire_stiffness_factor = 1.0
      ret.mass = 1640.0 + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 14.4 * 1.15   # 15% higher at the center seems reasonable - before was 14.44 
      ret.centerToFront = ret.wheelbase * 0.4
      ret.emsType = 1 

      if not UseLQR:
        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [0.]
        ret.lateralTuning.indi.innerLoopGainV = [3.62]
        ret.lateralTuning.indi.outerLoopGainBP = [0.]
        ret.lateralTuning.indi.outerLoopGainV = [2.5]
        ret.lateralTuning.indi.timeConstantBP = [0.]
        ret.lateralTuning.indi.timeConstantV = [1.4]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.]
      
      ret.longitudinalTuning.kpBP = [0, 10. * CV.KPH_TO_MS, 20. * CV.KPH_TO_MS, 40. * CV.KPH_TO_MS, 70. * CV.KPH_TO_MS, 100. * CV.KPH_TO_MS, 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kpV = [1.22, 1.155, 1.07, 0.98, 0.92, 0.87, 0.82]
      ret.longitudinalTuning.kiBP = [0., 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kiV = [0.05, 0.03]

    elif candidate == CAR.FORTE:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Kia.png img_spinner_comma.png")
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.CEED:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Kia.png img_spinner_comma.png")
      ret.mass = 1350. + STD_CARGO_KG
      ret.wheelbase = 2.65
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.6
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.SPORTAGE:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Kia.png img_spinner_comma.png")
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.NIRO_HEV, CAR.NIRO_HEV_2021]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Kia.png img_spinner_comma.png")
      ret.mass = 1737. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      tire_stiffness_factor = 0.7
      ret.emsType = 3
      ret.centerToFront = ret.wheelbase * 0.4
      if candidate == CAR.NIRO_HEV and not Params().get_bool('UseSMDPSHarness'):
        ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate in [CAR.NIRO_EV]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Kia.png img_spinner_comma.png")
      ret.mass = 1850. + STD_CARGO_KG # 265 hp version
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73 *1.15 # 15% increase from spec
      tire_stiffness_factor = 0.8 # works good with 17" wheels
      ret.centerToFront = ret.wheelbase * 0.4
      ret.emsType = 3
      
      if not UseLQR:
        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [0.]
        ret.lateralTuning.indi.innerLoopGainV = [3.9] 
        ret.lateralTuning.indi.outerLoopGainBP = [0.]
        ret.lateralTuning.indi.outerLoopGainV = [2.8] 
        ret.lateralTuning.indi.timeConstantBP = [0.]
        ret.lateralTuning.indi.timeConstantV = [1.0] 
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0.,60.*CV.KPH_TO_MS]
        ret.lateralTuning.indi.actuatorEffectivenessV = [1.7, 1.3]
        
      ret.longitudinalTuning.kpBP = [0, 10. * CV.KPH_TO_MS, 20. * CV.KPH_TO_MS, 40. * CV.KPH_TO_MS, 70. * CV.KPH_TO_MS, 100. * CV.KPH_TO_MS, 130. * CV.KPH_TO_MS]
      ret.longitudinalTuning.kpV = [1.32, 1.25, 1.17, 0.98, 0.91, 0.85, 0.8]
      ret.longitudinalTuning.kiBP = [0.,80.* CV.KPH_TO_MS, 100.* CV.KPH_TO_MS, 130.* CV.KPH_TO_MS]
      ret.longitudinalTuning.kiV = [0.08,0.05,0.04, 0.03]

    elif candidate in [CAR.K7, CAR.K7_HEV]:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Kia.png img_spinner_comma.png")
      tire_stiffness_factor = 0.7
      ret.mass = 1650. + STD_CARGO_KG
      ret.wheelbase = 2.855
      ret.centerToFront = ret.wheelbase * 0.4
      ret.steerRatio = 17.5
    elif candidate == CAR.SELTOS:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Kia.png img_spinner_comma.png")
      ret.mass = 1310. + STD_CARGO_KG
      ret.wheelbase = 2.6
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.K9:
      os.system("cd /data/openpilot/selfdrive/assets && rm -rf img_spinner_comma.png && cp Kia.png img_spinner_comma.png")
      ret.mass = 2005. + STD_CARGO_KG
      ret.wheelbase = 3.15
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 0.8
      ret.lateralTuning.lqr.scale = 1650.
      ret.lateralTuning.lqr.ki = 0.01
      ret.lateralTuning.lqr.dcGain = 0.0027

    ret.radarTimeStep = 0.05

    if ret.centerToFront == 0:
      ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    # no rear steering, at least on the listed cars above
    ret.steerRatioRear = 0.
    ret.steerControlType = car.CarParams.SteerControlType.torque

    ret.stoppingControl = True

    ret.enableBsm = 0x58b in fingerprint[0]
    ret.enableAutoHold = 1151 in fingerprint[0]

    # ignore CAN2 address if L-CAN on the same BUS
    ret.mdpsBus = 1 if 593 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sasBus = 1 if 688 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sccBus = 0 if 1056 in fingerprint[0] else 1 if 1056 in fingerprint[1] and 1296 not in fingerprint[1] \
                                                                     else 2 if 1056 in fingerprint[2] else -1

    if ret.sccBus >= 0:
      ret.hasScc13 = 1290 in fingerprint[ret.sccBus]
      ret.hasScc14 = 905 in fingerprint[ret.sccBus]

    ret.hasEms = 608 in fingerprint[0] and 809 in fingerprint[0]

    ret.radarOffCan = ret.sccBus == -1
    ret.pcmCruise = not ret.radarOffCan or not ret.radarDisablePossible

    # SPAS
    ret.spasEnabled = Params().get_bool('spasEnabled')

    # set safety_hyundai_community only for non-SCC, MDPS harrness or SCC harrness cars or cars that have unknown issue
    if ret.radarOffCan or ret.radarDisablePossible or ret.mdpsBus == 1 or ret.openpilotLongitudinalControl or ret.sccBus == 1 or Params().get_bool('MadModeEnabled') or Params().get_bool('spasEnabled'):
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCommunity, 0)]
    return ret
  
  @staticmethod
  def init(CP, logcan, sendcan):
    if CP.radarDisable:
      disable_ecu(logcan, sendcan, addr=0x7d0, com_cont_req=b'\x28\x83\x01')

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp2.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp2, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp2.can_valid and self.cp_cam.can_valid

    if self.CP.pcmCruise and (not self.CP.radarDisablePossible or not self.CP.radarOffCan):
      self.CP.pcmCruise = True
    elif not self.CP.pcmCruise and (self.CP.radarDisablePossible or self.CP.radarOffCan):
      self.CP.pcmCruise = False

    # most HKG cars has no long control, it is safer and easier to engage by main on

    if self.mad_mode_enabled:
      ret.cruiseState.enabled = ret.cruiseState.available

    # turning indicator alert logic
    if not self.CC.keep_steering_turn_signals and not self.CC.NoMinLaneChangeSpeed and (ret.leftBlinker or ret.rightBlinker or self.CC.turning_signal_timer) and ret.vEgo < LANE_CHANGE_SPEED_MIN - 1.2:
      self.CC.turning_indicator_alert = True
    else:
      self.CC.turning_indicator_alert = False

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 0.2) and self.CP.minSteerSpeed > 10.:
      self.CC.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 0.7):
      self.CC.low_speed_alert = False

    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise)

    buttonEvents = []
    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.unknown
      if self.CS.cruise_buttons != 0:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == Buttons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == Buttons.SET_DECEL:
        be.type = ButtonType.decelCruise
      elif but == Buttons.GAP_DIST:
        be.type = ButtonType.gapAdjustCruise
      elif but == Buttons.CANCEL:
        be.type = ButtonType.cancel
      else:
        be.type = ButtonType.unknown
      buttonEvents.append(be)

    if self.CS.cruise_main_button != self.CS.prev_cruise_main_button:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton3
      be.pressed = bool(self.CS.cruise_main_button)
      buttonEvents.append(be)

      ret.buttonEvents = buttonEvents
  # handle button presses
    for b in ret.buttonEvents:
      # do disable on button down
      if b.type == ButtonType.cancel and b.pressed:
        events.add(EventName.buttonCancel)
      if self.CC.longcontrol and (self.CP.radarDisablePossible or self.CP.radarOffCan):
        # do enable on both accel and decel buttons
        if b.type in [ButtonType.accelCruise, ButtonType.decelCruise] and not b.pressed:
          events.add(EventName.buttonEnable)
        if EventName.wrongCarMode in events.events:
          events.events.remove(EventName.wrongCarMode)
        if EventName.pcmDisable in events.events:
          events.events.remove(EventName.pcmDisable)
      elif not self.CC.longcontrol and ret.cruiseState.enabled:
        # do enable on decel button only
        if b.type == ButtonType.decelCruise and not b.pressed:
          events.add(EventName.buttonEnable)

      #for b in ret.buttonEvents:
        # do enable on both accel and decel buttons
        #if b.type in (ButtonType.accelCruise, ButtonType.decelCruise) and not b.pressed:
          #events.add(EventName.buttonEnable)
        # do disable on button down
        #if b.type == ButtonType.cancel and b.pressed:
          #events.add(EventName.buttonCancel)

    if self.CC.longcontrol and self.CS.cruise_unavail:
      events.add(EventName.brakeUnavailable)
    #if abs(ret.steeringAngleDeg) > 90. and EventName.steerTempUnavailable not in events.events:
    #  events.add(EventName.steerTempUnavailable)
    if self.low_speed_alert and not self.CS.mdps_bus:
      events.add(EventName.belowSteerSpeed)
    if self.CC.turning_indicator_alert:
      events.add(EventName.turningIndicatorOn)
    if self.mad_mode_enabled and EventName.pedalPressed in events.events:
      events.events.remove(EventName.pedalPressed)

    if self.CC.longcontrol and self.CS.cruise_unavail or self.CP.radarDisablePossible and self.CS.brake_error:
      print("cruise error")
      events.add(EventName.brakeUnavailable)
    if self.CS.park_brake:
      events.add(EventName.parkBrake)
    #if abs(ret.steeringAngleDeg) > 90. and EventName.steerTempUnavailable not in events.events:
    #  events.add(EventName.steerTempUnavailable)
    if self.CC.low_speed_alert and not self.CS.mdps_bus and Params().get_bool("LowSpeedAlerts"):
      events.add(EventName.belowSteerSpeed)
    if self.CC.turning_indicator_alert:
      events.add(EventName.turningIndicatorOn)
    if self.mad_mode_enabled and EventName.pedalPressed in events.events:
      events.events.remove(EventName.pedalPressed)

    if Params().get_bool('spasEnabled'):
      if self.CS.mdps11_stat == 7 and not self.CC.turning_indicator_alert:
        if self.CS.mdps11_stat == 7 and self.CC.mdps11_stat_last == 7 and not self.CC.lkas_active and self.CC.spas_active: # We need to alert driver when SPAS abort or fail.
          events.add(EventName.steerSaturated) 

      if self.CC.override:
        events.add(EventName.buttonCancel)

      if self.CS.mdps11_stat == 6 or self.CS.mdps11_stat == 8:
        events.add(EventName.steerTempUnavailable)

    # scc smoother
    if self.CC.scc_smoother is not None:
      self.CC.scc_smoother.inject_events(events)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # scc smoother - hyundai only
  def apply(self, c, controls):
    ret = self.CC.update(c, c.enabled, self.CS, self.frame, c, c.actuators,
                               c.cruiseControl.cancel, c.hudControl.visualAlert, c.hudControl.leftLaneVisible,
                               c.hudControl.rightLaneVisible, c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart,
                               c.hudControl.setSpeed, c.hudControl.leadVisible, controls, c.hudControl.setSpeed)
    self.frame += 1
    return ret
