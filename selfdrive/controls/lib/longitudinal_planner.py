#!/usr/bin/env python3
import math
import numpy as np
from common.numpy_fast import interp

import cereal.messaging as messaging
from cereal import log
from common.realtime import DT_MDL
from common.realtime import sec_since_boot
from selfdrive.modeld.constants import T_IDXS
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.fcw import FCWChecker
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.lead_mpc import LeadMpc
from selfdrive.controls.lib.long_mpc import LongitudinalMpc
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N
from selfdrive.controls.lib.vision_turn_controller import VisionTurnController
from selfdrive.controls.lib.speed_limit_controller import SpeedLimitController, SpeedLimitResolver
from selfdrive.controls.lib.turn_speed_controller import TurnSpeedController
from selfdrive.controls.lib.events import Events
from selfdrive.swaglog import cloudlog
from common.params import Params

LON_MPC_STEP = 0.2  # first step is 0.2s
AWARENESS_DECEL = -0.2     # car smoothly decel at .2m/s^2 when user is distracted
A_CRUISE_MIN = -1.2
A_CRUISE_MAX_VALS = [1.2, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 15., 25., 40.]

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego**2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max**2 - a_y**2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class Planner():
  def __init__(self, CP):
    self.CP = CP
    self.mpcs = {}
    self.mpcs['lead0'] = LeadMpc(0)
    self.mpcs['lead1'] = LeadMpc(1)
    self.mpcs['cruise'] = LongitudinalMpc()

    self.fcw = False
    self.fcw_checker = FCWChecker()

    self.v_desired = 0.0
    self.a_desired = 0.0
    self.longitudinalPlanSource = 'cruise'
    self.alpha = np.exp(-DT_MDL/2.0)
    self.lead_0 = log.ModelDataV2.LeadDataV3.new_message()
    self.lead_1 = log.ModelDataV2.LeadDataV3.new_message()

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)

    self.vision_turn_controller = VisionTurnController(CP)
    self.speed_limit_controller = SpeedLimitController()
    self.events = Events()
    self.turn_speed_controller = TurnSpeedController()
    self.use_cluster_speed = Params().get_bool('UseClusterSpeed')
    self.long_control_enabled = Params().get_bool('LongControlEnabled')

  def update(self, sm, CP):
    cur_time = sec_since_boot()
    v_ego = sm['carState'].vEgo
    a_ego = sm['carState'].aEgo

    v_cruise_kph = sm['controlsState'].vCruise
    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    if not self.use_cluster_speed or self.long_control_enabled:
      cluSpeedMs = sm['controlsState'].cluSpeedMs
      if v_ego > 3. and cluSpeedMs > 3.:
        v_cruise *= v_ego / cluSpeedMs
        v_cruise = int(v_cruise * CV.MS_TO_KPH) * CV.KPH_TO_MS

    long_control_state = sm['controlsState'].longControlState
    force_slow_decel = sm['controlsState'].forceDecel

    self.lead_0 = sm['radarState'].leadOne
    self.lead_1 = sm['radarState'].leadTwo

    enabled = (long_control_state == LongCtrlState.pid) or (long_control_state == LongCtrlState.stopping)
    if not enabled or sm['carState'].gasPressed:
      self.v_desired = v_ego
      self.a_desired = a_ego

    # Prevent divergence, smooth in current v_ego
    self.v_desired = self.alpha * self.v_desired + (1 - self.alpha) * v_ego
    self.v_desired = max(0.0, self.v_desired)

    accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
    accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    if force_slow_decel:
      # if required so, force a smooth deceleration
      accel_limits_turns[1] = min(accel_limits_turns[1], AWARENESS_DECEL)
      accel_limits_turns[0] = min(accel_limits_turns[0], accel_limits_turns[1])

    cruise_source, v_cruise, accel_limits_turns = self.cruise_solution(enabled, self.v_desired, self.a_desired,
                                                                       accel_limits_turns, v_cruise, sm)

    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired)
    self.mpcs['cruise'].set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])

    next_a = np.inf
    for key in self.mpcs:
      self.mpcs[key].set_cur_state(self.v_desired, self.a_desired)
      self.mpcs[key].update(sm['carState'], sm['radarState'], v_cruise)
      if self.mpcs[key].status and self.mpcs[key].a_solution[5] < next_a:
        self.longitudinalPlanSource = key if key != 'cruise' else cruise_source
        self.v_desired_trajectory = self.mpcs[key].v_solution[:CONTROL_N]
        self.a_desired_trajectory = self.mpcs[key].a_solution[:CONTROL_N]
        self.j_desired_trajectory = self.mpcs[key].j_solution[:CONTROL_N]
        next_a = self.mpcs[key].a_solution[5]

    # determine fcw
    if self.mpcs['lead0'].new_lead:
      self.fcw_checker.reset_lead(cur_time)
    blinkers = sm['carState'].leftBlinker or sm['carState'].rightBlinker
    self.fcw = self.fcw_checker.update(self.mpcs['lead0'].mpc_solution, cur_time,
                                       sm['controlsState'].active,
                                       v_ego, sm['carState'].aEgo,
                                       self.lead_1.dRel, self.lead_1.vLead, self.lead_1.aLeadK,
                                       self.lead_1.yRel, self.lead_1.vLat,
                                       self.lead_1.fcw, blinkers) and not sm['carState'].brakePressed
    if self.fcw:
      cloudlog.info("FCW triggered %s", self.fcw_checker.counters)

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired = self.v_desired + DT_MDL * (self.a_desired + a_prev)/2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.speeds = [float(x) for x in self.v_desired_trajectory]
    longitudinalPlan.accels = [float(x) for x in self.a_desired_trajectory]
    longitudinalPlan.jerks = [float(x) for x in self.j_desired_trajectory]

    longitudinalPlan.hasLead = self.mpcs['lead0'].status
    longitudinalPlan.longitudinalPlanSource = self.longitudinalPlanSource
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.visionTurnControllerState = self.vision_turn_controller.state
    longitudinalPlan.visionTurnSpeed = self.vision_turn_controller.v_turn

    longitudinalPlan.speedLimitControlState = self.speed_limit_controller.state
    longitudinalPlan.speedLimit = float(self.speed_limit_controller.speed_limit)
    longitudinalPlan.speedLimitOffset = float(self.speed_limit_controller.speed_limit_offset)
    longitudinalPlan.distToSpeedLimit = float(self.speed_limit_controller.distance)
    longitudinalPlan.isMapSpeedLimit = bool(self.speed_limit_controller.source == SpeedLimitResolver.Source.map_data)
    longitudinalPlan.eventsDEPRECATED = self.events.to_msg()

    longitudinalPlan.turnSpeedControlState = self.turn_speed_controller.state
    longitudinalPlan.turnSpeed = float(self.turn_speed_controller.speed_limit)
    longitudinalPlan.distToTurn = float(self.turn_speed_controller.distance)
    longitudinalPlan.turnSign = int(self.turn_speed_controller.turn_sign)

    pm.send('longitudinalPlan', plan_send)

  def cruise_solution(self, enabled, v_desired, a_desired, acc_limits, v_cruise, sm):
    # Update controllers
    self.events = Events()
    self.vision_turn_controller.update(enabled, v_desired, a_desired, v_cruise, acc_limits, sm)
    self.speed_limit_controller.update(enabled, v_desired, sm, v_cruise, acc_limits, self.events)
    self.turn_speed_controller.update(enabled, v_desired, sm, acc_limits)

    # Pick slowest solution
    v_solutions = {'cruise': v_cruise}
    limits_solutions = {'cruise': acc_limits}

    if self.vision_turn_controller.is_active:
      v_solutions['turn'] = self.vision_turn_controller.v_turn
      limits_solutions['turn'] = self.vision_turn_controller.acc_limits

    if self.speed_limit_controller.is_active:
      v_solutions['limit'] = self.speed_limit_controller.v_limit
      limits_solutions['limit'] = self.speed_limit_controller.acc_limits

    if self.turn_speed_controller.is_active:
      v_solutions['turnlimit'] = self.turn_speed_controller.v_turn_limit
      limits_solutions['turnlimit'] = self.turn_speed_controller.acc_limits

    source = min(v_solutions, key=v_solutions.get)

    return source, v_solutions[source], limits_solutions[source]