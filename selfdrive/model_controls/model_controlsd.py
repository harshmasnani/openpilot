#!/usr/bin/env python3
import os
import math
from cereal import car, log
from common.numpy_fast import clip
from common.realtime import sec_since_boot, config_realtime_process, Priority, Ratekeeper, DT_CTRL
from common.params import Params, put_nonblocking
import cereal.messaging as messaging
from selfdrive.config import Conversions as CV
from selfdrive.swaglog import cloudlog
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from selfdrive.controls.lib.lane_planner import CAMERA_OFFSET
from selfdrive.controls.lib.drive_helpers import update_v_cruise, initialize_v_cruise
from selfdrive.controls.lib.drive_helpers import get_lag_adjusted_curvature
from selfdrive.controls.lib.longcontrol import LongControl, STARTING_TARGET_SPEED
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from selfdrive.controls.lib.events import Events, ET
from selfdrive.controls.lib.alertmanager import AlertManager
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.longitudinal_planner import LON_MPC_STEP
from selfdrive.locationd.calibrationd import Calibration

import numpy as np
from random import random
MODEL_MIN_SPEED = 50 * CV.KPH_TO_MS # minimum speed to use model

STEER_FACTOR = 300/2 # We can add 4 out of 300 cNm per one output frame which runs at 50 Hz. So the factor is 300 due to the unit but /2 because 20ms instead of 10 
STEER_DELTA_UP = 4/STEER_FACTOR
STEER_DELTA_DOWN = 10/STEER_FACTOR

def apply_std_steer_torque_limits(apply_torque, apply_torque_last):
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - STEER_DELTA_DOWN, -STEER_DELTA_UP),
                        apply_torque_last + STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - STEER_DELTA_UP,
                        min(apply_torque_last + STEER_DELTA_DOWN, STEER_DELTA_UP))

  return int(round(float(apply_torque)))

#steering angle, speed, torque, IMU_linear, IMU_angular
norm = (16.049999237060547, 35.887664794921875, 1.0, [19.161849975585938, 8.428146362304688, 11.452590942382812], [0.0880889892578125, 0.8147430419921875, 0.1421051025390625])
groups = [5, 15, 4, 20, 20]

#Data storage timings
prev_data = 300
fwd_data = [20, 40, 60, 80, 100]

## Give car params for steering angle calculations because we are lazy
class CivicParams:
  MASS = 1326. + 70
  WHEELBASE = 2.70
  CENTER_TO_FRONT = WHEELBASE * 0.4
  CENTER_TO_REAR = WHEELBASE - CENTER_TO_FRONT
  ROTATIONAL_INERTIA = 2500
  TIRE_STIFFNESS_FRONT = 192150
  TIRE_STIFFNESS_REAR = 202500
def scale_rot_inertia(mass, wheelbase):
  return CivicParams.ROTATIONAL_INERTIA * mass * wheelbase ** 2 / (CivicParams.MASS * CivicParams.WHEELBASE ** 2)
def scale_tire_stiffness(mass, wheelbase, center_to_front, tire_stiffness_factor=1.0):
  center_to_rear = wheelbase - center_to_front
  tire_stiffness_front = (CivicParams.TIRE_STIFFNESS_FRONT * tire_stiffness_factor) * mass / CivicParams.MASS * \
                         (center_to_rear / wheelbase) / (CivicParams.CENTER_TO_REAR / CivicParams.WHEELBASE)

  tire_stiffness_rear = (CivicParams.TIRE_STIFFNESS_REAR * tire_stiffness_factor) * mass / CivicParams.MASS * \
                        (center_to_front / wheelbase) / (CivicParams.CENTER_TO_FRONT / CivicParams.WHEELBASE)

  return tire_stiffness_front, tire_stiffness_rear
def carParams():
  ret = car.CarParams.new_message()
  ret.carFingerprint = "VW or something"

  # standard ALC params
  ret.steerControlType = car.CarParams.SteerControlType.torque
  ret.steerMaxBP = [0.]
  ret.steerMaxV = [1.]
  ret.minSteerSpeed = 0.

  # stock ACC by default
  ret.enableCruise = True
  ret.minEnableSpeed = -1.  # enable is done by stock ACC, so ignore this
  ret.steerRatioRear = 0.  # no rear steering, at least on the listed cars aboveA
  ret.gasMaxBP = [0.]
  ret.gasMaxV = [.5]  # half max brake
  ret.brakeMaxBP = [0.]
  ret.brakeMaxV = [1.]
  ret.openpilotLongitudinalControl = False
  ret.startAccel = 0.0
  ret.minSpeedCan = 0.3
  ret.stoppingBrakeRate = 0.2 # brake_travel/s while trying to stop
  ret.startingBrakeRate = 0.8 # brake_travel/s while releasing on restart
  ret.stoppingControl = True
  ret.longitudinalTuning.deadzoneBP = [0.]
  ret.longitudinalTuning.deadzoneV = [0.]
  ret.longitudinalTuning.kpBP = [0.]
  ret.longitudinalTuning.kpV = [1.]
  ret.longitudinalTuning.kiBP = [0.]
  ret.longitudinalTuning.kiV = [1.]

  # VW port is a community feature, since we don't own one to test
  ret.communityFeature = True
  ret.carName = "volkswagen"
  ret.radarOffCan = True

  # Set common PQ35/PQ46/NMS parameters that will apply globally
  ret.safetyModel = car.CarParams.SafetyModel.volkswagenPq
  ret.steerActuatorDelay = 0

  # Global tuning defaults, can be overridden per-vehicle

  ret.steerRateCost = 1.0
  ret.steerLimitTimer = 0.4
  ret.steerRatio = 18.78  
  tire_stiffness_factor = 1 
  ret.lateralTuning.pid.kf = 0.00006
  
  ret.lateralTuning.pid.kpBP = [0., 105*CV.KPH_TO_MS]
  ret.lateralTuning.pid.kiBP = [0., 105*CV.KPH_TO_MS]
  ret.lateralTuning.pid.kdBP = [0., 105*CV.KPH_TO_MS]

  ret.lateralTuning.pid.kpV = [0.15, 0.19]
  ret.lateralTuning.pid.kiV = [0.05,  0.08]
  ret.lateralTuning.pid.kdV = [0.0003,  0.0012]

  ret.mass = 1617+100+70
  ret.wheelbase = 2.68
  ret.minSteerSpeed = 20 * CV.KPH_TO_MS

  ret.centerToFront = ret.wheelbase * 0.4

  ret.enableCamera = True  # Stock camera detection doesn't apply to VW

  # TODO: get actual value, for now starting with reasonable value for
  # civic and scaling by mass and wheelbase
  ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

  # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
  # mass and CG position, so all cars will have approximately similar dyn behaviors
  ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                        tire_stiffness_factor=tire_stiffness_factor)

  return ret
## End of copy pasted code
"""
  Generated using Konverter: https://github.com/ShaneSmiskol/Konverter
"""

import numpy as np

wb = np.load('/home/gregor/openpilot/model_s_keras_weights.npz', allow_pickle=True)
w, b = wb['wb']

def model(x):
  x = np.array(x, dtype=np.float32)
  l0 = np.dot(x, w[0]) + b[0]
  l0 = np.maximum(0, l0)
  l1 = np.dot(l0, w[1]) + b[1]
  l1 = np.maximum(0, l1)
  l2 = np.dot(l1, w[2]) + b[2]
  l2 = np.tanh(l2)
  return l2

def get_model_input(phi, v, M, IMU_v, IMU_alpha, M_fut):
  phi_out = phi.reshape(-1, groups[0]).mean(axis=1)/norm[0]
  v_out = v.reshape(-1, groups[1]).mean(axis=1)/norm[1]
  M_out_l = M.reshape(-1, groups[2]).mean(axis=1)/norm[2]
  M_out_r = M_fut.reshape(-1, groups[2]).mean(axis=1)/norm[2]
  M_out = np.concatenate([M_out_l, M_out_r])
  IMU_v_out = [IMU_v[i].reshape(-1, groups[3]).mean(axis=1)/norm[3][i] for i in range(3)]
  IMU_alpha_out = [IMU_alpha[i].reshape(-1, groups[4]).mean(axis=1)/norm[4][i] for i in range(3)]

  return np.concatenate([phi_out, v_out, M_out, IMU_v_out[0], IMU_v_out[1], IMU_v_out[2], IMU_alpha_out[0], IMU_alpha_out[1], IMU_alpha_out[2]])

class ModelControls:
  def __init__(self):
    self.frame = 0
    config_realtime_process(4, Priority.CTRL_HIGH)

    # Setup sockets
    self.pm = messaging.PubMaster(['modelTorque'])

    self.sm = messaging.SubMaster(['lateralPlan', 'controlsState', 'carState', 'sensorEvents'])

    self.rk = Ratekeeper(50, print_delay_threshold=None)
    
    self.phi = np.zeros((prev_data,))
    self.v = np.zeros((prev_data,))
    self.M = np.zeros((prev_data,))
    self.IMU_v = [np.zeros((prev_data,)) for _ in range(3)]
    self.IMU_alpha = [np.zeros((prev_data,)) for _ in range(3)]

    self.active = False #Ensure good init on change
    self.outputTorque = 0 #TODO refactor this variable
    ##DEBUG##
    self.vcount = 0
    self.acount = 0
    self.count = 0
    ##/DEBUG##

    self.memIMU_v = [0 for _ in range(3)]
    self.memIMU_alpha = [0 for _ in range(3)]

    self.CP = carParams()
    self.VM = VehicleModel(self.CP)
    self.angleOffset = -2.09

  def step(self):
    self.frame +=1
    self.parse_logs()
    self.predict_torque()
    self.publish_logs()

  def parse_logs(self):
    self.sm.update(0)
    self.count +=1
    CS = self.sm["carState"]
    phi = CS.steeringAngleDeg
    v = CS.vEgo
    M = 0
    try:
      M = self.sm["controlsState"].lateralControlState.pidState.output
    except Exception as e:
      # print(e)
      pass
    if not self.active and v > MODEL_MIN_SPEED:
      self.active = True
    if self.active and v < MODEL_MIN_SPEED - 5:
      self.active = False
    self.outputTorque = M

    #Parse IMU data
    se = self.sm["sensorEvents"]
    # iz neznanih razlogov je v logih nekako vec sensor eventov kot jih imam tukaj... Za silo bomo pac malo cachali te zadeve
    for s in se: # s je nek sensorEvent, tole mi načeloma ni najbolj všeč za parsanje :)
      if s.type == 1:
        # print("v")
        self.vcount +=1
        self.memIMU_v = s.acceleration.v
      elif s.type ==4:
        # print("alpha")
        self.acount +=1
        self.memIMU_alpha = s.gyro.v
    
    # print(phi, v, M, self.memIMU_v, self.memIMU_alpha)
    # print(self.count, self.vcount, self.acount)
    
    # spravi phi, v, M v numPy array
    self.phi = np.roll(self.phi, -1)
    self.phi[-1]=phi
    self.v = np.roll(self.v, -1)
    self.v[-1]=v
    self.M = np.roll(self.M, -1)
    self.M[-1]=M
    for i in range(3):
      self.IMU_v[i] = np.roll(self.IMU_v[i], -1)
      self.IMU_v[i][-1] = self.memIMU_v[i]
      self.IMU_alpha[i] = np.roll(self.IMU_alpha[i], -1)
      self.IMU_alpha[i][-1] = self.memIMU_alpha[i]
    
  def predict_torque(self):
    lat_plan = self.sm["lateralPlan"]
    
    # print(lat_plan.to_dict())
    # Compute desired steering angle profile 
    steering_angle = [0]*len(fwd_data)
    for i in range(len(fwd_data)):
      self.CP.steerActuatorDelay = -0.2 + fwd_data[i]/100
      desired_curvature, _ = get_lag_adjusted_curvature(self.CP, self.v[-1], lat_plan.psis, lat_plan.curvatures, lat_plan.curvatureRates)

      steering_angle[i] = math.degrees(self.VM.get_steer_from_curvature(-desired_curvature, self.v[-1]))
    # print(steering_angle)
    # bestTorque = self.outputTorque
    M_fut = np.ones((fwd_data[-1],))*self.outputTorque

    # for _ in range(3):
    model_input = get_model_input(self.phi, self.v, self.M, self.IMU_v, self.IMU_alpha, M_fut)
    predicted_angle = model(model_input)
    #L1 je negativen ce moramo precej bolj zaviti
    L1 = sum([predicted_angle[i]-steering_angle[i] for i in range(len(fwd_data))])
    # L2 = sum([(predicted_angle[i]-steering_angle[i])**2 for i in range(len(fwd_data))])

    #   tuning_factor = -1/20
    #   bestTorque += L1*tuning_factor
    #   bestTorque = clip(bestTorque, -1, 1)

    #   prevTorque = self.outputTorque

    #   for i in range(fwd_data[-1]):
    #     M_fut[i] = apply_std_steer_torque_limits(bestTorque, prevTorque)
    #     prevTorque = M_fut[i]
    #   if self.frame % 10 == 0:
    #     print("L1:",L1,"L2:",L2, "M:", bestTorque)
    # if self.frame % 10 == 0:
    #   print(bestTorque)
    #   print(self.outputTorque, M_fut[0])
    
    #Poenostavljena logika: Ali je trenutni navor dovolj velik?
    if L1 > 0:
      self.outputTorque = -1
    else:
      self.outputTorque = 1

  def publish_logs(self):
    model_send = messaging.new_message('modelTorque')
    model_send.valid = True
    # if self.frame % 10 == 0:
    #   print(self.active, self.outputTorque)
    model_send.modelTorque.active = self.active
    model_send.modelTorque.outputTorque = self.outputTorque
    self.pm.send('modelTorque', model_send)

  def model_thread(self):
    while True:
      self.step()
      self.rk.keep_time()

if __name__ == "__main__":
  controls = ModelControls()
  controls.model_thread()
