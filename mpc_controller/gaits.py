"""Gaits module."""
# Tested on Unitree A1
# Mostly those gaits are from Cheetah-Software

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from mpc_controller import gait_generator
import enum

class GaitType(enum.Enum):
  """The types of gait"""
  STAND = 0
  STATIC_WALK = 1
  AMBLE = 2
  TROT_WALK = 3
  TROT = 4
  TROT_RUN = 5
  PACE = 6
  BOUND = 7
  ROTARY_GALLOP = 8
  TRAVERSE_GALLOP = 9
  PRONK = 10
  CUSTOM = 11
  TRANSITION_TO_STAND = 12 

class Gait(object):
  NAME = GaitType.STAND
  SPEED = 0 # m/s
  STANCE_DURATION_SECONDS = [0.3]*4
  DUTY_FACTOR = [1.]*4
  INIT_PHASE_FULL_CYCLE = [0., 0., 0., 0.]
  INIT_LEG_STATE = (
      gait_generator.LegState.STANCE,  # FR frontRight # in MIT:  FL
      gait_generator.LegState.STANCE,  # FL frontLeft             FR
      gait_generator.LegState.STANCE,  # RR rearRight             RR
      gait_generator.LegState.STANCE,  # RL rearLeft              RL
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.1 # 0.1 DEFAULT

class STAND(Gait):
  NAME = GaitType.STAND
  SPEED = 0 # m/s
  STANCE_DURATION_SECONDS = [0.3]*4
  DUTY_FACTOR = [1.]*4
  INIT_PHASE_FULL_CYCLE = [0., 0., 0., 0.]
  INIT_LEG_STATE = (
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

# # were in this repo: Tripod
# class STATIC_WALK(Gait):
#   NAME = GaitType.STATIC_WALK
#   SPEED = 0.5 # m/s
#   STANCE_DURATION_SECONDS = [0.8]*4
#   DUTY_FACTOR = [0.8]*4
#   INIT_PHASE_FULL_CYCLE = [0., 0.25, 0.5, 0.]
#   INIT_LEG_STATE = (
#       gait_generator.LegState.STANCE,
#       gait_generator.LegState.STANCE,
#       gait_generator.LegState.STANCE,
#       gait_generator.LegState.SWING,
#   )
#   CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

class STATIC_WALK(Gait):
  NAME = GaitType.STATIC_WALK
  SPEED = 0.3 # m/s
  STANCE_DURATION_SECONDS = [0.5]*4 # in MIT: 1
  DUTY_FACTOR = [0.8]*4 
  INIT_PHASE_FULL_CYCLE = [0., 0.25, 0.75, 0.5] # in MIT: [0.25, 0.0, 0.75, 0.5]
  INIT_LEG_STATE = (
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

class AMBLE(Gait):
  NAME = GaitType.AMBLE
  SPEED = 0.4 # m/s
  STANCE_DURATION_SECONDS = [0.3125]*4
  DUTY_FACTOR = [0.6250]*4 # in MIT: 0.6250
  INIT_PHASE_FULL_CYCLE = [0.5, 0.0, 0.75, 0.5] # in MIT: [0.0, 0.5, 0.25, 0.75]
  INIT_LEG_STATE = (
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

# class TROT_WALK(Gait):
#   NAME = GaitType.TROT_WALK
#   SPEED = 0.6 # m/s
#   STANCE_DURATION_SECONDS = [0.3]*4
#   DUTY_FACTOR = [0.6]*4
#   INIT_PHASE_FULL_CYCLE = [0.9, 0, 0, 0.9]

#   INIT_LEG_STATE = (
#       gait_generator.LegState.SWING,
#       gait_generator.LegState.STANCE,
#       gait_generator.LegState.STANCE,
#       gait_generator.LegState.SWING,
#   )
#   CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

class TROT_WALK(Gait):
  NAME = GaitType.TROT_WALK
  SPEED = 0.6 # m/s
  STANCE_DURATION_SECONDS = [0.3]*4
  DUTY_FACTOR = [0.6]*4
  INIT_PHASE_FULL_CYCLE = [0.0, 0.5, 0.5, 0.0]
  INIT_LEG_STATE = (
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

class TROT(Gait):
  NAME = GaitType.TROT
  SPEED = 1 # m/s
  STANCE_DURATION_SECONDS = [0.25]*4
  DUTY_FACTOR = [0.5]*4
  INIT_PHASE_FULL_CYCLE = [0.0, 0.5, 0.5, 0.0]
  INIT_LEG_STATE = (
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

class TROT_RUN(Gait):
  NAME = GaitType.TROT_RUN
  SPEED = 1.5 # m/s
  STANCE_DURATION_SECONDS = [0.16]*4
  DUTY_FACTOR = [0.3]*4 # in MIT: 0.4
  INIT_PHASE_FULL_CYCLE = [0.0, 0.5, 0.5, 0.0]
  INIT_LEG_STATE = (
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

# class TROT_RUN(Gait):
#   NAME = GaitType.TROT
#   _SPEED = 1 # m/s
#   _STANCE_DURATION_SECONDS = [0.4*0.4]*4 # full cycle 0.4 s
#   _DUTY_FACTOR = [0.4]*4
#   _INIT_PHASE_FULL_CYCLE = [0.99, 0, 0, 0.99]

#   _INIT_LEG_STATE = (
#       gait_generator.LegState.SWING,
#       gait_generator.LegState.STANCE,
#       gait_generator.LegState.STANCE,
#       gait_generator.LegState.SWING,
#   )
#   _CONTACT_DETECTION_PHASE_THRESHOLD = 0.01

class PACE(Gait): # ??
  NAME = GaitType.PACE
  SPEED = 1.5 # m/s
  STANCE_DURATION_SECONDS = [0.175]*4
  DUTY_FACTOR = [0.5]*4
  INIT_PHASE_FULL_CYCLE = [0.0, 0.5, 0.0, 0.5]
  INIT_LEG_STATE = (
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

# class BOUND(Gait):
#   NAME = GaitType.BOUND
#   SPEED = 1.5 # m/s
#   STANCE_DURATION_SECONDS = [0.16]*4
#   DUTY_FACTOR = [0.4]*4
#   INIT_PHASE_FULL_CYCLE = [0.0, 0.0, 0.5, 0.5] # in MIT: [0.0, 0.0, 0.5, 0.5]
#   INIT_LEG_STATE = (
#       gait_generator.LegState.STANCE,
#       gait_generator.LegState.STANCE,
#       gait_generator.LegState.STANCE,
#       gait_generator.LegState.STANCE,
#   )
#   CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

class BOUND(Gait): # ??
  NAME = GaitType.BOUND
  SPEED = 1 # m/s
  STANCE_DURATION_SECONDS = [0.4*0.4]*4 # full cycle 0.4 s
  DUTY_FACTOR = [0.4]*4
  INIT_PHASE_FULL_CYCLE = [0.99, 0.5, 0.5, 0.99]
  INIT_LEG_STATE = (
      gait_generator.LegState.SWING,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.SWING,
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.01

class ROTARY_GALLOP(Gait):
  NAME = GaitType.ROTARY_GALLOP
  SPEED = 1.5 # m/s
  STANCE_DURATION_SECONDS = [0.08]*4
  DUTY_FACTOR = [0.2]*4
  INIT_PHASE_FULL_CYCLE = [0.8571, 0.0, 0.3571, 0.5] # in MIT: [0.0, 0.8571, 0.3571, 0.5]
  INIT_LEG_STATE = (
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

class TRAVERSE_GALLOP(Gait):
  NAME = GaitType.TRAVERSE_GALLOP
  SPEED = 1.5 # m/s
  STANCE_DURATION_SECONDS = [0.1]*4
  DUTY_FACTOR = [0.2]*4
  INIT_PHASE_FULL_CYCLE = [0.8571, 0.0, 0.3571, 0.5] # in MIT: [0.0, 0.8571, 0.3571, 0.5]
  INIT_LEG_STATE = (
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.1

class PRONK(Gait): # COT = 1.16
  NAME = GaitType.PRONK
  SPEED = 1.0 # m/s
  STANCE_DURATION_SECONDS = [0.25]*4
  DUTY_FACTOR = [0.5]*4
  INIT_PHASE_FULL_CYCLE = [0.0, 0.0, 0.0, 0.0]
  INIT_LEG_STATE = (
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
      gait_generator.LegState.STANCE,
  )
  CONTACT_DETECTION_PHASE_THRESHOLD = 0.1