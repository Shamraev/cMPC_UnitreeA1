"""Multiple gait pattern planning module."""

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import logging
import math

import numpy as np
from typing import Any, Sequence
from mpc_controller import openloop_gait_generator
from mpc_controller import gait_generator
from mpc_controller import gaits


class MultipleGaitGenerator(openloop_gait_generator.OpenloopGaitGenerator):
  """Generates openloop multiple gaits for quadruped robots.

  """
  def __init__(
      self,
      robot: Any,
      gait: gaits.Gait = gaits.TROT_WALK
  ):
    """Initializes the class.

    Args:
      robot: A quadruped robot that at least implements the GetFootContacts API
        and num_legs property.
      gait: A desired gait.
    """

    super().__init__(
        robot,
        stance_duration=gait.STANCE_DURATION_SECONDS,
        duty_factor=gait.DUTY_FACTOR,
        initial_leg_phase=gait.INIT_PHASE_FULL_CYCLE,
        initial_leg_state=gait.INIT_LEG_STATE,
        contact_detection_phase_threshold = gait.CONTACT_DETECTION_PHASE_THRESHOLD)
    self._gait = gait      # ??
    self._next_gait = gait # ??

  @property
  def gait(self) -> gaits.Gait:
    return self._gait

  def update_gait(self, gait):
    #self._next_gait = gait 
    self._gait = gait 

    self._stance_duration=gait.STANCE_DURATION_SECONDS,
    self._duty_factor=gait.DUTY_FACTOR,
    self._initial_leg_phase=gait.INIT_PHASE_FULL_CYCLE,
    self._initial_leg_state=gait.INIT_LEG_STATE,
    self._contact_detection_phase_threshold = gait.CONTACT_DETECTION_PHASE_THRESHOLD
    # ??
    self._desired_leg_state = gait.INIT_LEG_STATE

