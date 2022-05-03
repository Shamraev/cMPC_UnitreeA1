"""Example of whole body controller on A1 robot."""
use_cMPC = True
print_COT = True

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from absl import app
from absl import flags
from absl import logging
from datetime import datetime
import numpy as np
import os
import scipy.interpolate
import time

import pybullet_data
from pybullet_utils import bullet_client
import pybullet  # pytype:disable=import-error

from mpc_controller import logger
from mpc_controller import com_velocity_estimator
from mpc_controller import multiple_gait_generator
from mpc_controller import gaits
from mpc_controller import locomotion_controller
from mpc_controller import raibert_swing_leg_controller
if use_cMPC:
  from mpc_controller import torque_stance_leg_controller
  import mpc_osqp
else: from mpc_controller import torque_stance_leg_controller_quadprog as torque_stance_leg_controller

from motion_imitation.robots import a1
from motion_imitation.robots import robot_config
from motion_imitation.robots.gamepad import gamepad_reader

flags.DEFINE_bool("record_video", False, "Record video")
flags.DEFINE_string("logdir", None, "where to log trajectories.")
flags.DEFINE_bool("use_gamepad", False,
                  "whether to use gamepad to provide control input.")
flags.DEFINE_bool("use_real_robot", False,
                  "whether to use real robot or simulation")
flags.DEFINE_bool("show_gui", True, "whether to show GUI.")
flags.DEFINE_float("max_time_secs", 5., "maximum time to run the robot.")
FLAGS = flags.FLAGS

_NUM_SIMULATION_ITERATION_STEPS = 300
_MAX_TIME_SECONDS = 30.
time_step = 0.002 # 0.025

def _generate_example_linear_angular_speed(t):
  """Creates an example speed profile based on time for demo purpose."""
  vx = 0.6
  vy = 0.2
  wz = 0.8

  time_points = (0, 5, 10, 15, 20, 25, 30)
  speed_points = ((0, 0, 0, 0), (0, 0, 0, wz), (vx, 0, 0, 0), (0, 0, 0, -wz),
                  (0, -vy, 0, 0), (0, 0, 0, 0), (0, 0, 0, wz))

  speed = scipy.interpolate.interp1d(time_points,
                                     speed_points,
                                     kind="previous",
                                     fill_value="extrapolate",
                                     axis=0)(t)

  return speed[0:3], speed[3], False

def _generate_example_constant_linear_speed(t):
  vx = 1.2
  vy = 0
  vz = 0
  wz = 0
  return [vx,vy,vz],wz,False

def _generate_example_linear_speed(t):
  vx = 1 #gait_generator._gait.SPEED

  # walk trot_run 
  time_points = (0, 3)
  speed_points = ((vx, 0, 0, 0), (vx, 0, 0, 0))

  speed = scipy.interpolate.interp1d(time_points,
                                     speed_points,
                                     kind="previous",
                                     fill_value="extrapolate",
                                     axis=0)(t)
  return speed[0:3], speed[3], False

def _setup_controller(robot):
  """Demonstrates how to create a locomotion controller."""
  desired_speed = (0, 0)
  desired_twisting_speed = 0

  gait_generator = multiple_gait_generator.MultipleGaitGenerator(
      robot,
      gaits.PRONK)
  window_size = 20 if not FLAGS.use_real_robot else 1
  state_estimator = com_velocity_estimator.COMVelocityEstimator(
      robot, window_size=window_size)
  sw_controller = raibert_swing_leg_controller.RaibertSwingLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_height=robot.MPC_BODY_HEIGHT,
      foot_clearance=0.01)

  fullCycle = gait_generator.gait.STANCE_DURATION_SECONDS[0]/gait_generator.gait.DUTY_FACTOR[0] # fullCycle*DUTY_FACTOR = stance_duration --> fullCycle=stance_duration/DUTY_FACTOR
  time_steps = int(fullCycle/time_step)
  if use_cMPC:
    st_controller = torque_stance_leg_controller.TorqueStanceLegController(
        robot,
        gait_generator,
        state_estimator,
        desired_speed=desired_speed,
        desired_twisting_speed=desired_twisting_speed,
        desired_body_height=robot.MPC_BODY_HEIGHT,    
        qp_solver = mpc_osqp.QPOASES, #or mpc_osqp.OSQP
        body_mass = robot.MPC_BODY_MASS,
        body_inertia = robot.MPC_BODY_INERTIA,
        PLANNING_HORIZON_STEPS = time_steps, # 10
        PLANNING_TIMESTEP = time_step    # 0.025
        )
  else:
    st_controller = torque_stance_leg_controller.TorqueStanceLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_body_height=robot.MPC_BODY_HEIGHT
      )

  controller = locomotion_controller.LocomotionController(
      robot=robot,
      gait_generator=gait_generator,
      state_estimator=state_estimator,
      swing_leg_controller=sw_controller,
      stance_leg_controller=st_controller,
      clock=robot.GetTimeSinceReset)
  return controller


def _update_controller_params(controller, lin_speed, ang_speed):
  # if lin_speed[0]>0.8:
  #   controller.gait_generator.update_gait(gaits.TROT_RUN)
  controller.swing_leg_controller.desired_speed = lin_speed
  controller.swing_leg_controller.desired_twisting_speed = ang_speed
  controller.stance_leg_controller.desired_speed = lin_speed
  controller.stance_leg_controller.desired_twisting_speed = ang_speed

def _MoveCameraAlongRobot(p, robot):
  # for moving the camera with the robot
  cubePos = robot.GetBasePosition()
  cubePos = (cubePos[0], cubePos[1], robot.MPC_BODY_HEIGHT)
  croll, cpitch, cyaw = robot.GetBaseRollPitchYaw()
  cpitch = -np.pi/8*0
  cyaw = cyaw+np.pi/6*0+np.pi/2*0    
  cdist = 0.8
  p.resetDebugVisualizerCamera(cameraDistance=cdist, cameraYaw=np.rad2deg(cyaw), cameraPitch=np.rad2deg(cpitch), cameraTargetPosition=cubePos)

def _SetCamera(p, robot):
  _MoveCameraAlongRobot(p, robot)


def main(argv):
  """Runs the locomotion controller example."""
  del argv # unused

  # Construct simulator
  if FLAGS.record_video or True:
    p = pybullet
    p.connect(p.GUI, options="--width=1280 --height=720 --mp4=\"test.mp4\" --mp4fps=24")
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1) # it doesn't work on my computer

    # another variant:
    #p = pybullet
    #p.connect(p.GUI)
    #p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "test.mp4")
  elif FLAGS.show_gui and not FLAGS.use_real_robot:
    p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
  else:
    p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)
  p.setPhysicsEngineParameter(numSolverIterations=30)
  p.setTimeStep(0.001)
  p.setGravity(0, 0, -9.8)
  p.setPhysicsEngineParameter(enableConeFriction=0)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.loadURDF("plane.urdf")

  # Construct robot class:
  if FLAGS.use_real_robot:
    from motion_imitation.robots import a1_robot
    robot = a1_robot.A1Robot(
        pybullet_client=p,
        motor_control_mode=robot_config.MotorControlMode.HYBRID,
        enable_action_interpolation=False,
        time_step=time_step, # 0.002
        action_repeat=1)
  else:
    robot = a1.A1(p,
                  motor_control_mode=robot_config.MotorControlMode.HYBRID,
                  enable_action_interpolation=False,
                  reset_time=2,
                  time_step=time_step, # 0.002
                  action_repeat=1)

  controller = _setup_controller(robot)
  _logger = logger.Logger(robot, print_COT)

  controller.reset()
  if FLAGS.use_gamepad:
    gamepad = gamepad_reader.Gamepad()
    command_function = gamepad.get_command
  else:
    #command_function = _generate_example_linear_angular_speed
    command_function = _generate_example_linear_speed
    #command_function = _generate_example_constant_linear_speed

  if FLAGS.logdir:
    logdir = os.path.join(FLAGS.logdir,
                          datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
    os.makedirs(logdir)

  start_time = robot.GetTimeSinceReset()
  current_time = start_time
  _SetCamera(p, robot)
  while current_time - start_time < FLAGS.max_time_secs:
    _MoveCameraAlongRobot(p,robot)
    
    #time.sleep(0.0008) #on some fast computer, works better with sleep on real A1?
    start_time_robot = current_time
    start_time_wall = time.time()
    # Updates the controller behavior parameters.
    lin_speed, ang_speed, e_stop = command_function(current_time)
    # print(lin_speed)
    if e_stop:
      logging.info("E-stop kicked, exiting...")
      break
    _update_controller_params(controller, lin_speed, ang_speed)
    controller.update()
    hybrid_action, contact_force,  des_com_pos, des_com_vel = controller.get_action() # !! # DEBUG desired_com_position, desired_com_velocity
    #print("FootPositionsInBaseFrame: ",robot.GetFootPositionsInBaseFrame())

    if FLAGS.logdir: _logger.log(hybrid_action, contact_force, des_com_pos, des_com_vel)

    robot.Step(hybrid_action)
    current_time = robot.GetTimeSinceReset()

    if not FLAGS.use_real_robot:
      expected_duration = current_time - start_time_robot
      actual_duration = time.time() - start_time_wall
      if actual_duration < expected_duration:
        time.sleep(expected_duration - actual_duration)
    #print("actual_duration=", actual_duration)
  if FLAGS.use_gamepad:
    gamepad.stop()

  if FLAGS.logdir:
    _logger.save_to(os.path.join(logdir, 'states.csv'))
    logging.info("logged to: {}".format(logdir))


if __name__ == "__main__":
  app.run(main)
