import sys
import time

import cv2
import numpy as np

import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import estop_pb2
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client.estop import EstopClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2

movement_commands_definition = [
    [1.0, 0.0, 0.0], # forward
    [0.0, 0.0, np.pi/2], # left
    [0.0, 0.0, -1 * np.pi/2], # right
]

movement_commands = [
    [1.0, 0.0, 0.0], # forward
    [0.0, 0.0, -1 * np.pi/2], # right
    [1.0, 0.0, 0.0], # forward
    [0.0, 0.0, np.pi/2], # left
    [1.0, 0.0, 0.0], # forward
    [1.0, 0.0, 0.0], # forward
    [0.0, 0.0, np.pi/2], # left
    [1.0, 0.0, 0.0], # forward
    [0.0, 0.0, np.pi/2], # left
    [1.0, 0.0, 0.0], # forward
    [0.0, 0.0, -1 * np.pi/2], # right
    [1.0, 0.0, 0.0], # forward
    [0.0, 0.0, -1 * np.pi/2], # right
    [1.0, 0.0, 0.0], # forward
    [0.0, 0.0, np.pi/2], # left
    [1.0, 0.0, 0.0], # forward
    [0.0, 0.0, np.pi/2], # left
    [1.0, 0.0, 0.0], # forward
    [1.0, 0.0, 0.0], # forward
    [0.0, 0.0, np.pi/2], # left
    [1.0, 0.0, 0.0], # forward
    [0.0, 0.0, -1 * np.pi/2], # right
    [1.0, 0.0, 0.0] # forward
]

def verify_estop(robot):
    """Verify the robot is not estopped"""

    client = robot.ensure_client(EstopClient.default_service_name)
    if client.get_status().stop_level != estop_pb2.ESTOP_LEVEL_NONE:
        error_message = 'Robot is estopped. Please use an external E-Stop client, such as the' \
                        ' estop SDK example, to configure E-Stop.'
        robot.logger.error(error_message)
        raise Exception(error_message)


def preprogrammed_movements():
    """A simple example of using the Boston Dynamics API to command Spot's arm."""

    # See hello_spot.py for an explanation of these lines.

    sdk = bosdyn.client.create_standard_sdk('ArmObjectGraspClient')
    robot = sdk.create_robot("192.168.1.109")
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    assert robot.has_arm(), 'Robot requires an arm to run this example.'

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    verify_estop(robot)

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    lease_client.take()


    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        robot.logger.info('Powering on robot... This may take a several seconds.')
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), 'Robot power on failed.'
        robot.logger.info('Robot powered on.')

        # Tell the robot to stand up. The command service is used to issue commands to a robot.
        # The set of valid commands for a robot depends on hardware configuration. See
        # RobotCommandBuilder for more detailed examples on command building. The robot
        # command service requires timesync between the robot and the client.
        robot.logger.info('Commanding robot to stand...')
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec=10)
        robot.logger.info('Robot standing.')
        
        start_time = time.time()
        for movement_command in movement_commands:
            body_frame_move_command(command_client, robot, movement_command)
            time.sleep(1.5)

        print(f"All movements took {time.time() - start_time} seconds")

        robot.logger.info('Sitting down and turning off.')

        # Power the robot off. By specifying "cut_immediately=False", a safe power off command
        # is issued to the robot. This will attempt to sit the robot before powering off.
        robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not robot.is_powered_on(), 'Robot power off failed.'
        robot.logger.info('Robot safely powered off.')
        
def body_frame_move_command(command_client, robot, movement_command):
    ### Create odometry movement command
    x = movement_command[0]
    y = movement_command[1]
    yaw = movement_command[2]
    trajectory_command = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(x, y, yaw, robot.get_frame_tree_snapshot(), locomotion_hint=spot_command_pb2.HINT_CRAWL)
    robot.logger.info("Moving to object")
    cmd_id = command_client.robot_command(trajectory_command, end_time_secs=time.time()+10)
    robot.logger.info("Movement finished.")
    
    while True:
        feedback = command_client.robot_command_feedback(cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            print('Failed to reach the goal')
            return False
        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            print('Arrived at the goal.')
            break
        time.sleep(0.2)


def main():

    try:
        preprogrammed_movements()
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception('Threw an exception')
        return False


if __name__ == '__main__':
    if not main():
        sys.exit(1)