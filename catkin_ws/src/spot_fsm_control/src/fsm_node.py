#!/usr/bin/env python

import time
import sys
sys.path.append("./hagrid/")
sys.path.append("./src/")

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import ast
import numpy as np
import logging
# mp_drawing = mp.solutions.drawing_utils
# mp_drawing_styles = mp.solutions.drawing_styles

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit, block_until_arm_arrives
from bosdyn.choreography.client.choreography import ChoreographyClient

from bosdyn.api import (image_pb2, arm_command_pb2, geometry_pb2, robot_command_pb2, synchronized_command_pb2, trajectory_pb2)
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient

from spot_fsm_control.spot_control_interface import SpotControlInterface
from spot_fsm_control.finite_state_machine import SpotStateMachine

logging.basicConfig(format="[LINE:%(lineno)d] %(levelname)-8s [%(asctime)s]  %(message)s", level=logging.INFO)

def try_state_send(state_machine, action):
    print("Current state:", state_machine.current_state)
    try:
        print(f"trying to: .{action}.")
        state_machine.send(action)
    except Exception as e:
        print(e)
        try:
            state_machine.send("stop_action")
            print("Stop action first")
            state_machine.send(action)
        except Exception as e:
            print(e)
            try:
                state_machine.send("stand_up")
                print("Stand up first")
                state_machine.send(action)
            except Exception as e:
                print(f"{action} not possible")
                print(e)
                ## Do some handling or more feedback to user


class FsmNode:

    def __init__(self, robot: SpotControlInterface):
        self.robot = robot
        self.robot.init_pos_empty = False
        self.sm = SpotStateMachine(robot=robot)
        self.arm_pos_init = [0, 0, 0]
        self.arm_ori_init = [1, 0, 0, 0]
            
    def callback_action(self, data):
        print(f"\nI heard: {data.data}")
        if data.data == "stop_action" and self.robot.current_state_direct_control:
            self.robot.current_state_direct_control = False
            print("Turning Of direct control")
        try_state_send(self.sm, data.data)
        
    def callback_gripper(self, data):
        if self.robot.current_state_direct_control:
            print(data)
            close_or_open = data.data
            self.robot.gripper(close_or_open)
            # time.sleep(1)
        else:
            pass
        
    
    def callback_hand_pose(self, data):
        if self.robot.current_state_direct_control:
            if self.robot.init_pos_empty:
                self.arm_pos_init = [data.position.x, data.position.y, data.position.z]
                self.arm_ori_init = math_helpers.Quat(
                    w = data.orientation.w,
                    x = data.orientation.x,
                    y = data.orientation.y,
                    z = data.orientation.z
                )
                self.robot.init_pos_empty = False
        
            pos = 8*(np.array([data.position.x, data.position.y, data.position.z] - np.array(self.arm_pos_init)))
            
            quaternion = math_helpers.Quat(
                w = data.orientation.w,
                x = data.orientation.x,
                y = data.orientation.y,
                z = data.orientation.z
            )
            orientation = math_helpers.Quat()
            
            hand_pose = math_helpers.SE3Pose(x=pos[0], y=pos[1], z=pos[2], rot=orientation)
            traj_point = trajectory_pb2.SE3TrajectoryPoint(pose=hand_pose.to_proto())
            self.robot.direct_control_trajectory_list.append(traj_point)
            
            self.arm_pos_init = np.array([data.position.x, data.position.y, data.position.z])
            self.arm_ori_init = quaternion
            
        else:
            pass
    
    def run(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("chatter", String, self.callback_action)
        rospy.Subscriber("gripper", String, self.callback_gripper)
        rospy.Subscriber("hand_pose", Pose, self.callback_hand_pose)
        rospy.spin()


if __name__ == "__main__":

    robotInterface = SpotControlInterface()
    # robotInterface = None
    
    if robotInterface:
        sdk = bosdyn.client.create_standard_sdk('SpotControlInterface')
        robot = sdk.create_robot("192.168.51.157")
        bosdyn.client.util.authenticate(robot)
        robot.time_sync.wait_for_sync()
        assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                        "such as the estop SDK example, to configure E-Stop."
        
        lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            # Now, we are ready to power on the robot. This call will block until the power
            # is on. Commands would fail if this did not happen. We can also check that the robot is
            # powered at any point.
            robot.logger.info("Powering on robot... This may take several seconds.")
            robot.power_on(timeout_sec=20)
            assert robot.is_powered_on(), "Robot power on failed."
            robot.logger.info("Robot powered on.")

            robotInterface.image_client = robot.ensure_client(ImageClient.default_service_name)

            # Create a command client to be able to command the robot
            robotInterface.command_client = robot.ensure_client(RobotCommandClient.default_service_name)
            robotInterface.robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
            robotInterface.manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)
            
            robotInterface.stand(0.0)
            time.sleep(1)

            fsm = FsmNode(robot=robotInterface) 
            fsm.run()
    else:
        fsm = FsmNode(robot=robotInterface) 
        fsm.run()
        
    # robotInterface.sit_down()