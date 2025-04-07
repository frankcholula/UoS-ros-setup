#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from gazebo_msgs.srv import SpawnModel, DeleteModel
import random
import rospkg
import numpy as np

sim = False

# ----------- ADD YOUR POSES TO THESE LISTS ----------------------
pick_poses = [
    [-1.9715009956122742e-05, -0.9999084127158424, 0.2999024549788709, 0.6999359228991224],
    [0.00023957433215482382, -0.6306333934958621, 1.0696127583923065, 0.09814748120934524]
]

# Poses for placing objects in the left container              
place_left_poses = [
    [0.00023957433215482382, -0.6306333934958621, 1.0696127583923065, 0.09814748120934524],
    [-1.574681138961453, 0.09619623984785086, -0.2047853609996757, 1.423144433971653]
]

# Poses for placing objects in the right container
place_right_poses = [
    [0.00023957433215482382, -0.6306333934958621, 1.0696127583923065, 0.09814748120934524],
    [1.5031213262882686, 0.09628193659124484, -0.2048335912831618, 1.423125468545762]
]

# Poses for discarding objects
throw_it_away_poses = [
    [0.00023957433215482382, -0.6306333934958621, 0.2996127583923065, 0.09814748120934524],
    [-2.764681138961453, 0.09619623984785086, -0.2047853609996757, 1.423144433971653]
]
# ---------------------------------------------------


class PickAndPlace:
    def __init__(self, sim=False):
        print("========= Initialise pick and place system =========")
        # Set up moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.moveit_arm = moveit_commander.MoveGroupCommander("arm")
        self.moveit_gripper = moveit_commander.MoveGroupCommander("gripper")
        # Real limits
        self.open_lim = -0.006
        self.close_lim = 0.006
        self.step = 0.001
        # Simulated limits
        if sim:
            self.open_lim = 0.009
            self.close_lim = -0.009
            self.step = (
                -0.001
            )  # For some reason "open" and "close" are swapped, so we need to invert this step

    def grip(self):
        # Tighten the gripper one step at a time
        for val in np.arange(self.open_lim, self.close_lim, self.step):
            val = float(val)
            self.moveit_gripper.set_joint_value_target([val, val])
            moveit_plan = self.moveit_gripper.plan()
            self.moveit_gripper.execute(moveit_plan)
        if sim:
            self.scene.attach_box(
                "gripper_link",
                "block",
                touch_links=["gripper_link", "gripper_link_sub"],
            )

    def release(self):
        self.moveit_gripper.set_joint_value_target([self.open_lim, self.open_lim])
        moveit_plan = self.moveit_gripper.plan()
        self.moveit_gripper.execute(moveit_plan)

        if sim:
            self.scene.remove_attached_object("gripper_link", "block")

    def execute_move_sequence(self, sequence, name):
        print("========= Running {} sequence =========".format(name))
        for i, pose in enumerate(sequence):
            print("Goal pose {}: {}".format(i, pose))
            # ---- INSERT YOUR CODE HERE
            # ---------
            self.moveit_arm.set_joint_value_target(pose)
            plan = self.moveit_arm.plan()
            self.moveit_arm.execute(plan)
            self.moveit_arm.set_max_acceleration_scaling_factor(0.1)
            self.moveit_arm.set_max_velocity_scaling_factor(0.1)

    def inspect_object(self):
        # ---- REPLACE THIS WITH YOUR CODE
        gripper_state = self.moveit_gripper.get_current_joint_values()
        gripper_value = abs(gripper_state[0])
        print("Gripper joint value: {}".format(gripper_value))
        threshold = 0.0002
        small_obj = 0.0048
        big_obj = 0.0019
        if abs(gripper_value- small_obj) < threshold:
            print("Small object!")
            return 0
        elif abs(gripper_value - big_obj)< threshold:
            print("Big object!")
            return 1
        else:
            return -1


if __name__ == "__main__":
    rospy.init_node("run_challenge", anonymous=True)
    if sim:
        print("In sim mode")
    else:
        print("Not in sim mode")
    attempts = 0

    # Initialise pick and place controller
    controller = PickAndPlace(sim)
    controller.release()

    # Start pick and place loop
    while not rospy.is_shutdown():
        option = raw_input(
            "Attempt {}. Press enter to attempt pick and place (or enter q to quit):".format(
                attempts
            )
        )
        if option == "q":
            break

        # Delete old block and load new block
        if sim:
            controller.scene.remove_world_object("block")

            box_size = 0.0225  # small
            if random.randint(0, 1):
                box_size = 0.045  # large

            box_pose = PoseStamped()
            box_pose.header.frame_id = "world"
            box_pose.pose = Pose(position=Point(x=0.2, y=0.0, z=0.0))
            box_pose.pose.orientation.w = 1.0
            box_name = "block"

            controller.scene.add_box(
                box_name, box_pose, size=(box_size, box_size, box_size)
            )

        # Attempt pick and place
        controller.execute_move_sequence(pick_poses, "pick")
        controller.grip()
        object_type = controller.inspect_object()
        if object_type == 0:
            controller.execute_move_sequence(place_left_poses, "place left")
        elif object_type == 1:
            controller.execute_move_sequence(place_right_poses, "place right")
        else:
            print("Unknown object type {}".format(object_type))
            controller.execute_move_sequence(throw_it_away_poses, "throw it away")
        controller.release()

        attempts = attempts + 1
