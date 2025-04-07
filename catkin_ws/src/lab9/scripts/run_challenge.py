#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from gazebo_msgs.srv import SpawnModel, DeleteModel
import random
import rospkg
import numpy as np

sim = True


# ----------- ADD YOUR POSES TO THESE LISTS ----------------------
pick_poses = [
    [
        2.124673123038434e-05,
        -0.9961996054736515,
        0.29539655487202054,
        0.6955778031750208,
    ],
    [
        -0.0009911954172752147,
        0.15608225639418016,
        0.1315341578616458,
        1.0474036387372383,
    ],
]

place_left_poses = [
    [
        2.124673123038434e-05,
        -0.9961996054736515,
        0.29539655487202054,
        0.6955778031750208,
    ],
    [
        -0.0009911954172752147,
        0.15608225639418016,
        0.1315341578616458,
        1.0474036387372383,
    ],
]

place_right_poses = [
    [
        2.124673123038434e-05,
        -0.9961996054736515,
        0.29539655487202054,
        0.6955778031750208,
    ],
    [
        -0.0009911954172752147,
        0.15608225639418016,
        0.1315341578616458,
        1.0474036387372383,
    ],
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
        gripper_position = gripper_state[0]
        threshold = 0.006
        if gripper_position > threshold:
            rospy.loginfo("large object")
            return 1
        else:
            rospy.loginfo("small object")
            return 0


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
        controller.release()

        attempts = attempts + 1
