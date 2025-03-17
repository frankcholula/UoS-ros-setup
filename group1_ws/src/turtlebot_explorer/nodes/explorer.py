#!/usr/bin/env python

from __future__ import print_function

import rospy
import tf2_ros
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseGoal
from nav_msgs.srv import GetMap, GetMapResponse

from move_base_client import GoalSender

# Random Explorer Class
class RandomExplorer:

    def __init__(self):
        # Define a Service for Map
        rospy.wait_for_service('/dynamic_map')
        self.get_map_srv = rospy.ServiceProxy('/dynamic_map', GetMap)

        # Define a Simple Goal Client
        self.goal_sender = GoalSender()
        
        self.latest_map_msg = None
        self.latest_map = None

        # remember recent goals
        self.recent_goals = []
        self.max_recent = 10
        self.min_goal_distance = 1 # can tweak this knob

    def update_map(self):

        try:
            map_resp = self.get_map_srv()
            assert isinstance(map_resp, GetMapResponse)

            if map_resp.map.info.width <= 64:  #Default size of map before any input (64x64)
                return False

            self.latest_map_msg = map_resp.map
            self.latest_map = np.array(self.latest_map_msg.data)
            print(self.latest_map_msg.info)

        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed to get map: %s" % e)
            return False

        return True

    def process_map(self):

        # Get Map Metadata
        width = self.latest_map_msg.info.width
        height = self.latest_map_msg.info.height
        res = self.latest_map_msg.info.resolution
        map_origin = np.array([self.latest_map_msg.info.origin.position.x, self.latest_map_msg.info.origin.position.y])
        map_frame_id = self.latest_map_msg.header.frame_id
        stamp = self.latest_map_msg.header.stamp

        # Pick Goal and Create Msg
        cells_to_pick = self.get_valid_cells(height, self.latest_map, width)
        goal = self.get_goal(cells_to_pick, map_origin, res)
        goal_msg = self.make_goal_msg(goal, map_frame_id)

        return goal_msg

    # Return explored cells
    def get_valid_cells(self, height, gridmap, width):
        # -1 is unknown, 0 is free, >0 is occupied
        # Get Number of Explored Cells
        cells_explored = np.count_nonzero(gridmap == 0) # filter out obstacles
        rospy.loginfo("Cells Explored %i", cells_explored)

        # Create an Array with Cells to Pick
        cells = 0
        cells_to_pick = np.zeros((cells_explored, 2))
        for y in range(0, height):
            for x in range(0, width):
                idx = x + y * width
                if gridmap[idx] == 0:
                    cells_to_pick[cells][0] = x
                    cells_to_pick[cells][1] = y
                    cells = cells + 1
        return cells_to_pick

    # Select Random Valid Cell
    def get_goal(self, cells_to_pick, map_origin, res):
        max_attempts = 10

        for attempt in range(max_attempts):
            rand_idx = np.random.randint(0, len(cells_to_pick))
            goal = cells_to_pick[rand_idx]
            goal_world = (goal * res) + map_origin

            too_close = True
            for recent_goal in self.recent_goals:
                distance = np.sqrt((goal_world[0] - recent_goal[0])**2 + 
                                (goal_world[1] - recent_goal[1])**2)
                if distance < self.min_goal_distance:
                    too_close = True
                    break
            
            # If not too close to recent goals, use this goal
            if not too_close:
                break

            if attempt == max_attempts - 1:
                rospy.loginfo("Couldn't find goal away from recent ones, using random goal")

        if len(self.recent_goals) >= self.max_recent:
            self.recent_goals.pop(0)
        self.recent_goals.append(goal_world)
        
        return goal_world

    def make_goal_msg(self, goal, frame_id="map"):
        # Get Message
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]
        goal_msg.pose.position.z = 0
        goal_msg.pose.orientation.x = 0
        goal_msg.pose.orientation.y = 0
        goal_msg.pose.orientation.z = 0
        goal_msg.pose.orientation.w = 1
        goal_msg.header.frame_id = frame_id
        goal_msg.header.stamp = rospy.Time.now()
        return goal_msg

    def send_goal(self, goal):
        # bypasses our move_base_client send_goal method and add a timeout
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header = goal.header
        goal_msg.target_pose.pose = goal.pose

        rospy.loginfo("Sending goal to: %.2f, %.2f", # fuck we're using python2
                    goal.pose.position.x, 
                    goal.pose.position.y)
        
        self.goal_sender.client.send_goal(goal_msg, 
                                        self.goal_sender.done_cb, 
                                        self.goal_sender.active_cb, 
                                        self.goal_sender.feedback_cb)
        
        # Add timeout (this is the only change from the original)
        timeout = rospy.Duration(30)
        success = self.goal_sender.client.wait_for_result(timeout)
        
        if not success:
            rospy.logwarn("Goal not achieved in time, switching to a new goal")
            self.goal_sender.client.cancel_goal()
   
    def loop(self):

        while not rospy.is_shutdown():
            rospy.loginfo("New Goal...")

            if self.update_map():
                goal_msg = self.process_map()
                self.send_goal(goal_msg)


if __name__ == '__main__':
    rospy.init_node('random_explorer')
    explorer = RandomExplorer()
    explorer.loop()
