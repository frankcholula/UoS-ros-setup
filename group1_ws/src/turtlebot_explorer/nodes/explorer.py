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
        self.max_recent = 10 #
        self.min_goal_distance = 1 # can tweak this knob


        # Get Robot Posiion
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

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

    def neighbour_count(self, map, x, y, width=None, height=None):
        if width is None:
            width = int((len(map[0])))
        if height is None:
            height = int((len(map)))
        
        unknown_neighbors = 0
        walls = 0
        area_coefficient = 5
        
        for dx in range(-area_coefficient, area_coefficient+1):
            for dy in range(-area_coefficient, area_coefficient+1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    idx = nx + ny * width
                    if map[idx] == -1:  # Unknown
                        unknown_neighbors += 1
                    elif map[idx] > 0:  # Wall/obstacle
                        walls += 1
        
        # Penalize cells near walls, reward cells near unexplored space
        return max(unknown_neighbors - walls*0.3, 0) / float(area_coefficient**2)
    
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


    def get_robot_position(self, frame_id="map"):
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                'base_footprint', # gmapping uses base_footprint as default frame
                rospy.Time(0),
                rospy.Duration(1.0)
            )
        
            position = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y
            ])

            rospy.logwarn("Robot position: %.2f, %.2f", position[0], position[1])
            return True, position
        
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to get robot position: %s" % e)
            return False, None
        
    
    def get_goal(self, cells_to_pick, map_origin, res):
        """
        Optimization 4: Use heuristics to pick a goal
        """
        width = self.latest_map_msg.info.width
        height = self.latest_map_msg.info.height
        get_robot_success, robot_position = self.get_robot_position()
        
        scores = np.zeros(len(cells_to_pick), dtype =int)

        scores = np.array([self.neighbour_count(self.latest_map, int(cell[0]), int(cell[1]), width, height) for cell in cells_to_pick])
        scores[scores < np.mean(scores)] = 0
        exploration_weight = 0.1
        if get_robot_success:
            # take robot position into account
            dists = np.array([1.0/(np.linalg.norm((cell*res+map_origin)-robot_position) + 1e-6) for cell in cells_to_pick])
            max_dist = np.sqrt(width**2 + height**2) * res
            dists = dists / (max_dist*2)
            rospy.logwarn("dist mean, max: %f, %f", np.mean(dists), np.max(dists))
            rospy.logwarn("scores mean, max: %f, %f", np.mean(scores), np.max(scores))
            scores = (1-exploration_weight)*scores + exploration_weight * dists

        if np.any(scores > 0):
            # use softmax instead
            temp = 1.0
            exp_scores = np.exp(scores / temp)
            probs = exp_scores / np.sum(exp_scores) 
            # alternatively, can just use scores / scores.sum() and add 1 to all scores
            # scores = scores + 1
            # probs = scores / scores.sum()
            idx = np.random.choice(len(cells_to_pick), p=probs)
            rospy.loginfo("Selected cell with %d unknown neighbors", scores[idx]-1)
        else:
            idx = np.random.randint(0, len(cells_to_pick))
            rospy.loginfo("No cells with unknown neighbors, selecting randomly")


        goal = cells_to_pick[idx]
        goal_world = (goal * res) + map_origin
        
        # remember recent goals (previous implementation)
        if len(self.recent_goals) >= self.max_recent:
            self.recent_goals.pop(0)
        self.recent_goals.append(goal_world)
        
        return goal_world


        """
        Optimization 3: Pick a random cell a certain distance away from previous cells
        """
        # max_attempts = 10
        # for attempt in range(max_attempts):
        #     rand_idx = np.random.randint(0, len(cells_to_pick))
        #     goal = cells_to_pick[rand_idx]
        #     goal_world = (goal * res) + map_origin

        #     too_close = True
        #     for recent_goal in self.recent_goals:
        #         distance = np.sqrt((goal_world[0] - recent_goal[0])**2 + 
        #                         (goal_world[1] - recent_goal[1])**2)
        #         if distance < self.min_goal_distance:
        #             too_close = True
        #             break
            
        #     # If not too close to recent goals, use this goal
        #     if not too_close:
        #         break

        #     if attempt == max_attempts - 1:
        #         rospy.loginfo("Couldn't find goal away from recent ones, using random goal")

        # if len(self.recent_goals) >= self.max_recent:
        #     self.recent_goals.pop(0)
        # self.recent_goals.append(goal_world)
        
        # return goal_world

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
        timeout = rospy.Duration(60)
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
