.PHONY: turtle
turtle:
	@rosrun turtlesim turtlesim_node

.PHONY: teleop
teleop:
	@rosrun turtlesim turtle_teleop_key

.PHONY: clear
clear:
	@rosservice call /clear "{}"

.PHONY: reset
reset:
	@rosservice call /reset "{}"

.PHONY: teleport_absolute
teleport_absolute:
	@rosservice call /turtle1/teleport_absolute "{x: 5.0, y: 5.0, theta: 2.0}"

.PHONY: teleport_relative
teleport_relative:
	@rosservice call /turtle1/teleport_relative "{linear: 1.0, angular: 1.0}"

.PHONY: set_pen
set_pen:
	@rosservice call /turtle1/set_pen "{r: 255, g: 0, b: 0, width: 0, 'off': 0}" 


.PHONY: list_service
list_service:
	@rosservice list

.PHONY: list_params
list_params:
	@rosparam list

.PHONY: rviz
rviz:
	@rosrun rviz rviz

.PHONY: launch
launch:
	@roslaunch hw turtles.launch

.PHONY: rosbag
rosbag:
	@rosbag play --clock /root/catkin_ws/labs/lab3/lab3_data.bag

.PHONY: clean
clean:
	@rosparam delete /
