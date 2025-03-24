.PHONY: omws
omws:
	@echo "source /vol/vssp/signsrc/openmanipulator_ws/devel/setup.bash"

.PHONY: omc
omc:
	roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false

.PHONY: omg
omg:
	roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch

.PHONY: joint
joint:
	rostopic echo /joint_states

.PHONY: teleop
teleop:
	roslaunch open_manipulator_teleop 
	open_manipulator_teleop_keyboard.launch

.PHONY: moveit
moveit:
	roslaunch open_manipulator_controllers joint_trajectory_controller.launch sim:=true