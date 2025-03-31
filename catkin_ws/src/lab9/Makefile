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

.PHONY: spawn_block
spawn_block:
	rosrun gazebo_ros spawn_model -urdf -file $(shell rospack find lab9)/models/block.urdf -model block -x 0.2

.PHONY: spawn_trays
spawn_trays:
	rosrun gazebo_ros spawn_model -sdf -file $(shell rospack find lab9)/models/tray.sdf -model tray -y 0.3
	rosrun gazebo_ros spawn_model -sdf -file $(shell rospack find lab9)/models/tray.sdf -model tray2 -y -0.3

.PHONY: delete_block
delete_block:
	rosservice call /gazebo/delete_model "model_name: 'block'"

.PHONY: respawn_block
respawn_block: delete_block spawn_block

.PHONY: record_pos
record_pos:
	rostopic echo -n 1 /joint_states >> record_pos.txt

.PHONY: launch
launch:
	roslaunch lab9 run_challenge.launch
	
.PHONY: run_challenge
run_challenge:
	rosrun lab9 run_challenge.py