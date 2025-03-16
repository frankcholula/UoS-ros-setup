# How the code roughly works
1. gmapping builds a map from sensor data
2. explorer.py requests the current map
3. explorer.py selects a random goal
4. move_base_client sends the goal to move_base
5. move_base navigates the robot
6. As the robot moves, sensors feed new data to gmapping
7. gmapping updates the map
8. Once the goal is reached, the cycle repeats