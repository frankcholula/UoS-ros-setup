# How the code roughly works
1. gmapping builds a map from sensor data
2. explorer.py requests the current map
3. explorer.py selects a random goal
4. move_base_client sends the goal to move_base
5. move_base navigates the robot
6. As the robot moves, sensors feed new data to gmapping
7. gmapping updates the map
8. Once the goal is reached, the cycle repeats


## Optimizations:
1. Filtering to only choose free cells for exploration targets
2. Adding a timeout for goals to prevent the robot from getting stuck (30 seconds to get to the goal)
3. Remember the last 10 goals and randomly sample a goal (maximum 10 times) that is far from the last 10 goals.

## Still Need to Implement:
4. prefer edge of known space so the robot prefers goal near the boundary between known and unknown. Can do something like a weighted appraoch.
5. improve upon 4 and ignore all non-frontier cells completely.
6. build upon 5 and find the fastest path to a frontier cell.
7. information gain-based exploration.

## Probably outside the scope of this challenge...
8. multi-robot exploration
