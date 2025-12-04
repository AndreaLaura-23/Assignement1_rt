# Assignment 1: Two-Turtle Control System (ROS2)

## Description
This ROS2 package implements a two turtle control system in turtlesim.

### Nodes Implemented

1. **First Node: Interactive UI Control (`node1.py`)**
   - Through the terminal, the node asks the user to select which turtle to control.
   - Once selected, the user inputs the linear and angular velocities.
   - Each command is executed for 1 second, unless the turtle hits the window boundary or it collides with the other turtle.
   - If the user presses `q`, all nodes shut down immediately.
     
2. **Second Node: Distance Monitorating Node (`node2.py`)**
   - Continuously computes the distance between the two turtles and prints it.
   - Overrides user commands in case of: turtle-to-turtle collision or turtle-to-boundary collision.
   - Collision Avoidance: If the distance is less than 1.0 unit, both turtles perform a “back-off” maneuver to restore a safe separation distance.
   - Boundary Protection: When a turtle hits a boundary, it rotates toward the center of the window and moves inward. This prevents the turtle from getting stuck in corners.
     
3. **Third Node: Turtle Spawn (turtle_spawn.py)**
   - This Node just spawn the second terminal. (Already given in the assignment)

## How to build
Navigate to your ROS2 workspace (for me it is ros2_ws) and build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select assignment1_rt
source install/setup.bash
```

## How to run
Launch everything with one command:
```bash
ros2 run assignment1_rt node1.py
ros2 run assignment1_rt node2.py
ros2 run assignment1_rt turtle_spawn.py
```
