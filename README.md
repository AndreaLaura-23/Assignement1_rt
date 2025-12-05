# Assignment 1: Two-Turtle Control System (ROS2)

## Description
This ROS2 package implements a two turtle control system in turtlesim.

### Nodes Implemented

**First Node: Interactive UI Control (`node1.py`)**
   - Through the terminal, the node asks the user to select which turtle to control.
   - Once selected, the user inputs the linear and angular velocities.
   - Each command is executed for 1 second, unless the turtle hits the window boundary or it collides with the other turtle.
     
**Second Node: Distance Monitorating Node (`node2.py`)**
   - Continuously computes the distance between the two turtles and check it if the two turtles are too close or they hit one boundary.
   - Overrides user commands in case of: turtle-to-turtle collision or turtle-to-boundary collision.
   - Collision Avoidance: If the distance between two turtles becomes less than 1.0 unit, the moving turtle will stop and wait until the turtle ahead resumes its movement.
   - Boundary Protection: When a turtle reaches a boundary, it stops and can no longer move forward in that direction. It may still move backward or choose a different direction.
     
**Third Node: Turtle Spawn (turtle_spawn.py)**
   - This Node just spawn the second terminal. (Already given in the assignment)

## How to build
Navigate to your ROS2 workspace (for me it is ros2_ws) and build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select assignment1_rt
source install/setup.bash
```

## How to run
Launch everything with one command everytime in a new terminal:
```bash
ros2 run turtlesim turtlesim_node
ros2 run assignment1_rt node1.py
ros2 run assignment1_rt node2.py
ros2 run assignment1_rt turtle_spawn.py
```
