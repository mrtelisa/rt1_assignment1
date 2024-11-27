
# **Assignment1_rt**
During this assignment, two nodes were implemented: 
- `ass1_node1`, based on the code written in `UI.cpp`, which allows the user to interact with two turtles;
- `ass1_node2`, based on the code written in `distance.cpp`, which works as a controller of the distance between the two turtles and makes them stop in case of collision with the wall of between themselves.

# **Requirements**

- **ROS**: Must be installed and properly configured on the system.
- **Turtlesim Package**: Must be installed in your ROS workspace. 
  
# **Installation commands**
In order to use this package, cloning the repository in your ROS environment is necessary:

```bash
git clone https://github.com/mrtelisa/assignment1_rt
```
In order to compile the nodes, in the workspace the following command must be launched:

```bash
catkin_make
```

# **Compilation**
Firstly, launch ROS:
```bash
roscore
```
Then, open a new terminal window and launch the turtlesim environment:
```bash
rosrun turtlesim turtlesim_node
```
Now our nodes can be launched; the node1, based on `UI.cpp`, can be launched with:
```bash
rosrun assignment1_rt ass1_node1
```
while the node2, based on `distance.cpp`, can be launched with:
```bash
rosrun assignment1_rt ass1_node2
```

In order to monitor the distance topic, use:
```bash
rosrtopic echo /turtle_distance
```

# **Description of the nodes**
## Node1: UI.cpp
This node follows these steps:
- Spawns a new turtle (`turtle2`) in the environment;
- Generates in the terminal an interface where the user is required to interact with, selecting:
  - the turtle to control (`turtle1` or `turtle2`);
  - the linear velocities (along x and y axes);
  - the angular velocity;
- Sends the movement requested to the selected turtle for 1 second; then it stops and the user can insert a new command.

## Node2: distance.cpp
This node follows these steps:
- Calculates and publishes the value of the distance between the two turtles in a specific topic;
- Checks the distance and stops the moving turtle if:
  - the distance between the two turtles is smaller than a certain threshold, which in this case is set equal to 1.
  - the turtle is assuming a pose too near to the borders; this control is implemented using two fixed values of thresholds, too.
  
# **Interaction with the nodes**
While the node2 sends a message only if the turtle is too close to the borders or to the other turtle, the first node requires the user to interact with the terminal. 
The values required are respectively the following ones:
- The turtle to control: `turtle1` or `turtle2`;
- The linear velocity along the x-axis;
- The linear velocity along the y-axis;
- The angular velocity;
  
After every value inserted as input, pressing enter is required.
Also, to close the node it is necessary to press `Ctrl+C`, followed by "q" and eventually press enter.

