# Robotics Software Engineer Task

You are asked to design and implement a pick-and-place application using a Gazebo simulation environment. The expected result of this task is a docker image and a GitHub repo with the source code.

## Proposed Task

The scope of this assignment is limited to the robot control part of a continuous pick-and-place cycle. There are two tables; one with 5 blocks and one empty. The goal is to move all of the blocks onto the other table.

No vision/perception is needed. Poses of the blocks are accessible through the ros_service:

`/gazebo/get_model_state`

The gripper can be controlled through a topic:

`/gripper_joint_position/command`

## Solution

A video of my pick and place pipeline can be found [here](https://youtu.be/dgJlA0qIIqM), developed in Ubuntu 20.04 with ROS1 Noetic. I used the [MoveIt!](https://moveit.ros.org/) framework to implement the control and motion planning of the UR5E arm. The framework allows for the modular use of a variety of controllers, planners and sensing plugins to suite the needs of a given robot application. For the pick and place application highlighted here, I used the joint trajectory controller plugin to control the arm in joint space to a trajectory derived via [OMPL](https://ompl.kavrakilab.org/). Kinematic solvers are implemented with the KDL plugin.

## Build && Run

### Using docker image

```bash
$ docker pull docker.io/bsbretly/remy-robotics-test:latest
$ xhost local:root
$ docker-compose up
```

Alternatively, you can build manually with the given Dockerfile.

### Building from Source

Open a terminal window and from the root of this directory run:
```bash
$ ./build.sh
$ cd catkin_ws
$ catkin build
$ source devel/setup.bash
$ roslaunch simple_scene gazebo.launch
```

Next, open another terminal and from the root of this directory run:
```bash
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch pick_and_place pick_and_place.launch
```

## Additional Questions

- How would you improve the efficiency and reliability?

To improve time efficiency, I would further iterate upon the __default_velocity_scaling_factor__ and __default_acceleration_scaling_factor__, defined in the "joint_limits.yaml" of the robot_moveit_config package, with the intent of increasing each as close to 1 as possible while still robustly performing the task. This could of course come at the cost of pick/place efficiency, so an ideal trade-off must be reached. I would also devote more time into optimizing the path planning, experimenting with different planners (i.e. other than OMPL) as well as the joint limit parameters in the previously mentioned "joint_limits.yaml" file.

- What would you do if we needed multiple robots in a system?

My first solution would be to simply run multiple instances of the MoveIt! framework, one for each robot. Care would need to be taken in order to ensure the workspace of each robot does not overlap. If overlap can't be mitigated, a planning coordination framework must be implemented. A quick search reveals that multiple libraries exist to tackle such multi-robot planning problems (e.g. [multi-arm-coordination](https://nbfigueroa.github.io/multi-arm-coordination/)).

- How would you deploy the application to multiple physical locations? What is needed to scale it?

For scaling the addition of multiple robots, I would develop a modular framework that allows for the control and planning of each robot, with the communication to a "high-level" planning coordination node. The planning coordination node would communicate amongst each robot node and ensure workspaces were not overlapping. For application specific constraints dictated by different physical locations, I would look to create a hardware-agnostic control and planning framework that allowed for differing physical locations of the robot arm(s). This could be achieved by building inputs to the software framework via parameters (e.g. workspace parameters, obstacle parameters, etc.).

