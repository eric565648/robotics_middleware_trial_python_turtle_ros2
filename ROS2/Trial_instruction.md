# ROS2 Survey
The goal for this trial is to create python turtle node with ROS2 service subscriber action, a python node as client publish turtle bot motion or activate action, along with a given webcam image publisher to control on-screen turtle based on different color on the webcam.

**Please time yourself for each checkpoint**

## ROS2 Resources:
* ROS2 message:
* ROS2 service:
* ROS2 action:
* Minimal Example of Publisher/Subscriber (python):
* Full ROS2-Foxy online tutorial: 
* [Python Migration guide from ROS1 to ROS2](https://docs.ros.org/en/foxy/Contributing/Migration-Guide-Python.html)

## Outline
### Setup
* [Catkin Workspace](#catkin-workspace)
* [Package](#package)
* [Message Types](#message-types)
* [Service Types](#service-types)
* [Build Workspace](#build-workspace)
### ROS2 Publisher
### ROS2 Subscriber
* [Create Turtlebot Server](#create-turtlebot-server)
* [Create Turtlebot Client](#create-turtlebot-client)
### ROS2 Action

# Setup
**Start of Checkpoint 1**
## Workspace
For each ROS2 project, there's a dedicated workspace. Unlike in ROS1 we use `catkin`, in ROS2 we use `colcon` to build our workspace. The workspace here is `~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws` which is already created.

## Package
Like ROS1 (and unlike RobotRaconteur), ROS2 requires the workspace to build content. All package should be in `workspace/src` folder. In this repository there's already a webcam package (`~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src/webcam`), so you'll need to [create another package](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html) for python turtle.

```
cd ~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src
ros2 pkg create --build-type ament_python python_turtle
```

This creates a new package `python_turtle` under `~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src`.

Unlike ROS1, `python` and `C++` uses in different types of package. For `python` use `ament_python` and for `C++` use `ament_cmake`. However, it's possible to build the package with `ament_cmake_python` which both language can be used in the package.
There are also minor different touches for these two language. In this trial, we use `python`.

## Message Types
ROS2 shares the same pre-defined message types (e.g. std_msgs, sensor_msgs) with ROS1 which users can include. Furthermore, ROS2 allows customized [message types and service types](https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html). Remeber customized messages and only be defined and build in `ament_cmake` type of package (but of course can be used across all other packages.)

Let's create our own message type `turtle_msg`! We first create a new package `turtle_interfaces`.
```
cd ~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src
ros2 pkg create --build-type ament_cmake turtle_interfaces
```

Create a folder name `msg` under your package folder `turtle_interfaces`. Then create a file named `Turtlemsg.msg` and copy paste the below content. The message include the name, the pose of the turtle and the color of the turtle.
```
string name
geometry_msgs/Pose turtle_pose
string color
```

In order to let the compiler know the message and link necessary packages, we need to include them in `CMakeLists.txt` and `package.xml` as we did in ROS1.

First open `CMakeLists.txt` in `turtle_interfaces` and add the following lines before the line `ament_package()`.
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Turtlemsg.msg"
)
```

And open `package.xml` and add the following lines.
```
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

By this point, the message type should be built together when building the package.

You can include the message type in the similar way of other ROS2 messages:
```
from python_turtle import Turtlemsg
from geometry_msgs import Pose
```
And to create an object of that message type:
```
turtle_msg=Turtlemsg()
turtle_msg.name="myturtle"
turtle_msg.turtle_pose=Pose()
turtle_msg.color="red"
```

## Service Type
A ROS2 service is similar to a function call, and in the task we'll ask you to create a `setpose` and `setcolor`. ROS2 also shares the same pre-defined service types with ROS1 which users can include.

Please create a folder `srv` in the folder `turtle_interfaces` and create two files named `Setpose.srv` and `Setcolor.srv`. Copy-paste the following content to the file.

For `Setpose.srv`:
```
geometry_msgs/PoseStamped turtle_pose
---
int8 ret
```

For `Setcolor.srv`:
```
string color
---
int8 ret
```
ROS2 service has to have a return type (like a function call), so we can simply return an `int` instead of `void`.

We then also open `CMakeLists.txt` and include the following content

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/turtle_msg.msg"
  "srv/Setpose.srv"
  "srv/Setcolor.srv"
)
```

We'll get to how to use these services and messages in the later checkpoints.

## Build the Workspace and Packages
We use `colcon build` to build the packages. Remeber to setup the ROS2 environment whenever a new terminal is open.

```
source /opt/ros/foxy/setup.bash
cd ~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws
colcon build
```

You should see `Summary: X packages finished` where `X` is the number of your packages in the workspace. Three folders `build/` `install/` and `log/` were generated. Remeber to setup the workspace if you want to use packages in the workspace.

```
source ~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/install/setup.bash
```

You can also add the command to your bash script so everytime a new terminal is open, the workspace environment is setup.
```
echo 'source ~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/install/setup.bash' >> ~/.bashrc 
```

* **Checkpoint 1**: 
Try the following commands and you'll see the defined message and services.

```
ros2 interface show turtle_interface/msg/Turtlemsg
ros2 interface show turtle_interfaces/srv/Setpose
ros2 interface show turtle_interfaces/srv/Setcolor
```

Please direct to `Robotics_Middleware_Trial_Python_Turtle/readme.md` for question post.