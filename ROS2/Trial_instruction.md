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
**Starting Point of Checkpoint 1**
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

Please direct to [Readme](https://github.com/eric565648/robotics_middleware_trial_python_turtle_ros2) for question post.

# ROS2 Topic (Publisher-Subcriber)
**Starting Point of Checkpoint 2**

As ROS1, the basic element of ROS2 is **Node**. There are **three** three communication types between nodes, which are topics, services and actions. Among them, **action** is a new feature in ROS2. We will cover them all in this trial and start with topics.

Topics are buses between nodes. Nodes utilize **publisher** and **subscribers** to send/retrieve messages to/from a specified topic. In the trial, we have a node with publisher publishing images acquired from your webcam (i.e. a sensor) to a topic called `image_raw`, and another node suscribing the topic and display the image.

Quick note, the ROS2 is mode object-oriented than ROS1. 

### Publisher
Let's take a look at the code of the publisher node. Please see the file `webcam/cam_pub.py` in the `webcam` package folder (`~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src/webcam`). Note that instead of `src` folder, we put python scripts in the folder with the same name as the package in python-type package in ROS2. However, for the C++-type folder, we put it in `src` folder as we did in ROS1.

First, we include the necessary python libraries as we always did. The library for ROS2 in python is `rclpy`.
```
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
```

Then, in the `main` function, we initialize ROS2 and a webcam object which the class inherites ROS2 `node` class. As in ROS1, we have to spin the node otherwise it will just execute once and shutdown.
```
webcam=Webcam_Impl()
webcam.get_logger().info('Webcam Node Started!') 
rclpy.spin(webcam)
```

Let's see `Webcam_Impl` class. In the constructor, we initialize a publisher which publish message type `Image` to the topic `image_raw`. You can also see we name the node as `webcam`.
```
super().__init__('webcam')
self.img_publisher = self.create_publisher(Image, 'image_raw', 1)
```

After which are setup for webcams, `CvBridge` helps us covert image data formation from ROS1/2 message to opencv. Then a timer is create which periodicly calls the function `capture_frame`. Timers are extremely useful as well in ROS2.
```
self.timer = self.create_timer(0.03, self.capture_frame)
```

Finally, in the `capture_frame` function, images are read from the webcam. Whenever a image is read, the publisher will publisher it (to the topic name `image_raw`).
```
rval,img_data = self.camera.read()
  if rval:
      self.img_publisher.publish(self.bridge.cv2_to_imgmsg(img_data, "bgr8"))
      return img_data 
```

### Subscriber
Now, let us see `webcam/cam_sub.py` in `webcam` package.

First, we inistializa ROS and the object and spin it as we did in the pubisher script.
```
rclpy.init(args=args)
imgsub_obj = WebcamSub()
rclpy.spin(imgsub_obj)
```

Second, in the constructor, we name the node `stream_node`, create a CvBride object, and create a subcriber (i.e. `subscription`). Whenever a new message is retrieved from the topic (i.e. whenever a message is published by a publisher to the topic), the subscriber active the callback function.
```
super().__init__('stream_node')
self.bridge = CvBridge()
self.img_subscription = self.create_subscription(Image, 'image_raw', self.img_callback, 1)     
```

In the callback function, `img_msg` is the message object that is retrieve from the topic. We here convert it from a ROS message object to a opencv image. Afterware, the image is deplayed in a window.
```
def img_callback(self, img_msg):
  try:
      cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
  except CvBridgeError as e:
      self.get_logger().info(e)
  
  cv2.namedWindow("Image")
  if cv_image is not None:
      cv2.imshow("Image", cv_image)
  if cv2.waitKey(50)==-1:
      cv2.destroyAllWindows()
```

* **Checkpoint 2**

[Build the workspace](#build-the-workspace-and-packages). Please remeber to do so every time you make changes **even it is a python-script!** 

Open two terminal and run the publisher and subscriber nodes in one in each using following command. (Remeber to [setup your workspace](#build-the-workspace-and-packages) environment every time open a new terminal)
```
ros2 run <package_name> <node_name>
```
`<package_name>` here is obviously `webcam`. However, the `<node_name>` is not necessary the name of the script. We will explore that in the next checkpoint. You should see a window pop up and show whatever capture by your webcam.

From time to time, we are not able to examine if the message publish successfully in such clear way. Luckily, ROS2 provide a series of tool to help us with that (like in ROS1). Run the following in a new terminal. (Also remeber to setup your workspace.)
```
ros2 topic hz image_raw
```

The command will show the frequency of messages publishing to the topic. To see the newest message publishing to the topic, use the following command
```
ros2 topic echo image_raw
```

If you do so, you will see a lot of numbers appear in the terminal. That is because images is acutally reprented with a large matrix in computer.

If you would like to see all topics, use the following command
```
ros2 topic list
```

# ROS2 Service (Server-Client)
**Starting Point of Checkpoint 3**

ROS2 also provide **service** communication type among nodes. Service is based on call-responde model as opposed to topic which message can flow continuously. While there can be multiple publisher publishing to a topic and multiple subsriber subscribing to that, there can be only one server hosting a ROS2 service with one or more client requesting for a service.

In this checkpoint, you will write a node with a server hosting a service (to set the pose of our turtlebot), and another node as the client to request for the service (to request the set pose).

In contract to the previous checkpoint, you will write a node from the scratch. We will see a few new steps to allow the script being run by ROS2. But don't worry, we are here to hint and help!

## Service-Server

In [this section](#package) we've created a package `python_turtle`. 

## Service-Client