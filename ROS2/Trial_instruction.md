# ROS2 Survey
The goal for this trial is to create python turtle node with ROS2 service subscriber action, a python node as client publish turtle bot motion or activate action, along with a given webcam image publisher to control on-screen turtle based on different color on the webcam.

**Please time yourself for each checkpoint**

## ROS2 Resources:
* [ROS2 topic](https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html)
* [ROS2 service](https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html)
* [ROS2 action](https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Actions.html)
* Minimal Example of [Publisher/Subscriber (python)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
* Full ROS2-Foxy online [tutorial](https://docs.ros.org/en/foxy/Tutorials.html)
* [Python Migration guide from ROS1 to ROS2](https://docs.ros.org/en/foxy/Contributing/Migration-Guide-Python.html)

## Outline
### Setup
* [Workspace](#workspace)
* [Package](#package)
* [Message Types](#message-types)
* [Service Types](#service-types)
* [Build Workspace](#build-the-workspace-and-packages)
### ROS2 Topic (Publisher-Subscriber)
* [Publisher](#publisher)
* [Subscriber](#subscriber)
### Turtlebot Server-Client
* [Turtlebot Server](#turtlebot-server)
* [Turtlebot Client](#turtlebot-client)
### Task
* [Task-1](#task-1)
* [Task-2](#task-2)
### ROS2 Service (Server-Client)
* [Service-Server](#service-server)
* [Service-Client](#service-client)
### ROS2 Action (Server-Client)
* [Action Type](#action-type)
* [Action-Server](#action-server)
* [Action-Client](#action-client)
### Task
* [Task-3](#task-3)


## Setup
**Starting Point of Checkpoint 1**
### Workspace
For each ROS2 project, there's a dedicated workspace. Unlike in ROS1 we use `catkin`, in ROS2 we use `colcon` to build our workspace. The workspace here is `~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws` which is already created.

### Package
Like ROS1 (and unlike RobotRaconteur), ROS2 requires the workspace to build content. All package should be in `workspace/src` folder. In this repository there's already a webcam package (`~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src/webcam`), so you'll need to [create another package](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html) for python turtle.

```
$ cd ~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src
$ ros2 pkg create --build-type ament_python python_turtle
```

This creates a new package `python_turtle` under `~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src`.

Unlike ROS1, `python` and `C++` uses in different types of package. For `python` use `ament_python` and for `C++` use `ament_cmake`. However, it's possible to build the package with `ament_cmake_python` which both language can be used in the package.
There are also minor different touches for these two language. In this trial, we use `python`.

### Message Type
ROS2 shares the same pre-defined message types (e.g. std_msgs, sensor_msgs) with ROS1 which users can include. Furthermore, ROS2 allows customized [message types and service types](https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html). Remeber customized messages and only be defined and build in `ament_cmake` type of package (but of course can be used across all other packages.)

Let's create our own message type `turtle_msg`! We first create a new package `turtle_interfaces`.
```
$ cd ~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src
$ ros2 pkg create --build-type ament_cmake turtle_interfaces
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

### Service Type
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

### Build the Workspace and Packages
We use `colcon build` to build the packages. Remeber to setup the ROS2 environment whenever a new terminal is open.

```
$ source /opt/ros/foxy/setup.bash
$ cd ~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws
$ colcon build
```

You should see `Summary: X packages finished` where `X` is the number of your packages in the workspace. Three folders `build/` `install/` and `log/` were generated. Remeber to setup the workspace if you want to use packages in the workspace.

```
$ source ~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/install/setup.bash
```

You can also add the command to your bash script so everytime a new terminal is open, the workspace environment is setup.
```
$ echo 'source ~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/install/setup.bash' >> ~/.bashrc 
```

* **Checkpoint 1**: 
Try the following commands and you'll see the defined message and services.

```
ros2 interface show turtle_interface/msg/Turtlemsg
ros2 interface show turtle_interfaces/srv/Setpose
ros2 interface show turtle_interfaces/srv/Setcolor
```

Please direct to [Readme](https://github.com/eric565648/robotics_middleware_trial_python_turtle_ros2) for question post.

## ROS2 Topic (Publisher-Subcriber)
**Starting Point of Checkpoint 2**

As ROS1, the basic element of ROS2 is **Node**. There are **three** three communication types between nodes, which are topics, services and actions. Among them, **action** is a new feature in ROS2. We will cover them all in this trial and start with topics.

Topics are buses between nodes. Nodes utilize **publisher** and **subscribers** to send/retrieve messages to/from a specified topic. In the trial, we have a node with publisher publishing images acquired from your webcam (i.e. a sensor) to a topic called `image_raw`, and another node suscribing the topic and display the image.

Quick note, the ROS2 is mode object-oriented than ROS1. 

### Publisher
Let's take a look at the code of the publisher node. Please see the file `webcam/cam_pub.py` in the `webcam` package folder (`~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src/webcam`). Note that instead of `src` folder, we put python scripts in the folder with **the same name as the package** in python-type package in ROS2. However, for the C++-type folder, we put it in `src` folder as we did in ROS1.

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
`<package_name>` here is obviously `webcam`. However, the `<node_name>` is not necessary the name of the script. Here it's `webcam_pub` and `webcam_sub`. We will explore that in the next checkpoint. You should see a window pop up and show whatever capture by your webcam.

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

## Turtlebot Server-Client
**Starting Point of Checkpoint 3**

Now you know the basic ROS2 node, publish and subscription concept and usage and the details to be aware of. We are going to write two python scripts with one as a ROS2 node. A node will be *server* which host the state of the turtle bot, update the (2D) pose (i.e. x, y position and yaw). Another node will be the client which give car commmands and visualize the robot.

### Turtlebot Server

Let's start with the server script. Don't worry, we have prepared a template for you. Please copy `~/robotics_middleware_trial_python_turtle_ros2/ROS2/templates/turtlebot_server.py` to `~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src/python_turtle/python_turtle/turtlebot_server.py`. As mentioned in the previous section, python scripts should be put in the folder **with the same name** as the package.

In the `def main()` function, we have initialize ROS2 and the object for you. Remeber to spin the object otherwise the script will only be execute once.
```
#initial ros2
rclpy.init(args=args)

# initial turtlebotserver object
ser_obj = TurtlebotServer()
ser_obj.get_logger().info('Turtlebot server started!')

# spin the node
rclpy.spin(ser_obj)
```

Let's see the constrctor of the object class. You can notice the class inheret a ROS2 Node class. Please name your node here. Replact the whole <> with the name of the node. The name should be a string so remeber it should be 'name_of_node'
```
class TurtlebotServer(Node):
    def __init__(self):
        super().__init__(<node name>)
```

Then, we initialize a varaible hosting the state of the turtlebot using the data type of TurtleMsg that we create in the previous section. We also have two variable hosting velocity `self.vel_x` and augular velocity `self.ang_z`. Beside variables, we declare a publisher and a subscriber. Please fill up the topic name, topic type and callback function. (One topic type should be TurtleMsg and another Twist. Figure that out!) Finally, there's a timer simulating the real world driving which callback the function every `self.sim_interval`. You might see other stuff in the scipt which were commended out. Don't worry, we will use those in the later chapter.
```
# initialize a turtle
self.turtle = TurtleMsg()

# publisher of turtlebot state
self.turtle_pub = self.create_publisher(<turtle message topic type>, <topic name>, 1)

self.vel_x = 0 # velocty in x-direction (in the turtle frame), unit: pix/sec
self.ang_vel = 0 # angular velicty in yaw-direction (in the turtle frame), unit: rad/sec

#### subsciber to car cmd ####
self.twist_sub = self.create_subscription(<car command topic type>, <car command topic name>, <car command callback>, 1)
self.twist_sub
#######################

#### Driving Simulation Timer ####
self.sim_interval = 0.02
self.create_timer(self.sim_interval, self.driving_timer_cb)
``` 

Let's see the callback functions of subscriber. The function decode the message and store the info in the variable. Please refer to this [website](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) to see how to decode the message. Remeber the callback msg has datatype `Twist` (Hint: `self.vel_x = msg.linear.x` Figuer out another one yourself!)
```
def twist_callback(self, msg):

  self.vel_x = <decode message>
  self.ang_vel = <decode message>
```

Finally the callback function of the timer. Since the orientation of ROS2 (and also ROS1) used quaternion as the representation, we have to transfer quaternion to euler angles and tranfer back after processing. We do all that for you, still it's good to take a look and put that in mind. The main thing we have to do here is to integrate the velocity and add it to the old pose so that we can know the new pose. Please rember the time interval was declare in the constructor (i.e. `self.sim_interval`).
```
def driving_timer_cb(self):
    # basic position/velocity physics
    new_x = <old_x + vel*time_interval*cos(turtle angle)>
    new_y = <old_y + vel*time_interval*sin(turtle angle)>
    new_yaw = <old_ang + ang_vel*time_interval>

    # assign to the turtle obj
    self.turtle.turtle_pose.position.x = new_x
    self.turtle.turtle_pose.position.y = new_y

    # publish new turtle state
    self.turtle_pub.publish(self.turtle)
```

### Turtlebot Client

We have finished `turtle_server.py`. Let's move on to turtle client. Please copy the file `~/robotics_middleware_trial_python_turtle_ros2/ROS2/templates/turtlebot_client.py` to `~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src/python_turtle/python_turtle/turtlebot_client.py`. And let's see the main function!.

As we did in `turtle_server.py`, we also initialize ROS2 and a object for the node. The difference is that instead of `rclpy.spin('the object')`, we have a while loop here with a `rclpy.spin_once('the object')`. You can see these two as equivelent but we can add some steps here before and after each spin. The while loop first call the update function of the object, then spin the object node. Finally using the publisher of the object to publish a Twist message. You can see the twist has linear velocity in x-direction and angular velocity in z-direction. (Think about the callback function in `turtle_server.py`). We can guess that the motion of the turtlebot might be a circle if the twist does not change during the time. Please put the unit of linear and angular velocity you like. Depends on the amount and ratio between them, the turtle might move quicker or slower, the circle might get larger or smaller.
```
def main(args=None):

    #initial ROS2
    rclpy.init(args=args)

    #initial turtle client
    cli_obj = TurtleClient()
    cli_obj.get_logger().info('Turtlebot Client Started!')
        
    while rclpy.ok():
    
        cli_obj.update()
        rclpy.spin_once(cli_obj)

        unit_x = <put how many unit you like>
        unit_z = <put how many unit you like>
        
        #### publish twist ####
        cmd_msg = Twist()
        cmd_msg.linear.x = float(50 * unit_x)
        cmd_msg.angular.z = float(1 * unit_z)
        cli_obj.twist_pub.publish(cmd_msg)
```

Still, we have to take care of the class definition. We also first name the node in the constructor. 
```
def __init__(self):
    super().__init__(<node name>)
```

Then we setup the display with python `turtle` library. Last, we declare publisher and subsriber. Please also put the topic name, topic type, and callback function here. Remind that the topic namd and topic type should match with the once in `turtle_server.py`
```
#### publisher define ####
self.twist_pub = self.create_publisher(<car command topic type>, <car command topic name>, 1)
##########################

#### subsribing turtlebot state ####
self.turtle_sub = self.create_subscription(<turtle message topic type>, <topic name>, <turtle callback function>, 1)
```

We then take a look at the callback function of subsriber. As easy as in `turtle_server.py`, the function merely store the message into the variable. Remind that the variable is for storing the turtle state so that we can visualize it later. The datatype of both message and the variable is `TurtleMsg`.
```
def turtle_callback(self, msg):
    self.turtle = msg
```

Last, the `update` function, we found it call every time in the while loop. The function first change the color of the turtle (i.e. the pen). Then set the position of the robot. Please fill the x and y coordinate here. (Hint: the x-coordinate should be `self.turtle.pose.position.x`, figure y-coordinate out!). Finally tranfer the angle from quaternion to eualer angle and set the angle.
```
def update(self):

    if self.turtle.color is 'None':
        self.turtle_display.penup()
    else:
        self.turtle_display.pencolor(self.turtle.color)

    self.turtle_display.setpos(<x coordinate>, <y coordinate>)
    
    roll, pitch, yaw = rpy_from_quat(self.turtle.turtle_pose.orientation.x,
                                    self.turtle.turtle_pose.orientation.y,
                                    self.turtle.turtle_pose.orientation.z,
                                    self.turtle.turtle_pose.orientation.w)
    self.turtle_display.seth(math.degrees(yaw))
```

### Dependencies and Entry Point and Build

Wait! Although the scripts are done. We haven't ready yet. Since we have added new scripts in the package. We need to add dependencies, entry points and build the packages. These are the steps easily to be forgotten. If you ever encounter problems, please think if you have done these steps properly.

Please open the `package.xml` first and uncommended the following line. These two lines add dependencies to the package.
```
<depend>rclpy</depend>
<depend>turtle_interfaces</depend>
```

Open `setup.py` and uncommended the following two lines. These two lines gives the entry points to the scripts. The `turtlebot_server` and `turtlebot_client` is the `<node_name>` when we use the command `ros2 run <package_name> <node_name>`.
```
'turtlebot_server = python_turtle.turtlebot_server:main',
'turtlebot_client = python_turtle.turtlebot_client:main',
```

Finally, build the workspace as we have mentioned in [this section](#build-the-workspace-and-packages). You have to build the workspace **even you only make changes in python scripts**

* **Checkpoint 3**:

Run the server script and client scripts. Open two terminals. In one terminal, source the workspace and run the following
```
$ ros2 run python_turtle turtle_server
```
Another terminal
```
$ ros2 run python_turtle turtle_client
```

You should see a turtlebot circling!

## Task
### Task-1
Now you know all the basic to use ROS2! Here are some little challenges for you! For the task 1, given `Examples/keyboard.py`, please create a scripts that allows moving turtlebot through reading from the keyboard.

* **Checkpoint 4**
Write a script that allows driving turtlebot with arrow keys. Hint: You can use most of the part in `turtle_client.py`. Try imitate the scructure of the code before you are familiar with ROS2!

### Task-2
The final goal is to drive turtlebot by showing different colors. 

* **Checkpoint 5**
Given `Example/detection_red.py` or `Example/detection_red_hsv.py` (They use different color space. One uses RGB and one uses [HSV](https://programmingdesignsystems.com/color/color-models-and-color-spaces/)). Please drive the turtlebot forward while seeing `Red`, backward while seeing `Blue`, turning while seeing `Yellow`. 

## ROS2 Advance Survey
Here, we will lead you to the survey on how to use ROS2 service and action.

## ROS2 Service (Server-Client)
**Starting Point of Checkpoint 6**

ROS2 also provide **service** communication type among nodes. Service is based on call-responde model as opposed to topic which message can flow continuously. While there can be multiple publisher publishing to a topic and multiple subsriber subscribing to that, there can be only one server hosting a ROS2 service with one or more client requesting for a service.

In this checkpoint, we will continue using `turtle_server.py` as the server script. We will write another node as the client requesting the service (request to set color).

Note that a node can host several services with several publisher and subscirber, and even have several clients at the same time.

### Service-Server

Open your `turtlebot_server.py` and let's start adding service server. The service here provided to change the color of our turtle. Please uncomment the following line and fill up the service type, service name and service callback function. Remind that we have create our customize service type in this [section](#service-types)
```
self.turtle_color_srv = self.create_service(<service type>, <service name>, <service callback function>)
```

Then we still have to add callback function so when a service is called, the function will do something and return. Please uncomment this function.
```
def set_color_callback(self, request, response):

    self.turtle.color = request.color
    self.get_logger().info('Turtle color set: %s' % (self.turtle.color))

    response.ret = 1
    return response
```

The function is pretty straightforward also. It set the color from the request message, then return a respond. The server of the service is now all set!

### Service-Client

Please copy the file `~/robotics_middleware_trial_python_turtle_ros2/ROS2/templates/service_client.py` to `~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src/python_turtle/python_turtle/service_client.py`. It share a similar code structure as `turtlebot_client.py` Let's see the main function.

After initialize ROS2 and the object. The object call the service through the function `cli_obj.color_srvcall()`. Once the function was called, it enters a while loop and waits for the service to be done. When the `future` is `done`, it shows the repsond and break out the loop. The scripts then end. Note that the script is meant to execute only once, it's not necessary to end the script when the service call has completed.
```
def main(args=None):

    #initial ROS2
    rclpy.init(args=args)

    #initial turtle client
    cli_obj = TurtleClient()
    cli_obj.get_logger().info('Turtlebot Client Started!')
    
    # call the service
    cli_obj.color_srvcall()

    while rclpy.ok():
        rclpy.spin_once(cli_obj)

        if cli_obj.service_future.done() and cli_obj.server_call:
            cli_obj.server_call = False
            try:
                response = cli_obj.service_future.result()
            except Exception as e:
                cli_obj.get_logger().info('Server call failed')
            else:
                cli_obj.get_logger().info('Server call success: %d' % (response.ret))
                break
```

In the constructor of the object class, the client of the service is created and waiting for the server of the service is online. Although it's not necessary, it's sometime safe to do so. Please fill up the type and the name of the service. Remind that it should match the ones in the `turtlebot_server.py`
```
def __init__(self):
    super().__init__('service_client')

    #### Color service client ####
    self.color_cli = self.create_client(<service type>, <service name>)
    while not self.color_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('Color service not available, waiting...')
    self.color_req = SetColor.Request()

    self.server_call = False
    #################################
```

Finally, the service call function, this function define a color and call the service. Please note that the service is called *asynchornizely*. It's highly recommend by the official ROS2 website.
```
def color_srvcall(self):

    col = 'cyan'
    
    self.color_req.color = col
    self.server_call = True
    self.service_future = self.color_cli.call_async(self.color_req)
```

### Dependencies and Entry Point and Build

Remeber to add dependencies, entry point and build your package!!!!

As we have already add dependencies in the previous chapter, we only need to add entry point in `setup.py` here. Please add the following line
```
'service_client = python_turtle.service_client:main',
```

Finally, build the workspace as we have mentioned in [this section](#build-the-workspace-and-packages). You have to build the workspace **even you only make changes in python scripts**

* **Checkpoint 6**:

Run the server script and client scripts. Open two terminals. In one terminal, source the workspace and run the following
```
$ ros2 run python_turtle turtle_server
```
Another terminal
```
$ ros2 run python_turtle service_client
```

You should see in the terminal saying the color has changed to cyan.

## ROS2 Action (Server-Client)
**Starting Point of Checkpoint 7**

**ROS2 Action** is a new communication type in ROS2 that does not exist in ROS1. We can know from the name that it will be pretty helpful if we will like to activate an action on a robot. Actions are built on service and topic. It's similar to service with also one server and one to multiple clients which can call the service. The action client will send an action goal to the action server which send a series of feedback messages during execution and a result after the action. Actions are preemptable which means you can stop them any time you want. 

In this part, you will first create a action interface which is very much like what we've done in [the previous section](#service-type). We will then add server to our `turtlebot_server.py` and create a script to add the client. The action is the robot travelling to several designated goal. The client send multiple goal pose to the server and the server tell the client which goal is the robot currently heading to and if the action success after all.

### Action Type

First create a folder name `action` in the package `turtle_interfaces`. Create a file name `TurtleToGoals.action` and copy the follow content to the file. The first part of the content is the goal of the action sent from the client to the server. The second part is the result sent from the server to the client after the action. The third part is the feedback message type which is also sent from the server to the client.
```
geometry_msgs/Pose[] goal_poses
---
int32 ret
---
geometry_msgs/Pose mid_goal_pose
```

As we did in the previous section, open `CMakelist.txt` in the package `turtle_interfaces` and add the following line to the part.
```
rosidl_generate_interfaces(${PROJECT_NAME}
  ...
  "action/TurtleToGoals.action"
  DEPENDENCIES geometry_msgs
)
```

Great that all! Now build the workspace as we did [before](#build-the-workspace-and-packages). Source the workspace and run the following command to see if the action type was built.
```
$ ros2 interface show turtle_interface/action/TurtleToGoal
``` 

### Action-Server

Please open the file `turtlebot_server.py`. As this point we are very familiar to the structure of the code. Let's head to the constructor and see how we add a action server. Please uncomment the following line. You can see that the object initialization is slightly different as we did for service, however, it's still quite straightforward. Please add action type (which we just created), action name and action callback. 
```
#### action server ####
self.action_server = ActionServer(self, <action type>, <action name>, <action callback>)
#######################
```

Next please uncomment our callback function! The function first stop whatever the turtlebot is doing by setting the velocity to 0. Then the callback message is initialized. A for loop then loops over the goal poses and publish the current goal poses as the feedback message. It also publish the turtlebot state. Finally it report success and return a result as we did for service server.
```
def travel_to_goals_cb(self, goal_handle):
    self.get_logger().info('To goals')

    self.vel_x = 0
    self.ang_vel = 0

    feedback_msg = TurtleToGoals.Feedback()

    for goal in goal_handle.request.goal_poses:
        feedback_msg.mid_goal_pose = goal
        goal_handle.publish_feedback(feedback_msg)

        self.turtle.turtle_pose = goal
        self.turtle_pub.publish(self.turtle)
        time.sleep(2)

    goal_handle.succeed()

    result = TurtleToGoals.Result()
    result.ret = 1
    return result
```

### Action-Client

Please copy the file `~/robotics_middleware_trial_python_turtle_ros2/ROS2/templates/action_client.py` to `~/robotics_middleware_trial_python_turtle_ros2/ROS2/dev_ws/src/python_turtle/python_turtle/action_client.py`. It share a similar code structure as `server_client.py`. The main function pretty much to the same thing (i.e. create node object, call the action, call rclpy.spin). Let's head to the constructor of the class.

As easy as it said, the constructor create an action client. Please fill up the type and the name of the action.
```
#### Action client ####
self.action_cli = ActionClient(self, <action type>, <action name>)
while not self.action_cli.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('TraveltoGoal action not available, waiting...')
####################################
```

Next, let's see the actino call function. The function first creates a goal object and stacks in 5 random goal poses. Then the action is called asynchronizely with a feedback callback function. Finally a response call back function is added.
```
def goal_actioncall(self):
        
    goal_msg = TurtleToGoals.Goal()
    for i in range(5):
        goal = Pose()
        goal.position.x = float(random.randint(-1*self.screen.window_width()/2+20,self.screen.window_width()/2-20))
        goal.position.y = float(random.randint(-1*self.screen.window_height()/2+20,self.screen.window_height()/2-20))

        goal_msg.goal_poses.append(goal)
    
    self.action_response_future = self.action_cli.send_goal_async(goal_msg, feedback_callback=self.goal_feedback_cb)
    self.action_response_future.add_done_callback(self.goal_response_cb)
```

Let's see the response callback function. The response callback function will response once the action server have the action call (as opposed to result callback which is call after the action is executed.) The function basically show if the goal is accepted or rejected. Then a result callback function is added.
```
def goal_response_cb(self, future):

    goal_handle = future.result()
    if not goal_handle.accepted:
        self.get_logger().info('Goal Rejected')
        return
    
    self.get_logger().info('Goal accepted')

    self.action_result_future = goal_handle.get_result_async()
    self.action_result_future.add_done_callback(self.goal_result_callback)
```

In the result callback function, it shows the result of the action.
```
def goal_result_callback(self, future):
    result = future.result().result
    self.get_logger().info('Goal Travel Done! Result:%d' % (result.ret))
```

Finally, let's see the feedback callback function. The feedback function is called every time a feedback message is sent. Here it shows the current goal heading to.
```
def goal_feedback_cb(self, feedback_msg):
    feedback = feedback_msg.feedback
    mid_goal = feedback.mid_goal_pose
    self.get_logger().info('Traveling to x:%d, y:%d' % (mid_goal.position.x, mid_goal.position.y))
```

### Dependencies and Entry Point and Build

Remeber to add dependencies, entry point and build your package!!!!

As we have already add dependencies in the previous chapter, we only need to add entry point in `setup.py` here. Please add the following line
```
'action_client = python_turtle.action_client:main',
```

Finally, build the workspace as we have mentioned in [this section](#build-the-workspace-and-packages). You have to build the workspace **even you only make changes in python scripts**

* **Checkpoint 7**:

Run the server script and client scripts. Open two terminals. In one terminal, source the workspace and run the following
```
$ ros2 run python_turtle turtle_server
```
Another terminal
```
$ ros2 run python_turtle service_client
```

You should see in the terminal that it's traveling to different goal pose.

## Task
### Task-3

**Checkpoint 8**

Now, the final checkpoint have come. You go through all essential part of ROS2. Please integrate the `Example/keyboard.py` to your `turtlebot_client.py`. When the `s` key is pressed, use the service call to change the color of the turtle. When the `a` key is pressed, use the action call to let the turtlebot travel to 5 different goal points. Use what you've learn, the code and have fun!
