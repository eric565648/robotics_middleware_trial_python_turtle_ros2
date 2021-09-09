import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import random
import turtle
import termios, fcntl, sys, os

import cv2
import numpy as np
from cv_bridge import CvBridge
from cv_bridge.core import CvBridgeError
from sensor_msgs.msg import Image

from geometry_msgs.msg import Twist, Pose

from turtle_interfaces.srv import SetColor, SetPose
from turtle_interfaces.msg import TurtleMsg
from turtle_interfaces.action import TurtleToGoals

class TurtleClient(Node):
    def __init__(self):
        super().__init__('turtlebot_client')

        #### Display/Turtle Setup ####
        self.screen = turtle.Screen()
        self.screen.bgcolor('darkblue')
        self.turtle_display = turtle.Turtle()
        self.turtle_display.shape("turtle")
        self.turtle = TurtleMsg()

        #### Color service client ####
        self.color_cli = self.create_client(SetColor, 'set_turtle_color')
        while not self.color_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Color service not available, waiting...')
        self.color_req = SetColor.Request()
        
        self.color_i = 0
        self.server_call = False
        #################################

        #### publisher define ####
        self.twist_pub = self.create_publisher(Twist, "turtle_car_cmd", 1)
        ##########################

        #### Travel goals action client ####
        self.action_cli = ActionClient(self, TurtleToGoals, 'turtle_to_goals')
        while not self.color_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TraveltoGoal action not available, waiting...')
        ####################################

        #### for cv ####
        self.cv_bridge = CvBridge()
        self.img_control = False

        #### subscribing img ####
        self.img_sub = self.create_subscription(Image, 'image_raw', self.img_callback, 1)

        #### subscribing turtlebot state ####
        self.turtle_sub = self.create_subscription(TurtleMsg, 'turtle_state', self.turtle_callback, 1)

    def color_srvcall(self):

        col_series = ['red', 'yellow', 'green', 'cyan', 'magenta']
        
        self.color_req.color = col_series[self.color_i]
        self.server_call = True
        self.service_future = self.color_cli.call_async(self.color_req)

        self.color_i += 1
        self.color_i %= 5
    
    def goal_actioncall(self):
        
        goal_msg = TurtleToGoals.Goal()
        for i in range(5):
            goal = Pose()
            goal.position.x = float(random.randint(-1*self.screen.window_width()/2+20,self.screen.window_width()/2-20))
            goal.position.y = float(random.randint(-1*self.screen.window_height()/2+20,self.screen.window_height()/2-20))

            qx, qy, qz, qw = quat_from_rpy(0, 0, (random.random()-0.5)*math.pi)
            goal.orientation.x = qx
            goal.orientation.y = qy
            goal.orientation.z = qz
            goal.orientation.w = qw

            goal_msg.goal_poses.append(goal)
        
        self.action_response_future = self.action_cli.send_goal_async(goal_msg, feedback_callback=self.goal_feedback_cb)
        self.action_response_future.add_done_callback(self.goal_response_cb)
    
    def goal_response_cb(self, future):

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected')
            return
        
        self.get_logger().info('Goal accepted')

        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal Travel Done! Result:%d' % (result.ret))

    def goal_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        mid_goal = feedback.mid_goal_pose
        self.get_logger().info('Traveling to x:%d, y:%d' % (mid_goal.position.x, mid_goal.position.y))

    def turtle_callback(self, msg):

        self.turtle = msg
    
    def img_callback(self, msg_img):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg_img, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().info(e)
        
        image_size=len(cv_image)*len(cv_image[0]) #get image size
        image_dimension=np.array([len(cv_image),len(cv_image[0])])    #get image dimension

        # use hsv
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Define the boundaries of "white", "yellow" and "red"
        hsv_blue1 = np.array([100, 150,0])
        hsv_blue2 = np.array([140, 255, 255])
        hsv_yellow1 = np.array([25,50,50])
        hsv_yellow2 = np.array([45,255,255])
        # The color "red" needs two set of boundaries cause it pass 255 to 0
        hsv_red1 = np.array([0,100,100])
        hsv_red2 = np.array([15,255,255])
        hsv_red3 = np.array([165,100,100])
        hsv_red4 = np.array([180,255,255])

        # detect red
        # filtered_red=cv2.inRange(cv_image,np.array([5,5,200]),np.array([200,200,255])) #filter the image with upper bound and lower bound in bgr format
        filtered_red1 = cv2.inRange(hsv_img, hsv_red1, hsv_red2)
        filtered_red2 = cv2.inRange(hsv_img, hsv_red3, hsv_red4)
        filtered_red = cv2.bitwise_or(filtered_red1, filtered_red2)
        # detect yellow
        filtered_yellow = cv2.inRange(hsv_img, hsv_yellow1, hsv_yellow2)
        # detect blue
        filtered_blue = cv2.inRange(hsv_img, hsv_blue1, hsv_blue2)
        
        
        #show filtered image
        cv2.namedWindow("Image")
        cv2.imshow("Image",cv_image)
        cv2.namedWindow("Image Red")
        cv2.imshow("Image Red",filtered_red)
        cv2.namedWindow("Image Yellow")
        cv2.imshow("Image Yellow",filtered_yellow)
        cv2.namedWindow("Image Blue")
        cv2.imshow("Image Blue",filtered_blue)
        cv2.waitKey(1)

        # unit of twist
        unit_x = 0
        unit_z = 0
        self.img_control = False

        #run color connected components to filter the counts and centroid
        def ccc_detection(img):
            retval, labels, stats, centroids=cv2.connectedComponentsWithStats(img) #run CCC on the filtered image
            idx=np.where(np.logical_and(stats[:,4]>=0.01*image_size, stats[:,4]<=0.1*image_size))[0]    #threshold the components to find the best one
            for i in idx:
                if np.linalg.norm(centroids[i]-image_dimension/2.)<50:  #threshold again, only for ones near the center
                    return True
            return False
        # red
        if ccc_detection(filtered_red):
            print("red detected")
            unit_x += 1
            self.img_control = True
        # blue
        if ccc_detection(filtered_blue):
            print("blue detected")
            unit_x -= 1
            self.img_control = True
        # yellow
        if ccc_detection(filtered_yellow):
            print("yellow detected")
            unit_z += 1
            self.img_control = True
        
        msg = Twist()
        msg.linear.x = 50.*unit_x
        msg.angular.z = 1.*unit_z
        self.twist_pub.publish(msg)

    def update(self):

        if self.turtle.color is 'None':
            self.turtle_display.penup()
        else:
            self.turtle_display.pencolor(self.turtle.color)

        self.turtle_display.setpos(self.turtle.turtle_pose.position.x, self.turtle.turtle_pose.position.y)
        
        roll, pitch, yaw = rpy_from_quat(self.turtle.turtle_pose.orientation.x,
                                        self.turtle.turtle_pose.orientation.y,
                                        self.turtle.turtle_pose.orientation.z,
                                        self.turtle.turtle_pose.orientation.w)
        self.turtle_display.seth(math.degrees(yaw))

def quat_from_rpy(roll, pitch, yaw):

    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll*0.5)
    sr = math.sin(roll*0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw

def rpy_from_quat(x, y, z, w):

    srcp = 2*(w*x + y*z)
    crcp = 1-2*(x*x + y*y)
    roll = math.atan2(srcp, crcp)

    sp = 2*(w*y - z*x)
    if math.fabs(sp) >= 1:
        pitch = (sp/math.fabs(sp))*math.pi/2
    else:
        pitch = math.asin(sp)
    
    sycp = 2*(w*z + x*y)
    cycp = 1 - 2*(y*y + z*z)
    yaw = math.atan2(sycp, cycp)

    return roll, pitch, yaw

def main(args=None):

    #keyboard setup
    fd = sys.stdin.fileno()
    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)
    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

    #initial ROS2
    rclpy.init(args=args)

    #initial turtle client
    cli_obj = TurtleClient()
    cli_obj.get_logger().info('Turtlebot Client Started!')
    cli_obj.color_srvcall()

    try:
        while rclpy.ok():
        
            cli_obj.update()
            rclpy.spin_once(cli_obj)

            unit_x = 0
            unit_z = 0
            
            try:
                #read input and print "command"
                c = sys.stdin.read()
                if "\x1b[A" in c:
                    print("drive forward")          ####Drive forward
                    unit_x = 1
                if "\x1b[B" in c:
                    print("drive backward")         ####Drive backward
                    unit_x = -1
                if "\x1b[C" in c:
                    print("drive right")            ####Drive right
                    unit_z = -1
                if "\x1b[D" in c:
                    print("drive left")             ####Drive left
                    unit_z = 1
                if "s" in c:
                    print("Color change service call")
                    cli_obj.color_srvcall()         ####Call Color change service
                if "a" in c:
                    print("Goal travel action call")
                    cli_obj.goal_actioncall()       ####Call Travel to goals action
                if "q" in c:
                    break

            except IOError: pass
            except TypeError: pass

            if cli_obj.server_call and cli_obj.service_future.done():
                cli_obj.server_call = False
                try:
                    response = cli_obj.service_future.result()
                except Exception as e:
                    cli_obj.get_logger().info('Server call failed')
                else:
                    cli_obj.get_logger().info('Server call success: %d' % (response.ret))
            
            #### publish twist ####
            if not cli_obj.img_control:
                cmd_msg = Twist()
                cmd_msg.linear.x = float(50 * unit_x)
                cmd_msg.angular.z = float(1 * unit_z)
                cli_obj.twist_pub.publish(cmd_msg)
            
    #finish reading keyboard input
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

    # Destory the node explicitly
    cli_obj.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()