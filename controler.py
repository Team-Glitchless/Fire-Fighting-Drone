#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix, Image
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from time import sleep
from cv_bridge import CvBridge, CvBridgeError

class Flight_controller:
    def __init__(self):
        
        #data
        self.gps_lat = 0
        self.gps_long = 0

        self.curr_x = 0
        self.curr_y = 0
        self.curr_z = 0
        self.curr_ori_x = 0
        self.curr_ori_y = 0
        self.curr_ori_z = 0.707
        self.curr_ori_w = 0.707

        self.set_x = 0
        self.set_y = 0
        self.set_z = 0
        self.set_ori_x = 0
        self.set_ori_y = 0
        self.set_ori_z = 0.707
        self.set_ori_w = 0.707

        self.bridge = CvBridge()
        self.depth_bridge = CvBridge()

        self.delta = 0.05
        self.delta_z = 0.1
        self.waypoint_number = 0


        #NODE
        rospy.init_node('iris_drone', anonymous=True)

        #SUBSCRIBERS
        self.gps_subscriber = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
        self.get_rgb_image = rospy.Subscriber('/camera/rgb/image_raw', Image, self.get_rgb)
        self.get_depth_image = rospy.Subscriber('/camera/depth/image_raw', Image, self.get_depth)

        #PUBLISHERS
        self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)

        #SERVICES
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.loginfo('INIT')

    #Mode setup

    def toggle_arm(self, arm_bool):

        rospy.wait_for_service('/mavros/cmd/arming')

        try:
            self.arm_service(arm_bool)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " %e)

    def takeoff(self, t_alt):
        self.gps_subscriber

        t_lat = self.gps_lat
        t_long = self.gps_long

        
        rospy.wait_for_service('/mavros/cmd/takeoff')

        try:
            self.takeoff_service(0.0,0.0,47.3977421, 8.5455945, t_alt)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " %e)

    def land(self, t_alt):
        self.gps_subscriber

        t_lat = self.gps_lat
        t_long = self.gps_long

        rospy.wait_for_service('/mavros/cmd/land')

        try:
            self.land_service(0.0,0.0, t_lat, t_long, t_alt)
            rospy.loginfo('LANDING')
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " %e)

    def set_offboard_mode(self):
        rate = rospy.Rate(20)

        rospy.wait_for_service('/mavros/set_mode')

        PS = PoseStamped()

        PS.pose.position.x = 0
        PS.pose.position.y = 0
        PS.pose.position.z = 3.37

        for i in range (0,100):
            self.publish_pose.publish(PS)
            rate.sleep()
        
        try:
            self.flight_mode_service(0, 'OFFBOARD')
            rospy.loginfo('OFFBOARD')

        except rospy.ServiceException as e:
            rospy.loginfo('OFFBOARD Mode could not be set: ' %e)

    #CALLBACKS

    def gps_callback(self, data):
        self.gps_lat = data.latitude
        self.gps_long = data.longitude

    def get_pose(self, location_data):
        self.curr_x = location_data.pose.position.x
        self.curr_y = location_data.pose.position.y
        self.curr_z = location_data.pose.position.z
        self.curr_ori_x = location_data.pose.orientation.x
        self.curr_ori_y = location_data.pose.orientation.y
        self.curr_ori_z = location_data.pose.orientation.z
        self.curr_ori_w = location_data.pose.orientation.w


    def click_rgb_image(self):
        self.get_rgb_image

    def click_depth_image(self):
        self.get_depth_image


    def get_rgb(self, rgb_data):
        cv2_img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        self.rgb_image = cv2_img.copy()

    def get_depth(self, depth_data):
        cv2_img_depth = self.depth_bridge.imgmsg_to_cv2(depth_data, "32FC1")
        self.depth_image = cv2_img_depth.copy()

    
    def set_pose(self):
        update_rate = rospy.Rate(20)

        PS = PoseStamped()

        PS.pose.position.x = self.set_x
        PS.pose.position.y = self.set_y
        PS.pose.position.z = self.set_z

        PS.pose.orientation.x = self.set_ori_x
        PS.pose.orientation.y = self.set_ori_y
        PS.pose.orientation.z = self.set_ori_z
        PS.pose.orientation.w = self.set_ori_w

        distance = math.sqrt((self.set_x - self.curr_x)**2 + (self.set_y - self.curr_y)**2 + (self.set_z - self.curr_z)**2)

        while (distance > self.delta):
            self.publish_pose.publish(PS)
            self.get_pose_subscriber
            distance = math.sqrt((self.set_x - self.curr_x)**2 + (self.set_y - self.curr_y)**2 + (self.set_z - self.curr_z)**2)
            update_rate.sleep()

        self.waypoint_number += 1
        rospy.loginfo('Waypoint reached: ' + str(self.waypoint_number))



    #Testing

    def set_waypoint(self, x,y,z):
        self.set_x = x
        self.set_y = y
        self.set_z = z

    def move_to(self, x, y, z):
        self.set_waypoint(x,y,z)
        self.set_pose()

    def test_control(self):
        self.toggle_arm(True)
        self.takeoff(2.0)

        self.set_offboard_mode()
        self.move_to(1,1,3.3)
        self.click_depth_image()
        self.click_rgb_image()
        sleep(2)






if __name__ == '__main__':

    try:
        iris_controller = Flight_controller()

        iris_controller.test_control()
    

    except rospy.ROSInterruptException:
        pass