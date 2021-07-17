#!/usr/bin/env python
import rospy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix, Image, PointCloud2
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from time import sleep
from cv_bridge import CvBridge
import math


class Flight_controller:
    def __init__(self):

        #Data
        self.current_position = [0.0, 0.0, 0.0]
        self.current_orientation = [0.0, 0.0, 0.707, 0.707]
        self.set_orientation = [0.0, 0.0, 0.707, 0.707]
        self.set_position = [0.0, 0.0, 0.0]
        self.set_linear_velocity = [0.0, 0.0, 0.0]
        self.set_angular_velocity = [0.0, 0.0, 0.0]
        self.current_linear_velocity = [0.0, 0.0, 0.0]
        self.current_angular_velocity = [0.0, 0.0, 0.0]
        self.gps_lat = 0
        self.gps_long = 0
        self.gps_alt = 0
        self.gps_alt_correction = 535.298913472
        self.bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.delta = 0.02
        self.waypoint_number = 0
        self.last_time = 0.0

        #Node
        rospy.init_node('iris_drone', anonymous=True)

        #subscriber
        self.gps_subscriber = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
        self.get_rgb_image = rospy.Subscriber('/r200/rgb/image_raw', Image, self.get_rgb)
        self.get_depth_image = rospy.Subscriber('/r200/depth/image_raw', Image, self.get_depth)

        #SERVICES
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        #PUBLISHERS
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

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
        sleep(2)
        t_lat = self.gps_lat
        t_long = self.gps_long
        try:
            print('gps_alt ' + str(self.gps_alt))
            if self.gps_alt > 1.0:
                print('Drone is in air')
                if self.gps_alt > t_alt:
                    return
                if self.gps_alt < t_alt:
                    t_alt -= self.gps_alt
        except:
            pass
        print(t_lat)
        print(t_long)
        
        rospy.wait_for_service('/mavros/cmd/takeoff')

        try:
            #self.takeoff_service(0.0,0.0,47.3977421, 8.5455945, t_alt)
            self.takeoff_service(0.0,0.0,t_lat, t_long, t_alt)
            sleep(5)
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

        
        try:
            self.flight_mode_service(0, 'OFFBOARD')
            #rospy.loginfo('OFFBOARD')

        except rospy.ServiceException as e:
            rospy.loginfo('OFFBOARD Mode could not be set: ' %e)

    #CALLBACKS

    def gps_callback(self, data):
        self.gps_lat = data.latitude
        self.gps_long = data.longitude
        self.gps_alt = data.altitude
        if self.gps_alt >= 530:
            self.gps_alt -= self.gps_alt_correction

    def get_pose(self, location_data):
        self.current_position[0] = location_data.pose.position.x
        self.current_position[1] = location_data.pose.position.y
        self.current_position[2] = location_data.pose.position.z
        self.current_orientation[0] = location_data.pose.orientation.x
        self.current_orientation[1] = location_data.pose.orientation.y
        self.current_orientation[2] = location_data.pose.orientation.z
        self.current_orientation[3] = location_data.pose.orientation.w

    def get_rgb(self, rgb_data):
        cv2_img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        self.rgb_image = cv2_img.copy()

    def get_depth(self, depth_data):
        cv2_img_depth = self.depth_bridge.imgmsg_to_cv2(depth_data, "32FC1")
        self.depth_image = cv2_img_depth.copy()


    #Functions

    def click_rgb_image(self):
        self.get_rgb_image

    def click_depth_image(self):
        self.get_depth_image

    '''def dist_position(self, pose1, pose2):
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dz = pose1.pose.position.z - pose2.pose.position.z
        dr = math.sqrt(dx*dx + dy*dy + dz*dz)
        return dr'''

    '''def set_pose(self):
        update_rate = rospy.Rate(20)

        raw_msg = PositionTarget()
        raw_msg.header.frame_id = "home"
        raw_msg.header.stamp = rospy.Time.now()
        raw_msg.coordinate_frame = 1
        raw_msg.type_mask  = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +PositionTarget.IGNORE_AFX +PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE
        raw_msg.position.x = self.set_position.pose.position.x
        raw_msg.position.y = self.set_position.pose.position.y
        raw_msg.position.z = self.set_position.pose.position.z
        self.publish_raw.publish(raw_msg)

        distance = self.dist_position(self.current_position, self.set_position)

        while (distance > self.delta):
            self.publish_raw.publish(raw_msg)
            self.get_pose_subscriber
            distance = self.dist_position(self.current_position, self.set_position)
            update_rate.sleep()

        self.waypoint_number += 1
        rospy.loginfo('Waypoint reached: ' + str(self.waypoint_number))'''

    def set_velocity(self):
        update_rate = rospy.Rate(20)
        val_msg = TwistStamped()

        val_msg.twist.linear.x = self.set_linear_velocity[0]
        val_msg.twist.linear.y = self.set_linear_velocity[1]
        val_msg.twist.linear.z = self.set_linear_velocity[2]

        val_msg.twist.angular.x = self.set_angular_velocity[0]
        val_msg.twist.angular.y = self.set_angular_velocity[1]
        val_msg.twist.angular.z = self.set_angular_velocity[2]
        self.vel_pub.publish(val_msg)
        #update_rate.last_time()
        self.last_time = rospy.get_time()

    def update_target_vel(self, linear, angular):
        self.set_linear_velocity = linear
        self.set_angular_velocity = angular
        if(rospy.get_time() - self.last_time < (1.0/20.0)):
            rospy.sleep(0.05 - rospy.get_time() +self.last_time)
        if not rospy.is_shutdown():
            self.set_offboard_mode()
            self.set_velocity()
            #print('done')
        if rospy.is_shutdown():
            rospy.loginfo('Connnection lost')
            self.set_offboard_mode()
            self.set_velocity()












    '''def set_waypoint(self, x, y, z):
        self.set_position.pose.position.x = x
        self.set_position.pose.position.y = y
        self.set_position.pose.position.z = z'''




'''if __name__ == '__main__':
    try:
        iris_controller = Flight_controller()
    except rospy.ROSInterruptException:
        pass

    alt = 3.0
    iris_controller.toggle_arm(True)
    iris_controller.takeoff(alt)
    iris_controller.set_offboard_mode()
    #print('offboard mode')
    iris_controller.set_linear_velocity[0] = -2
    iris_controller.set_linear_velocity[1] = 2
    #print(iris_controller.set_linear_velocity)
    iris_controller.set_velocity()'''

    
