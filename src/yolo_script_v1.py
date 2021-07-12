#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix, Image, PointCloud2
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from time import sleep
import cv2
print('importing tf')
import tensorflow as tf
import ml_helper as ml
from tf.transformations import euler_from_quaternion


class Object_detection:
    def __init__(self):
        
        self.curr_x = 0
        self.curr_y = 0
        self.curr_z = 0
        self.curr_roll = 0.0
        self.curr_pitch = 0.0
        self.curr_yaw = 0.0

        self.bridge = CvBridge()
        self.depth_bridge = CvBridge()


        self.model = ml.YoloV3()

        
        #NODE
        rospy.init_node('object_detection', anonymous=True)

        #SUBSCRIBERS
        self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
        self.get_rgb_image = rospy.Subscriber('/r200/rgb/image_raw', Image, self.get_rgb)
        self.get_depth_image = rospy.Subscriber('/r200/depth/image_raw', Image, self.get_depth)


    def get_pose(self, location_data):
        self.curr_x = location_data.pose.position.x
        self.curr_y = location_data.pose.position.y
        self.curr_z = location_data.pose.position.z
        rot_q = location_data.pose.orientation
        (self.curr_roll ,self.curr_pitch ,self.curr_yaw)=euler_from_quaternion([rot_q.x ,rot_q.y,rot_q.z ,rot_q.w])

    def get_rgb(self, rgb_data):
        cv2_img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        self.rgb_image = cv2_img.copy()

    def get_depth(self, depth_data):
        cv2_img_depth = self.depth_bridge.imgmsg_to_cv2(depth_data, "32FC1")
        self.depth_image = cv2_img_depth.copy()


    def load_wt(self):
        self.model.load_weights('checkpoints/yolov3.tf')


    def test(self):
        self.load_wt()
        size= 416
        while True:
            self.get_rgb_image
            
            img_raw = self.rgb_image
            img = tf.expand_dims(img_raw, 0)
            img = ml.preprocess_image(img, size)
            outputs = self.model.predict(img)

            img_out = ml.draw_outputs(img_raw, outputs)

            cv2.imshow('yolo output',img_out)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
        cv2.destroyAllWindows()



if __name__ == '__main__':
    try:
        yolo = Object_detection()
    except rospy.ROSInterruptException:
        pass

    yolo.test()
