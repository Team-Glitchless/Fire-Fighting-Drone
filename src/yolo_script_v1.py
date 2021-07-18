#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix, Image, PointCloud2
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from time import sleep
import cv2
import math
import tensorflow as tf
import ml_helper as ml
from tf.transformations import euler_from_quaternion
import numpy as np

class_names =  ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
  "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
  "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
  "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
  "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
  "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
  "banana","apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut",
  "cake","chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop",
  "mouse","remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
  "refrigerator","book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]


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
        self.img_count = 0
        
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

    def click_rgb_image(self):
        self.get_rgb_image

    def click_depth_image(self):
        self.get_depth_image


    def load_wt(self):
        self.model.load_weights('checkpoints/yolov3.tf')


    def test(self):
        self.load_wt()
        size= 416
        while True:
            self.get_rgb_image
            
            img_raw = self.rgb_image
            depth = self.depth_image
            self.tx = self.curr_x
            self.ty = self.curr_y
            self.tyaw = self.curr_yaw
            self.save_rgb()
            self.save_depth()
            img = tf.expand_dims(img_raw, 0)
            img = ml.preprocess_image(img, size)
            outputs = self.model.predict(img)

            img_out = ml.draw_outputs(img_raw, outputs)
            if(outputs[3][0] > 0):
                r = self.updater(img_raw, depth, outputs)
                
                for i in range(0, r.shape[1]):
                    if((r[0,i]) is not 'nan'):
                        print('(x,y) of ' +str(class_names[int(outputs[2][0][i])]) + ' are ' +str(r[0,i]) +', ' + str(r[1,i]))
            cv2.imshow('yolo output',img_out)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
        cv2.destroyAllWindows()

    def save_rgb(self):
        
        self.click_rgb_image()
        rgb_filename = 'drone_img/rgb/rgb_camera_image' + str(self.img_count)  + '.jpeg'
        cv2.imwrite(rgb_filename, self.rgb_image)
        #self.img_count += 1
        #print(rgb_filename + ' Saved')

    def save_depth(self):
        
        self.click_depth_image()
        depth_filename = 'drone_img/depth/depth_camera_image' + str(self.img_count)  + '.png'
        cv2.imwrite(depth_filename, self.depth_image)
        self.img_count += 1
        #print(depth_filename + ' Saved')

    def updater(self, rgb, depth, outputs):
        rgb = np.array(rgb)
        depth = np.array(depth)
        centers = center_finder(rgb, outputs)
        dz = dist_z_finder(centers, depth)
        dx = dist_x_finder(dz, rgb.shape, centers)
        r = final_pos(dz, dx, self.tx, self.ty, self.tyaw)
        return r

def center_finder(img, outputs):
    centers = []
    wh = np.flip(img.shape[0:2])
    boxes, score, classes, nums = outputs
    boxes, score, classes, nums = boxes[0], score[0], classes[0], nums[0]
    for i in range(nums):
        x1y1 = np.array((np.array(boxes[0][0:2]) * wh).astype(np.int32))
        x2y2 = np.array((np.array(boxes[0][2:4]) * wh).astype(np.int32))
        xy = (x1y1 + x2y2)/2
        centers.append(xy)
    return np.array(centers)

def dist_z_finder(center, depth):
    c = center
    j = 3
    dist = []
    for i in range(c.shape[0]):
        box = depth[c[i,1]-j:c[i,1]+j, c[i,0]-j:c[i,0]+j]
        d = np.sum(box)/box.size
        dist.append(d)
    return np.array(dist)

def dist_x_finder(z, shape_rgb, centers, h_pov = 1.3962634):
    p_fac = h_pov/shape_rgb[1]
    c_x = centers[:,0]
    px = shape_rgb[1]/2
    dr = c_x - px
    angle = np.array(dr)*p_fac
    dx = z*np.tan(angle)
    return dx

def final_pos(z, x, d_x, d_y, yaw):
    d_x += z*math.cos(yaw) + x*math.sin(yaw)
    d_y += z*math.sin(yaw) - x*math.cos(yaw)
    return np.array([d_x, d_y])




if __name__ == '__main__':
    try:
        yolo = Object_detection()
    except rospy.ROSInterruptException:
        pass

    yolo.test()
