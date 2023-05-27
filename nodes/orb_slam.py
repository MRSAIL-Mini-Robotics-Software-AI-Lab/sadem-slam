#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Lock
import time
import sys
import os
sys.path.append(os.path.dirname('/home/nvidia/SADEM/ORB_SLAM3/lib'))
sys.path.append(os.path.dirname('/home/nvidia/SADEM/ORB_SLAM3/lib/orbslam3_py.cpython-38-aarch64-linux-gnu.so'))

import orbslam3_py

def mutexLock(lock: Lock):
    """
    Decorator to lock a function with a given mutex
    Parameters
    ----------
    lock : threading.Lock
        Mutex lock to fetch and release
    """

    def decorator(func):
        def newFunc(*args, **kwargs):
            lock.acquire()
            val = None
            try:
                val = func(*args, **kwargs)
            finally:
                lock.release()
            return val

        return newFunc

    return decorator

class OrbSlamRos:
    lock = Lock()
    def __init__(self, left_img_topic, right_img_topic):
        rospy.Subscriber(left_img_topic, Image, self.left_img_callback)
        rospy.Subscriber(right_img_topic, Image, self.right_img_callback)

        self.left_image = None
        self.right_image = None

        self.bridge = CvBridge()
        param_path = "/home/nvidia/SADEM/catkin_ws/src/sadem_slam/configs/ZedVGA.yaml"
        self.slam = orbslam3_py.OrbSlam3Py("/home/nvidia/SADEM/ORB_SLAM3/Vocabulary/ORBvoc.txt", param_path, "stereo")
        self.last_time = None

    @mutexLock(lock)
    def left_img_callback(self, img:Image):
        self.left_image = self.bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

    @mutexLock(lock)
    def right_img_callback(self, img:Image):
        self.right_image = self.bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

    @mutexLock(lock)
    def run(self):
        if self.left_image is None or self.right_image is None:
            return
        timestamp = 0
        if self.last_time:
            timestamp = time.time() - self.last_time
        self.last_time = time.time()
        output = self.slam.TrackStereo(self.left_image, self.right_image, time.time())
        # print(output)

import numpy as np
def main():
    # Initialize the ROS node
    rospy.init_node('orb_slam', anonymous=True)

    # Create publishers for left and right images
    left_image_topic = rospy.get_param("/left_image_topic", "/left_image_topic")
    right_image_topic = rospy.get_param("/right_image_topic", "/right_image_topic")
    slam = OrbSlamRos(left_image_topic, right_image_topic)

    rate = rospy.Rate(10)  # Publish rate (10 Hz)

    while not rospy.is_shutdown():
        slam.run()
        # if slam.left_image is not None:
        #     print(slam.left_image.shape)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass