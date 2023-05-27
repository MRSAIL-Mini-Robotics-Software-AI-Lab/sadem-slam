#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzed.sl as sl


def stereo_camera_publisher():
    # Initialize the ROS node
    rospy.init_node('stereo_camera_publisher', anonymous=True)

    # Create publishers for left and right images
    left_image_topic = rospy.get_param("/left_image_topic", "/left_image_topic")
    right_image_topic = rospy.get_param("/right_image_topic", "/right_image_topic")
    left_image_pub = rospy.Publisher(left_image_topic, Image, queue_size=10)
    right_image_pub = rospy.Publisher(right_image_topic, Image, queue_size=10)

    # Create a CvBridge object
    bridge = CvBridge()
    use_zed = False

    # Initialize the video capture
    if use_zed:
        zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.VGA
        init_params.camera_fps = 100
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(err)
            exit(-1)
            
        imgLeft = sl.Mat()
        imgRight = sl.Mat()
    else:
        capture = cv2.VideoCapture(0)
        # capture.set(3, 672*2)
        # capture.set(4, 376)
        capture.set(3, 1280*2)
        capture.set(4, 720)
        capture.set(cv2.CAP_PROP_FPS, 60)
    
    
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        
        if use_zed:
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(imgLeft, sl.VIEW.LEFT)
                zed.retrieve_image(imgRight, sl.VIEW.RIGHT)
                
                # Convert the left and right images to ROS messages
                left_image_msg = bridge.cv2_to_imgmsg(imgLeft.get_data()[:, :, :3], encoding="bgr8")
                right_image_msg = bridge.cv2_to_imgmsg(imgRight.get_data()[:, :, :3], encoding="bgr8")

                # Publish the left and right images
                left_image_pub.publish(left_image_msg)
                right_image_pub.publish(right_image_msg)
            else:
                print("Fail")
                zed.close()
                zed = sl.Camera()
                init_params = sl.InitParameters()
                init_params.camera_resolution = sl.RESOLUTION.VGA
                init_params.camera_fps = 100
                err = zed.open(init_params)
                if err != sl.ERROR_CODE.SUCCESS:
                    print(err)
        else:
            ret, frame = capture.read()

            if ret:
                # Split the stereo image into left and right images
                width = frame.shape[1]
                half_width = width // 2
                left_image = frame[:, :half_width, :]
                right_image = frame[:, half_width:, :]
                # print(left_image.shape)
                left_image = cv2.resize(left_image, (672,376))
                right_image = cv2.resize(right_image, (672,376))

                # Convert the left and right images to ROS messages
                left_image_msg = bridge.cv2_to_imgmsg(left_image, encoding="bgr8")
                right_image_msg = bridge.cv2_to_imgmsg(right_image, encoding="bgr8")

                # Publish the left and right images
                left_image_pub.publish(left_image_msg)
                right_image_pub.publish(right_image_msg)

            
        rate.sleep()


    # Release the video capture
    zed.close()

if __name__ == '__main__':
    try:
        stereo_camera_publisher()
    except rospy.ROSInterruptException:
        pass