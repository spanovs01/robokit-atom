#!/usr/bin/env python3

import numpy as np
import cv2

from sensor_msgs.msg import CameraInfo, Image, Point
import message_filters
import rospy

from pipeline import Pipeline
import vision

if __name__ == '__main__':
    rospy.init_node('vision_node', anonymous=True)

    image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image)
    info_sub = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)

    pipeline = Pipeline()

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.2)
    ts.registerCallback(pipeline.callback)

    rospy.spin()
