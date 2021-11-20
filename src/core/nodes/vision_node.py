#!/usr/bin/env python3

import numpy as np
import cv2

from sensor_msgs.msg import CameraInfo, Image, Point
import message_filters
import rospy
import importlib

from module import Module
#from pipeline import Pipeline
import vision

class VisionPipeline(Module):
    def __init__(self, params):
        super.__init__(self, params)

        #publisher
        rospy.init_node(params["node_name"])
        self.image_publisher = rospy.Publisher(params["topic_name"], Point)

        #subscriber
        self.image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image)
        self.info_sub = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)

        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.info_sub], 10, 0.2)
        ts.registerCallback(self.apply)

        #load vision filters
        self.load_configuration(params["vision_config"])

        self.filters = []

    def apply(self, inp):
        #read, process, write

        results = []

        for f in self.filters:
            results.append(f.apply(results[-1]))
        
        return results[-1]

    def load_configuration(self, config_path):
        with open(config_path) as config_file:
            config = json.load(config_file)

        vision_module_config = config["processors"][0]
	self.vision_module_name = vision_module_config["name"]

        vision_library = importlib.import_module("vision")
        
        for f in vision_module_config["filters"]:
            #Creating instance of module
	    module_instance = getattr(vision_library, f["name"])
	    module = module_instance(params)

            self.filters.append(module)

	    print("Module \033[92m{}\033[0m added to container".format(module_name))

if __name__ == '__main__':
    rospy.init_node('vision_node', anonymous=True)
    vision_config = rospy.get_param("vision_config")

    pipeline = VisionPipeline(vision_config)

    rospy.spin()

