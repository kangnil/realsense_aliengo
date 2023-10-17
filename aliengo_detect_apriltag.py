#! /usr/bin/env python
import os
import torch
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
import dt_apriltags
import numpy as np
import sys
import time
import math
from dt_apriltags import Detector
from cv2 import imshow
import matplotlib.pyplot as plt
import copy
 
#  sudo kill -9 $(sudo lsof -t -i:8081-8082)

def printcallback(data):
    print(data)

def project_image_points_to_world_torch2(img_points,homo):
    world_points = []
    print(homo)
    inv_camera_matrix = torch.linalg.inv(homo)
    # Transform each image point to a world point
    for img_point in img_points:
        point_world = torch.matmul(inv_camera_matrix, img_point)
        point_world=point_world/point_world[2]
        world_points.append(point_world.reshape((-1,3)))
    return torch.vstack(world_points)

def process_paint(tag):
    grid_2d_np = np.load('/home/kangni/Pictures/star.npy')
    grid_2d = torch.from_numpy(grid_2d_np)
    grid_2d_w = torch.cat([grid_2d, torch.ones((len(grid_2d), 1))], dim=1).double()
    print("grid 2d {}".format(grid_2d_w))
    homo1=torch.from_numpy(np.array(tag.homography))
    output2=project_image_points_to_world_torch2(grid_2d_w,homo1)
    points_in_tag0= output2*0.067/2

    #np.save("/home/kangni/Pictures/predicted.npy", points_in_tag0.numpy())
    #print(tag)
    #time.sleep(5000)
    result = np.zeros((points_in_tag0.shape[0], 3))
    result[:, 0]=points_in_tag0[:,0]
    result[:, 2]=points_in_tag0[:,1]
    print(result)
    np.save("/home/kangni/Pictures/star_res.npy",result)
    print("Done")
    time.sleep(5000)
    return result

 
def callbackgood(data1):
    #pub.publish("I got an image")
    torch.set_printoptions(sci_mode=False)
    bridge = CvBridge()
    color_image = bridge.imgmsg_to_cv2(data1, 'bgr8')

    global fx, fy, cx, cy
    tags = at_detector.detect(color_image[:,:,0], estimate_tag_pose=True, camera_params=[fx,fy, cx, cy], tag_size=0.067)
    #print(tags)
    if len(tags)>0:
        tag = tags[0]
        #print(tag)
        #process_paint( tag)
        if tag.pose_t[2]>0.3:
            pub.publish("forward")
            rospy.loginfo("forward")
        elif tag.pose_t[2]<0.3:
            pub.publish("backward")
            rospy.loginfo("backward")
        else:
            pub.publish("step")
            rospy.loginfo("step")

    # Display the annotated frame
    # cv2.imwrite(savingName, annotated_frame)
    cv2.imshow("find apriltag", color_image)
    cv2.waitKey(10)
 

if __name__ == '__main__':
    
 
    #K: [606.4332275390625, 0.0, 328.3807678222656, 0.0, 606.4595947265625, 257.0664367675781, 0.0, 0.0, 1.0]
 
    at_detector = dt_apriltags.Detector(searchpath=['apriltags'],
                       families='tag25h9',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

    global fx, fy, cx, cy
    fx = 606.4332275390625
    fy = 606.4595947265625
    cx = 328.3807678222656
    cy = 257.0664367675781

    
    rospy.init_node('get_image', anonymous=True)
    color = message_filters.Subscriber("/camera/color/image_raw", Image)
    color.registerCallback(callbackgood)
    
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10)
    #caminfo = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo)
    #caminfo.registerCallback(printcallback)
    # color_depth = message_filters.ApproximateTimeSynchronizer([color, depth], 10, 1, allow_headerless=True)  # 接近时间同步
    # color_depth = message_filters.TimeSynchronizer([color, depth], 1)  # 绝对时间同步
    # color_depth.registerCallback(callback)
    # 同时订阅/camera/color/image_raw和/camera/aligned_depth_to_color/image_raw话题，并利用message_filters实现话题同步，共同调用callback
    rospy.spin()
   
