#!/usr/bin/env python
import rospy
import math
from pose_estimation.msg import ObjectHypothesis
from visualization_msgs.msg import Marker

topic = 'object_pose_marker'
publisher = rospy.Publisher(topic, Marker, queue_size=1)

def scale_original_model(label):

    if label == 'kleenex_paper_towels':
        scalex = 0.15288*2 
        scaley = 0.0737*2 
        scalez = 0.0674*2 
    
    return  scalex,scaley,scalez


def callback(data):
    print "*****In callback*****"
    centerX = 0 #data.mean.x 
    centerY = 0 #data.mean.y 
    centerZ = 0 #data.mean.z 
    
    transX = data.pose.position.x
    transY = data.pose.position.y
    transZ = data.pose.position.z

    orientationX = data.pose.orientation.x
    orientationY = data.pose.orientation.y
    orientationZ = data.pose.orientation.z
    orientationW = data.pose.orientation.w

    scaleX,scaleY,scaleZ = scale_original_model(data.label) # scale is from ply model
    
    # Make marker msg and publish
    marker = Marker()
    marker.header.frame_id = "/camera_rgb_optical_frame"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.15288*2 #scaleX
    marker.scale.y = 0.0737*2 #scaleY
    marker.scale.z = 0.0674*2 #scaleZ
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.3
    marker.pose.orientation.x = orientationX
    marker.pose.orientation.y = orientationY
    marker.pose.orientation.z = orientationZ
    marker.pose.orientation.w = orientationW
    marker.pose.position.x = centerX - transX/2
    marker.pose.position.y = centerY - transY
    marker.pose.position.z = centerZ + transZ

    publisher.publish(marker)


    


def listener():

    rospy.init_node('marker_pub', anonymous=True)

    rospy.Subscriber("object_hypothesis", ObjectHypothesis, callback)


if __name__ == '__main__':
    listener()
    rospy.spin()

