#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sensor_stick.srv import GetNormals
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from franka_description.srv import *
from rospy_message_converter import message_converter
import yaml
import time

import message_filters
from gqcnn.srv import GQCNNGraspPlanner
from gqcnn.msg import BoundingBox
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import tf2_ros
import tf2_geometry_msgs

import sys
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg

from io_interface import *

import rospy
from phoxi_camera.srv import *

#height: 576
#width: 1024
#distortion_model: plumb_bob
#K: [549.5941880572811, 0.0, 512.5, 0.0, 549.5941880572811, 288.5, 0.0, 0.0, 1.0]
# gqcnn 640*480

# camera matrix of photoneo model S with half resolution
global camera_info
camera_info = CameraInfo()
camera_info.K = [1137.29, 0.0, 524.67, 0.0, 1136.43, 404.61, 0.0, 0.0, 1.0]
camera_info.header.frame_id = 'PhoXi3Dscanner_sensor'
camera_info.height = 772
camera_info.width = 1032

global place_joint_positions
place_joint_positions = [-97.25/180*3.14, 2.0/180*3.14, 125.8/180*3.14, 34.1/180*3.14, 96.1/180*3.14, 12.7/180*3.14]


def pcl_callback(pcl_msg):
    pass

def callback(depth, rgb):
    bounding_box = BoundingBox()
    bounding_box.minX = 0
    bounding_box.minY = 0
    bounding_box.maxX = 1032
    bounding_box.maxY = 772

    print "*************************************************"
    # print ("camera_info", camera_info.K)

    rospy.wait_for_service('plan_gqcnn_grasp')

    try:
        plan_routine = rospy.ServiceProxy('plan_gqcnn_grasp', GQCNNGraspPlanner)
        resp = plan_routine(rgb, depth, camera_info, bounding_box)
        grasp = resp.grasp
        grasp.pose.position.x = grasp.pose.position.x/1000
        grasp.pose.position.y = grasp.pose.position.y/1000
        grasp.pose.position.z = grasp.pose.position.z/1000
        print "*************************************************"
        print ("Pose: ", grasp)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    listener = tf2_ros.TransformListener(tfBuffer)
    try:
        trans = tfBuffer.lookup_transform('world', 'PhoXi3Dscanner_sensor', rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print "Service call failed: %s"%e
    pose_transformed = tf2_geometry_msgs.do_transform_pose(grasp, trans)
    print "*************************************************"
    print ("transform matrix: ", trans)
    print ("pose_transformed: ", pose_transformed)
    pose_pub.publish(pose_transformed)

    pose_transformed.pose.position.z = pose_transformed.pose.position.z + 0.05
    group.set_pose_target(pose_transformed, end_effector_link='ee_link')
    plan = group.plan()
    group.execute(plan)

    pose_transformed.pose.position.z = pose_transformed.pose.position.z - 0.055
    group.set_pose_target(pose_transformed, end_effector_link='ee_link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)
    set_digital_out(0, False)
    time.sleep(1)

    pose_transformed.pose.position.z = pose_transformed.pose.position.z + 0.05
    group.set_pose_target(pose_transformed, end_effector_link='ee_link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)
    set_digital_out(0, True)
    time.sleep(1)

    group.set_joint_value_target(place_joint_positions)
    plan = group.plan()
    group.execute(plan)
    time.sleep(5)
    rospy.ServiceProxy('phoxi_camera/get_frame', GetFrame)(-1)

# function to load parameters and request PickPlace service

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('perception', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)
    global group, robot, scene
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator_i5")
    group.set_planner_id('RRTConnectkConfig1')
    group.set_num_planning_attempts(5)
    group.set_planning_time(5)
    group.set_max_velocity_scaling_factor(0.5)
    group.set_joint_value_target(place_joint_positions)
    plan = group.plan()
    group.execute(plan)
    time.sleep(5)

    rospy.ServiceProxy('phoxi_camera/get_frame', GetFrame)(-1)
    time.sleep(5)

    get_states()
    set_states()
    print "listener has been activated"

    # TODO: Create Subscribers
    depth_sub = message_filters.Subscriber("/phoxi_camera/depth_map", Image)
    rgb_sub = message_filters.Subscriber("/phoxi_camera/rgb_texture", Image)
    pose_pub = rospy.Publisher("/grasp_pose", PoseStamped, queue_size=1)

    ts = message_filters.TimeSynchronizer([depth_sub, rgb_sub], 1)
    ts.registerCallback(callback)

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
