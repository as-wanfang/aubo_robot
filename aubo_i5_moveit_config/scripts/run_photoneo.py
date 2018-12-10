import rospy
from phoxi_camera.srv import *

while(1):
    rospy.ServiceProxy('phoxi_camera/get_frame', GetFrame)(-1)
    rospy.sleep(20)
