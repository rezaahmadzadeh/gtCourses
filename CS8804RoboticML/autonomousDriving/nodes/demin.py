#!/usr/bin/env python
import roslib; roslib.load_manifest('vrep_vsv_driver')
import rospy
import copy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, Vector3, Point
from std_msgs.msg import Float32
from math import pi,sqrt
import tf
from tf.transformations import euler_from_quaternion

from vrep_vsv_driver.arm_ik import *

class Demin:
    def __init__(self):

