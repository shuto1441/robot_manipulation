#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
# import sys
# LIBPATH = "../../dobot/src"
# sys.path.append(LIBPATH)
import DobotClient as dc

def control(msg):#joy to twist
    isJoint = False
    if(msg.buttons[3]):
        dc.set_jog_cmd(isJoint, 1)
    elif(msg.buttons[1]):
        dc.set_jog_cmd(isJoint, 2)
    elif(msg.buttons[0]):
        dc.set_jog_cmd(isJoint, 3)
    elif(msg.buttons[2]):
        dc.set_jog_cmd(isJoint, 4)
    elif(msg.buttons[4]):
        dc.set_jog_cmd(isJoint, 5)
    elif(msg.buttons[5]):
        dc.set_jog_cmd(isJoint, 6)
    elif(msg.buttons[6]):
        dc.set_jog_cmd(isJoint, 7)
    elif(msg.buttons[7]):
        dc.set_jog_cmd(isJoint, 8)
    elif(msg.buttons[8]):
        dc.set_home_cmd()
    elif(msg.buttons[9]):
        dc.set_jog_cmd(isJoint, 0)
    elif(msg.buttons[10]):
        print(dc.get_pose())

if __name__ == '__main__':
    #ノードの初期化
    rospy.init_node('controller')
    #購読
    rospy.Subscriber('/joy', Joy, control)
    rospy.spin()