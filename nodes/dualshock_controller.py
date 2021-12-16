#! /usr/bin/env python

from re import L
import rospy
from sensor_msgs.msg import Joy
from dobot.srv import *


class Subscribers():
    def __init__(self):
        # Subscriberを作成
        self.subscriber = rospy.Subscriber('/joy', Joy, self.callback)
            # messageの型を作成
        self.joy = Joy()
        self.right_status = 0
        self.left_status = 0
        # l_horizontal = joy.axes[0]
        # l_vertical = joy.axes[1]
        # r_horizontal = joy.axes[2]
        # r_vertical = joy.axes[5]
        # axis_horizontal = joy.axes[9]
        # axis_vertical = joy.axes[10]
        # button_Sq = joy.buttons[0]
        # button_X = joy.buttons[1]
        # button_O = joy.buttons[2]
        # button_Tr = joy.buttons[3]
        # button_Touch = joy.buttons[13]
        # button_sh = joy.buttons[8]
        # button_op = joy.buttons[9]
        # button_PS = joy.buttons[12]
        # l1 = joy.buttons[4]
        # r1 = joy.buttons[5]

    def callback(self, joy):
        r_horizontal = joy.axes[2]
        r_vertical = joy.axes[5]
        l_horizontal = joy.axes[0]
        l_vertical = joy.axes[1]
        print(l_vertical)
        isJoint = False
        r_vec, r_speed = self.select_ver_hor(r_horizontal, r_vertical)
        l_vec, l_speed = self.select_ver_hor(l_horizontal, l_vertical)
        if r_speed == 0 and self.right_status != 0:
            self.run_dobot_right(SetJOGCmd, isJoint, 0)
        else:
            if r_vec == 1:
                if r_speed > 0 and self.right_status != 4:
                    #self.run_dobot_right(SetJOGCoordinateParams, r_speed, 0, True)
                    self.run_dobot_right(SetJOGCmd, isJoint, 4)
                elif self.right_status != 3:
                    #self.run_dobot_right(SetJOGCoordinateParams, -r_speed, 0, True)
                    self.run_dobot_right(SetJOGCmd, isJoint, 3)
            else:
                if r_speed > 0 and self.right_status != 1:
                    #self.run_dobot_right(SetJOGCoordinateParams, r_speed, 0, True)
                    self.run_dobot_right(SetJOGCmd, isJoint, 1)
                elif self.right_status != 2:
                    #self.run_dobot_right(SetJOGCoordinateParams, -r_speed, 0, True)
                    self.run_dobot_right(SetJOGCmd, isJoint, 2)
        if l_speed == 0 and self.left_status != 0:
            self.run_dobot_left(SetJOGCmd, isJoint, 0)
        else:
            if l_vec == 1:
                if l_speed > 0 and self.left_status != 4:
                    #self.run_dobot_left(SetJOGCoordinateParams, l_speed, 0, True)
                    self.run_dobot_left(SetJOGCmd, isJoint, 4)
                elif self.left_status != 3:
                    #self.run_dobot_left(SetJOGCoordinateParams, -l_speed, 0, True)
                    self.run_dobot_left(SetJOGCmd, isJoint, 3)
            else:
                if l_speed > 0 and self.left_status != 1:
                    #self.run_dobot_left(SetJOGCoordinateParams, l_speed, 0, True)
                    self.run_dobot_left(SetJOGCmd, isJoint, 1)
                elif self.left_status != 2:
                    #self.run_dobot_left(SetJOGCoordinateParams, -l_speed, 0, True)
                    self.run_dobot_left(SetJOGCmd, isJoint, 2)

    def run_dobot_left(self, command, *args, **kwargs):
        rospy.wait_for_service('/dobot1/{}'.format(command.__name__))
        cmd = rospy.ServiceProxy('/dobot1/{}'.format(command.__name__), command)
        response = cmd(*args, **kwargs)
        return response

    def run_dobot_right(self, command, *args, **kwargs):
        rospy.wait_for_service('/dobot2/{}'.format(command.__name__))
        cmd = rospy.ServiceProxy('/dobot2/{}'.format(command.__name__), command)
        response = cmd(*args, **kwargs)
        return response


    def select_ver_hor(self, horizontal, vertical):
            if abs(horizontal) > abs(vertical):
                speed = self.set_active(horizontal)
                return 1, speed
            else:
                speed = self.set_active(vertical)
                return -1, speed

    def set_active(self, input):
        if input < 0.1:
            return 0
        else:
            return input
def main():
    # nodeの立ち上げ
    rospy.init_node('Node_name')

    # クラスの作成
    sub = Subscribers()
    rospy.spin()

if __name__ == '__main__':
    main()
