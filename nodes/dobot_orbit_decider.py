#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# メッセージの型等のimport
from robot_manipulation.msg import dobot_orbit
from robot_manipulation.msg import pack_predicted_position
#from std_msgs.msg import Float32MultiArray

from robot_manipulation import HIT
from time import sleep


# class Publishsers():
#     def __init__(self):
#         # Publisherを作成
#         self.pub = rospy.Publisher('/dobot_orbit_speed', dobot_orbit, queue_size=10)
#         # messageの型を作成
#         self.msg = dobot_orbit()

#     def make_msg(self, position):
#         pdt_x = position.x #cur_x 未来のpackのx座標
#         pdt_y = position.y #cur_y 未来のpackのy座標
#         pdt_t = position.header.stamp
#         # 処理を書く
#         self.msg.x = 100 + pdt_x #軌道計算後のpackのx座標
#         self.msg.y = 100 + pdt_y #軌道計算後のpackのy座標
#         self.msg.header.stamp = rospy.Time.now()
#         self.msg.velocityRatio = 1 #軌道計算後のdobotへの司令速度
#         self.msg.accelerationRatio = 1 #軌道計算後のdobotへの司令加速度

#     def send_msg(self, position):
#         # messageを送信
#         self.make_msg(position)
#         self.pub.publish(self.msg)

class Subscribers:
    def __init__(self):
        #self.pub = pub
        # Subscriberを作成
        rospy.Subscriber("/pack_pdt_pos", pack_predicted_position, self.callback)
        self.hit = HIT()

    def callback(self, orbit_predict):
        if self.hit.moving:
            return False
        else:
            xyt = orbit_predict.xyt
            direction = orbit_predict.direction
            self.hit.hitHeadon(xyt, direction)
            # self.hit.returnDobot(1)
            return True


def main():
    # nodeの立ち上げ
    rospy.init_node('dobot_orbit_decider')

    sub = Subscribers()
    rospy.spin()

if __name__ == '__main__':
    main()