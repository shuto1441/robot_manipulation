#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# メッセージの型等のimport
from robot_manipulation.msg import pack_current_position
# from robot_manipulation.msg import pack_predicted_position
from std_msgs.msg import Float32MultiArray

from robot_manipulation.scripts import Orbit


class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.pub = rospy.Publisher('/pack_pdt_pos', Float32MultiArray, queue_size=10)
        # messageの型を作成
        self.msg = Float32MultiArray()

        self.orbit = Orbit(
            linearity_thresh=0.9, positional_resolution=10, max_pred_iter=100,
            floorfriction_ratio=(1.0, 1.0), wallbounce_ratio=(1.0, 1.0))

    def make_msg(self, position):
        cur_x = position.x #cur_x 現在のpackのx座標
        cur_y = position.y #cur_y 現在のpackのy座標
        cur_t = position.header.stamp

        # 処理を書く
        self.orbit.add([cur_t, cur_x, cur_y])
        preds = self.orbit.predict()
        if preds is None:
            return

        self.msg.data = preds

    def send_msg(self, position):
        # messageを送信
        self.make_msg(position)
        self.pub.publish(self.msg)

class Subscribe_publishers:
    def __init__(self, pub):
        self.pub = pub
        # Subscriberを作成
        rospy.Subscriber("/pack_cur_pos", pack_current_position, self.callback)

    def callback(self, position):
        # publish
        self.pub.send_msg(position)

def main():
    # nodeの立ち上げ
    rospy.init_node('positon_predictor')

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)

    rospy.spin()

if __name__ == '__main__':
    main()