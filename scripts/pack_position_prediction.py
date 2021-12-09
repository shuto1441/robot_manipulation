#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# メッセージの型等のimport
from robot_manipulation.msg import pack_current_position
from robot_manipulation.msg import pack_predicted_position


class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.pub = rospy.Publisher('/pack_pdt_pos', pack_predicted_position, queue_size=10)
        # messageの型を作成
        self.msg = pack_predicted_position()

    def make_msg(self, position):
        cur_x = position.x #cur_x 現在のpackのx座標
        cur_y = position.y #cur_y 現在のpackのy座標
        cur_t = position.header.stamp
        # 処理を書く
        self.msg.x = 100 + cur_x #未来のpackのx座標
        self.msg.y = 100 + cur_y #未来のpackのy座標
        self.msg.header.stamp = rospy.Time.now()

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
    rospy.init_node('pack_positon_predictor')

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)

    rospy.spin()

if __name__ == '__main__':
    main()