#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# メッセージの型等のimport
from robot_manipulation.msg import pack_current_position
from robot_manipulation.msg import pack_predicted_position
# from std_msgs.msg import Int32MultiArray

from robot_manipulation import Orbit
from robot_manipulation import HIT
from time import sleep


class Publishers():
    def __init__(self):
        # Publisherを作成
        self.pub = rospy.Publisher('/pack_pdt_pos', pack_predicted_position, queue_size=10)
        self.orbit = Orbit(
            linearity_thresh=0.9, positional_resolution=10, max_pred_iter=100,
            floorfriction_ratio=(1.0, 1.0), wallbounce_ratio=(1.0, 1.0))

    def make_and_send_msg(self, position):
        cur_x = position.x #cur_x 現在のpackのx座標
        cur_y = position.y #cur_y 現在のpackのy座標
        cur_t = position.header.stamp.secs

        # 処理を書く
        msg = pack_predicted_position()
        self.orbit.add([cur_x, cur_y, cur_t])
        preds = self.orbit.predict()
        data_len = len(preds)
        if preds is None or data_len == 0:
            print('no data')
            return
        print(f'data len = {data_len}')

        hit = HIT()
        condition = hit.hitCondition(preds)
        if condition is not None:
            xyt, direction =  condition
            msg.xyt = xyt
            msg.direction = direction
            self.pub.publish(msg)


class Subscribe_publishers:
    def __init__(self, pub):
        self.pub: 'Publishers' = pub
        # Subscriberを作成
        rospy.Subscriber("/pack_cur_pos", pack_current_position, self.callback)

    def callback(self, position):
        # publish
        self.pub.make_and_send_msg(position)

def main():
    # nodeの立ち上げ
    rospy.init_node('positon_predictor')

    # クラスの作成
    pub = Publishers()
    sub = Subscribe_publishers(pub)

    rospy.spin()

if __name__ == '__main__':
    main()