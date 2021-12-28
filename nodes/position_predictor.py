#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# メッセージの型等のimport
from robot_manipulation.msg import pack_current_position
# from robot_manipulation.msg import pack_predicted_position
from std_msgs.msg import Int32MultiArray

from robot_manipulation import Orbit


class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.pub = rospy.Publisher('/pack_pdt_pos', Int32MultiArray, queue_size=10)
        self.orbit = Orbit(
            linearity_thresh=0.9, positional_resolution=10, max_pred_iter=100,
            floorfriction_ratio=(1.0, 1.0), wallbounce_ratio=(1.0, 1.0))

    def make_and_send_msg(self, position):
        cur_x = position.x #cur_x 現在のpackのx座標
        cur_y = position.y #cur_y 現在のpackのy座標
        cur_t = position.header.stamp.secs

        # 処理を書く
        self.orbit.add([cur_x, cur_y, cur_t])
        preds = self.orbit.predict()
        if preds is None or len(preds) == 0:
            print('no data')
            return
        msg = Int32MultiArray(data=preds, dim=2)
        self.pub.publish(msg)
        print(f'data len = {len(preds)}')

class Subscribe_publishers:
    def __init__(self, pub):
        self.pub = pub
        # Subscriberを作成
        rospy.Subscriber("/pack_cur_pos", pack_current_position, self.callback)

    def callback(self, position):
        # publish
        self.pub.make_and_send_msg(position)

def main():
    # nodeの立ち上げ
    rospy.init_node('positon_predictor')

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)

    rospy.spin()

if __name__ == '__main__':
    main()