#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from robot_manipulation.msg import pack_current_position
from robot_manipulation.msg import pack_predicted_position

from robot_manipulation import Orbit
from robot_manipulation import HIT


class Publishers():

    def __init__(self):
        self.pub = rospy.Publisher(
            '/pack_pdt_pos', pack_predicted_position, queue_size=1)
        self.orbit = Orbit(
            linearity_thresh=0.9, positional_resolution=20, static_resolution=3,
            max_pred_iter=200, floorfriction_ratio=(0.99, 0.99),
            wallbounce_ratio=(1.0, 1.0))
        self.hit = HIT(1)

    def make_and_send_msg(self, position):
        cur_x = position.x  # cur_x 現在のpackのx座標
        cur_y = position.y  # cur_y 現在のpackのy座標
        cur_t = int(position.header.stamp.secs * 1000) \
                + int(position.header.stamp.nsecs / 1000000)

        msg = pack_predicted_position()
        self.orbit.add([cur_x, cur_y, cur_t])  # 観測座標追加
        preds = self.orbit.predict()           # 軌道予測

        if preds is None:
            print('none')
            return

        if len(preds) == 0:
            print('no data')
            return

        condition = self.hit.hitCondition(preds)
        if condition is not None:        # Dobotの可動範囲内に予測地点が存在する場合
            xyt, direction =  condition  # 打ち返し座標，時刻と方向ベクトル
            print('xyt is ' + str(xyt))
            msg.xyt = xyt
            msg.direction = direction
            self.pub.publish(msg)
        else:
            print("condition is None")


class Subscribe_publishers:

    def __init__(self, pub):
        self.pub: 'Publishers' = pub
        rospy.Subscriber(
            "/pack_cur_pos", pack_current_position, self.callback, queue_size=1)

    def callback(self, position):
        self.pub.make_and_send_msg(position)


def main():
    rospy.init_node('positon_predictor')

    pub = Publishers()
    sub = Subscribe_publishers(pub)

    rospy.spin()


if __name__ == '__main__':
    main()
