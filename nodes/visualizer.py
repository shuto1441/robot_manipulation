#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

# 複数のtopicを読みたい　https://qiita.com/nabion/items/319d4ffdc3d87bfb0076
import message_filters

import rospy
import cv2 as cv

#自作ライブラリ
from robot_manipulation import Visualization

# メッセージの型等のimport
from robot_manipulation.msg import pack_current_position
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robot_manipulation.msg import pack_predicted_position
# from std_msgs.msg import Int32MultiArray

start=0

def callback(img_msg, position, orbit_predict):
    print('ok')
    try:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg, "bgr8")

        if start==0: #pre_positionsを初期化(imgの大きさの空のndarrayが欲しいだけ)
            pre_positions=np.copy(img)
            pre_positions[:,:,:]=0
            start=1

        cur_x=int(position.x) #現在のパックの推定x座標．mm単位
        cur_y=int(position.y) #現在のパックの推定y座標．mm単位
        orbit_predict_list = list(orbit_predict.xyt) #現在のパックの予測軌道．mm単位．#int64のリスト
        # img_and_positions: カメラ画像に過去および現在の推定座標と予測軌道を重ねたもの. ndarray
        # pre_positions: 過去の推定座標と推定軌道が作図された図．古いものほど薄くなっている．ndarray
        img_and_positions, pre_positions = Visualization.msgs_to_img(img,cur_x,cur_y,orbit_predict_list,pre_positions)
        print('1')
        cv.imshow('image', img)
        cv.waitKey(1)
    except Exception as err:
        print(err)

def main():
    # nodeの立ち上げ
    rospy.init_node('visualizer')

    # Subscriberを作成
    sub1 = message_filters.Subscriber("usb_cam/image_raw", Image)
    sub2 = message_filters.Subscriber("/pack_cur_pos", pack_current_position)
    sub3 = message_filters.Subscriber("/pack_pdt_pos", pack_predicted_position)


    queue_size = 10
    fps = 30.
    delay = 1 / fps * 1.5

    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3], queue_size, delay)
    print('ok2')
    mf.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    main()
