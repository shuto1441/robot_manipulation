#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

import rospy
import cv2 as cv

#自作ライブラリ
from robot_manipulation import Visualization
from robot_manipulation import Orbit

# メッセージの型等のimport
from robot_manipulation.msg import pack_current_position
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robot_manipulation.msg import pack_predicted_position



class Subscribers:
    def __init__(self):
        # Subscriberを作成
        rospy.Subscriber("/pack_cur_pos", pack_current_position, self.callback2, queue_size=1)
        rospy.Subscriber("usb_cam/image_raw/calib", Image, self.callback1, queue_size=1)
        # Publisherを作成
        self.pub = rospy.Publisher('/visualized_image', Image, queue_size=10)
        # messageの型を作成
        self.image = Image()
        self.bridge = CvBridge()
        self.start = 0
        self.orbit = Orbit(
            linearity_thresh=0.9, positional_resolution=20, static_resolution=3, max_pred_iter=200,
            floorfriction_ratio=(0.99, 0.99), wallbounce_ratio=(1.0, 1.0))
        self.cur_x,self.cur_y=0,0
        self.orbit_predict_list=[]


    def callback1(self, img_msg):
        self.img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        cv.waitKey(1)
        if self.start==0: #pre_positionsを初期化(imgの大きさの空のndarrayが欲しいだけ)
            self.pre_positions=np.copy(self.img)
            self.pre_positions[:,:,:]=0
            self.start=1
        img_and_positions, self.pre_positions = Visualization.msgs_to_img(self.img, self.cur_x, self.cur_y, self.orbit_predict_list, self.pre_positions)
        self.image = self.bridge.cv2_to_imgmsg(img_and_positions, encoding="bgr8")
        self.pub.publish(self.image)

    def callback2(self, position):
        self.cur_x=int(position.x) #現在のパックの推定x座標．mm単位
        self.cur_y=int(position.y) #現在のパックの推定y座標．mm単位
        cur_t = int(position.header.stamp.secs * 1000) + int(position.header.stamp.nsecs / 1000000)
        self.orbit.add([self.cur_x, self.cur_y, cur_t])
        preds = self.orbit.predict()
        if preds is None:
            self.orbit_predict_list=[]
        else:
            self.orbit_predict_list=preds


def main():
    # nodeの立ち上げ
    rospy.init_node('visualizer')

    # クラスの作成
    sub = Subscribers()

    rospy.spin()

if __name__ == '__main__':
    main()
