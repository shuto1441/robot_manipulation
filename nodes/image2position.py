#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
# メッセージの型等のimport
from robot_manipulation.msg import pack_current_position
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robot_manipulation import video_capture
import cv2 as cv
import numpy as np
from robot_manipulation import CameraCalibration
from time import sleep

class Publishers():
    def __init__(self):
        # Publisherを作成
        self.pub = rospy.Publisher('/pack_cur_pos', pack_current_position, queue_size=1)
        # messageの型を作成
        self.msg = pack_current_position()

    def make_msg(self, img):
        # 引数のimgは画像のnumpy配列

        # 画像からパック座標を推定
        mm_x,mm_y=video_capture.img2mm(img)
        if np.isnan(mm_x) or np.isnan(mm_y):
            print('no data')
            return
        print(f'mm_x={mm_x}, mm_y={mm_y}')
        self.msg.x = int(mm_x) #pack x座標
        self.msg.y = int(mm_y) #pack y座標
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)

class Subscribe_publishers:
    def __init__(self, pub: 'Publishers'):
        self.pub = pub
        # Subscriberを作成
        rospy.Subscriber("usb_cam/image_raw", Image, self.callback, queue_size=1)
        self.cal = CameraCalibration('/home/ishii/dobot_ws/src/robot_manipulation/src/robot_manipulation/calibration_params.npz')
        self.publisher = rospy.Publisher('usb_cam/image_raw/calib', Image, queue_size=1)


    def callback(self, img_msg):
        try:
            bridge = CvBridge()
            img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
            img = self.cal.undistort(img)
            image_message = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        except Exception as err:
            print(err)
        # publish
        self.publisher.publish(image_message)
        self.pub.make_msg(img)

def main():
    # nodeの立ち上げ
    rospy.init_node('image2positon')

    # クラスの作成
    pub = Publishers()
    sub = Subscribe_publishers(pub)

    rospy.spin()

if __name__ == '__main__':
    main()
