#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# メッセージの型等のimport
from robot_manipulation.msg import pack_current_position
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robot_manipulation import video_capture
import cv2 as cv
import numpy as np
class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.pub = rospy.Publisher('/pack_cur_pos', pack_current_position, queue_size=10)
        # messageの型を作成
        self.msg = pack_current_position()
        # self.start = False;
        # self.step = 1

    def make_msg(self, img):
        # 処理を書く
        # 引数のimgは画像のnumpy配列

        mm_x,mm_y=videoCapture.img2mm(img)
        self.msg.x = int(mm_x) #pack x座標
        self.msg.y = int(mm_y) #pack y座標
        self.msg.header.stamp = rospy.Time.now()

    def send_msg(self, img):
        # messageを送信
        self.make_msg(img)
        self.pub.publish(self.msg)

class Subscribe_publishers:
    def __init__(self, pub):
        self.pub = pub
        # Subscriberを作成
        rospy.Subscriber("usb_cam/image_raw", Image, self.callback)

    def callback(self, img_msg):
        try:
            bridge = CvBridge()
            img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
            #cv.imshow('image', img)
            cv.waitKey(1)
        except Exception as err:
            print(err)
        # publish
        self.pub.send_msg(img)

def main():
    # nodeの立ち上げ
    rospy.init_node('image2positon')

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)

    rospy.spin()

if __name__ == '__main__':
    main()
