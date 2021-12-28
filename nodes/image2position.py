#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# メッセージの型等のimport
from robot_manipulation.msg import pack_current_position
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.pub = rospy.Publisher('/pack_cur_pos', pack_current_position, queue_size=10)
        # messageの型を作成
        self.msg = pack_current_position()
        self.start = False;
        self.step = 1

    def make_msg(self, img):
        # 処理を書く
        # 引数のimgは画像のnumpy配列
        self.step += 1
        if not self.start:
            self.base_img = img
            self.start = True
        frame_diff=((self.base_img > img)*(self.base_img - img)+(self.base_img < img) * (img-self.base_img))
        frame_diff=(frame_diff>=10)*(frame_diff-0)
        # plt.imshow(frame_diff)
        # diff_sum=np.sum(frame_diff, axis=2)
        # plt.imshow(diff_sum)

        median = cv.medianBlur(frame_diff,31)
        # median = cv.medianBlur(frame_diff,1)

        # x,y方向それぞれ畳み込む
        median=np.sum(median, axis=2)
        diff_sum_x=np.sum(median, axis=0)
        diff_sum_y=np.sum(median, axis=1)

        # 画素の濃淡で重み付けして重心を計算
        diff_sum_all=np.sum(median)
        if(diff_sum_all==0):
            diff_sum_all=1
        # print(diff_sum_all)
        axis_x=np.arange(diff_sum_x.size)
        axis_y=np.arange(diff_sum_y.size)
        ave_x=np.sum(diff_sum_x*axis_x)/diff_sum_all
        ave_y=np.sum(diff_sum_y*axis_y)/diff_sum_all
        if(self.step%10==0):
            print(self.step, ave_x, ave_y)
        # print(ave_x,ave_y)

        # 重心を中心とする円を描画
        cv.circle(img, (int(ave_x),int(ave_y)), 100, (255, 0, 0), thickness=5)
        self.msg.x = int(ave_x) #pack x座標
        self.msg.y = int(ave_y) #pack y座標
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