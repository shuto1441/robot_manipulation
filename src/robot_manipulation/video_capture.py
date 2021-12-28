#!/usr/bin/env python
# coding: utf-8

import cv2 as cv
import numpy as np
from scipy.signal import argrelextrema
import matplotlib.pyplot as plt
import time


# frame=cv.imread('basetest1.jpg')
# plt.imshow(frame)


# 参考　https://qiita.com/code0327/items/c6e468da7007734c897f
# 時計回りで左上から順に角を定義
m = np.empty((4,2))
# m[0] = [70,100]
# m[1] = [1100,90]
# m[2] = [1140,620]
# m[3] = [50,620]

# 320*640用
m[0] = [30,20]
m[1] = [550,15]
m[2] = [570,285]
m[3] = [20,290]

# フィールドの実際の大きさ
x_min,x_max=59,859
y_min,y_max=-200,200
width, height = (x_max-x_min,y_max-y_min) # 変形後画像サイズ

marker_coordinates = np.float32(m)
true_coordinates   = np.float32([[0,0],[width,0],[width,height],[0,height]])
trans_mat = cv.getPerspectiveTransform(marker_coordinates,true_coordinates)
# img_trans = cv.warpPerspective(frame,trans_mat,(width, height))
# plt.imshow(img_trans)
# print(trans_mat)


def img2mm(img):
    frame_b=img[:,:,0]
    frame_g=img[:,:,1]
    frame_r=img[:,:,2]


    frame_extract_cian=(frame_r<150)*(frame_b>170)*(frame_g>160)
    # plt.imshow(frame_extract_cian)

    diff_sum_x=np.sum(frame_extract_cian, axis=0)
    diff_sum_y=np.sum(frame_extract_cian, axis=1)
    # 画素の濃淡で重み付けして重心を計算
    diff_sum_all=np.sum(frame_extract_cian)
    # print(diff_sum_all)
    axis_x=np.arange(diff_sum_x.size)
    axis_y=np.arange(diff_sum_y.size)
    ave_x=np.sum(diff_sum_x*axis_x)/diff_sum_all
    ave_y=np.sum(diff_sum_y*axis_y)/diff_sum_all


    pix_x=ave_x
    pix_y=ave_y
    tmp0,tmp1,tmp2=np.dot(trans_mat,[pix_x,pix_y,1])

    mm_x=width-tmp0+59
    mm_y=-height/2+tmp1
    return mm_x,mm_y
