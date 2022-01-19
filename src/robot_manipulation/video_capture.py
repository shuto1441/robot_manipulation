#!/usr/bin/env python
# coding: utf-8

import cv2 as cv
import numpy as np
from scipy.signal import argrelextrema
import matplotlib.pyplot as plt
import time

# 参考　https://qiita.com/code0327/items/c6e468da7007734c897f
# 時計回りで左上から順に角を定義
m = np.empty((4,2))
m[0] = [48, 31]
m[1] = [528,31]
m[2] = [525, 278]
m[3] = [46, 272]

# フィールドの実際の大きさ
x_min,x_max=110,910
y_min,y_max=-200,200
width, height = (x_max-x_min,y_max-y_min) # 変形後画像サイズ

# 透視変換行列の作成
marker_coordinates = np.float32(m)
true_coordinates   = np.float32([[x_max,y_min],[x_min,y_min],[x_min,y_max],[x_max,y_max]])
trans_mat = cv.getPerspectiveTransform(marker_coordinates,true_coordinates)

def img2mm(img):
    img = cv.blur(img, (5, 5))
    frame_b=img[:,:,0]
    frame_g=img[:,:,1]
    frame_r=img[:,:,2]

    # cian を抽出
    frame_extract_cian = (130 <= frame_b) * (frame_b <= 220) * (50 <= frame_g) * (frame_g <= 150) * (0 <= frame_r) * (frame_r <= 80)

    # 画素の濃淡で重み付けして重心を計算
    diff_sum_x=np.sum(frame_extract_cian, axis=0)
    diff_sum_y=np.sum(frame_extract_cian, axis=1)
    diff_sum_all=np.sum(frame_extract_cian)
    ave_x=0
    ave_y=0
    if(diff_sum_all!=0):
        axis_x=np.arange(diff_sum_x.size)
        axis_y=np.arange(diff_sum_y.size)
        ave_x=np.sum(diff_sum_x*axis_x)/diff_sum_all
        ave_y=np.sum(diff_sum_y*axis_y)/diff_sum_all

    mm_x,mm_y,tmp=np.dot(trans_mat,[ave_x,ave_y,1])

    return mm_x,mm_y
