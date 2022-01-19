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

marker_coordinates = np.float32(m)
true_coordinates   = np.float32([[x_max,y_min],[x_min,y_min],[x_min,y_max],[x_max,y_max]])
trans_mat_inv = cv.getPerspectiveTransform(true_coordinates,marker_coordinates)

vanish_num = 255

def msgs_to_img(img,cur_x,cur_y,orbit_predict_list,pre_positions):

    # 薄くする
    pre_positions[:,:,:]=(pre_positions[:,:,:]>vanish_num)*(pre_positions[:,:,:]-vanish_num)


    # 現在の推定パック座標をmm単位からピクセルに逆変換
    tmp0=cur_x
    tmp1=cur_y
    pix_x,pix_y,tmp=np.dot(trans_mat_inv,[tmp0,tmp1,1])

    # 現在の推定パック座標を描画
    cv.circle(pre_positions, (int(pix_x), int(pix_y)), 30, (255, 0, 0), thickness=5)
    
    # パックの予測の軌道についても点線を描画する
    # for i in range(int(len(orbit_predict_list)/3)):
    for xyt in orbit_predict_list:
        predict_i_x=xyt[0]
        predict_i_y=xyt[1]
        tmp0=predict_i_x
        tmp1=predict_i_y
        pix_x,pix_y,tmp=np.dot(trans_mat_inv,[tmp0,tmp1,1])
        cv.circle(pre_positions, (int(pix_x), int(pix_y)), 5, (0, 0, 255), thickness=-1)

    # 半透明にしつつ足し合わせ
    hsvimage = cv.cvtColor(pre_positions, cv.COLOR_BGR2HSV)
    img_and_positions=img*(1.-hsvimage[:,:,2]/255.)[:,:,np.newaxis]+pre_positions*(hsvimage[:,:,2]/255.)[:,:,np.newaxis]

    return img_and_positions.astype(np.uint8), pre_positions

# test
if(0):
    img=cv.imread('./../../../basetest1.jpg')
    # cv.imshow('img', img)
    pre_positions=np.copy(img)
    pre_positions[:,:,:]=0
    for i in range(1000):
        cur_x,cur_y=500,(i*50)%400-200
        orbit_predict_list=[100,(100+i*50)%400-200,1,200,(100+i*50)%400-200,1,300,(100+i*50)%400-200,1]
        img_and_positions, pre_positions=msgs_to_img(img,cur_x,cur_y,orbit_predict_list,pre_positions)
        cv.imshow('img_and_positions', img_and_positions.astype(np.uint8))
        cv.waitKey(1000)

    cv.destroyAllWindows()
