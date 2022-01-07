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

# 720*1280用
m[0] = [70,100]
m[1] = [1100,90]
m[2] = [1140,620]
m[3] = [50,620]

# # 320*640用
# m[0] = [30,20]
# m[1] = [550,15]
# m[2] = [570,285]
# m[3] = [20,290]

# フィールドの実際の大きさ
x_min,x_max=110,910
y_min,y_max=-200,200
width, height = (x_max-x_min,y_max-y_min) # 変形後画像サイズ

marker_coordinates = np.float32(m)
true_coordinates   = np.float32([[0,0],[width,0],[width,height],[0,height]])
trans_mat = cv.getPerspectiveTransform(marker_coordinates,true_coordinates)
# img_trans = cv.warpPerspective(frame,trans_mat,(width, height))
# plt.imshow(img_trans)
# print(trans_mat)

trans_mat_inv=np.linalg.inv(trans_mat)
vanish_num = 50

def msgs_to_img(img,cur_x,cur_y,orbit_predict_list,pre_positions):

    # cv.imshow('func_img', img.astype(np.uint8))

    # 薄くする
    pre_positions[:,:,:]=(pre_positions[:,:,:]>vanish_num)*(pre_positions[:,:,:]-vanish_num)


    # 現在の推定パック座標をmm単位からピクセルに逆変換
    tmp0=-cur_x+width+59
    tmp1=cur_y+height/2
    pix_x,pix_y,tmp=np.dot(trans_mat_inv,[tmp0,tmp1,1])

    # 現在の推定パック座標を描画
    # print(int(pix_x), int(pix_y))
    cv.circle(pre_positions, (int(pix_x), int(pix_y)), 30, (255, 0, 0), thickness=5)
    # cv.imshow('func_pre_positions', pre_positions.astype(np.uint8))

    # パックの予測の軌道についても点線を描画する
    for i in range(int(len(orbit_predict_list)/3)):
        predict_i_x=orbit_predict_list[i*3+0]
        predict_i_y=orbit_predict_list[i*3+1]
        tmp0=-predict_i_x+width+59
        tmp1=predict_i_y+height/2
        pix_x,pix_y,tmp=np.dot(trans_mat_inv,[tmp0,tmp1,1])
        cv.circle(pre_positions, (int(pix_x), int(pix_y)), 5, (0, 0, 255), thickness=-1)

    # 半透明にしつつ足し合わせ
    hsvimage = cv.cvtColor(pre_positions, cv.COLOR_BGR2HSV)
    img_and_positions=img*(1.-hsvimage[:,:,2]/255.)[:,:,np.newaxis]+pre_positions*(hsvimage[:,:,2]/255.)[:,:,np.newaxis]

    return img_and_positions, pre_positions

# test
if(0):
    img=cv.imread('./../../../basetest1.jpg')
    # cv.imshow('img', img)
    pre_positions=np.copy(img)
    pre_positions[:,:,:]=0
    for i in range(1000):
        cur_x,cur_y=500,(i*50)%400-200
        orbit_predict_list=[100,(100+i*50)%400-200,1,200,(100+i*50)%400-200,1,300,(100+i*50)%400-200,1]
        # orbit_predict_list=[[100,(100+i*50)%400-200],[200,(100+i*50)%400-200],[300,(100+i*50)%400-200]]
        img_and_positions, pre_positions=msgs_to_img(img,cur_x,cur_y,orbit_predict_list,pre_positions)
        cv.imshow('img_and_positions', img_and_positions.astype(np.uint8))
        cv.waitKey(1000)

    cv.destroyAllWindows()
