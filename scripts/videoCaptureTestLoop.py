import cv2 as cv
import numpy as np
from scipy.signal import argrelextrema
import matplotlib.pyplot as plt
import time

cap = cv.VideoCapture(0)
time.sleep(1)
ret, base = cap.read()
step=1
while True:
    step=step+1
    ret, frame = cap.read()

    # フレームを表示
    # cv.imshow('Flame', frame)
    # if cv.waitKey(1) & 0xFF == ord('q'):
    #     break
    # cv.imwrite('basetest5.jpg', frame)

    # ret, frame = cap.read()
    # 差分抽出
    # frame1=base_frame
    frame_diff=((base>frame)*(base-frame)+(base<frame)*(frame-base))
    frame_diff=(frame_diff>=10)*(frame_diff-0)
    # plt.imshow(frame_diff)
    diff_sum=np.sum(frame_diff, axis=2)
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
    if(step%10==0):
        print(step,ave_x,ave_y)
    # print(ave_x,ave_y)

    # 重心を中心とする円を描画
    cv.circle(frame, (int(ave_x),int(ave_y)), 100, (255, 0, 0), thickness=5)
    cv.imshow('Flame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
