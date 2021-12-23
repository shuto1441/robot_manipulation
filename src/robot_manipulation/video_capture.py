#!/usr/bin/env python
# coding: utf-8

# In[1]:


import cv2 as cv
import numpy as np
from scipy.signal import argrelextrema
import matplotlib.pyplot as plt


# In[2]:



# VideoCapture オブジェクトを取得します
capture = cv.VideoCapture(0)
for i in range(10):
    ret, base_frame = capture.read()
#     cv.imshow('base_frame',base_frame)
#     cv.waitKey(100)
capture.release()
# cv.destroyAllWindows()
plt.imshow(base_frame)


# In[3]:



# VideoCapture オブジェクトを取得します
capture = cv.VideoCapture(0)
for i in range(10):
    ret, frame2 = capture.read()
#     cv.imshow('base_frame',base_frame)
#     cv.waitKey(100)
capture.release()
# cv.destroyAllWindows()
plt.imshow(frame2)


# In[4]:


# 差分抽出
frame1=base_frame
frame_diff=((frame1>frame2)*(frame1-frame2)+(frame1<frame2)*(frame2-frame1))
frame_diff=(frame_diff>=10)*(frame_diff-0)
plt.imshow(frame_diff)
diff_sum=np.sum(frame_diff, axis=2)
plt.imshow(diff_sum)

# # ガウス平滑化してから極大箇所抽出
# blur = cv.GaussianBlur(frame_diff,(101,101),0)
# # plt.imshow(blur)
# diff_sum=np.sum(blur, axis=2)
# plt.imshow(diff_sum)
# maxInd = argrelextrema(diff_sum, np.greater)
# print(maxInd)
# # cv2.circle(img, center, radius, color, thickness=1, lineType=cv2.LINE_8, shift=0)
# print(maxInd[0].size)


# In[5]:


median = cv.medianBlur(frame_diff,81)
plt.imshow(median)


# In[6]:


# x,y方向それぞれ畳み込む
median=np.sum(median, axis=2)
diff_sum_x=np.sum(median, axis=0)
diff_sum_y=np.sum(median, axis=1)
# diff_sum_x=np.sum(diff_sum, axis=0)
# diff_sum_y=np.sum(diff_sum, axis=1)
# print(diff_sum_x.size)
# print(diff_sum_y.size)
# print(diff_sum_x)
# print(diff_sum_y)


# In[7]:


# 画素の濃淡で重み付けして重心を計算
diff_sum_all=np.sum(median)
print(diff_sum_all)
axis_x=np.arange(diff_sum_x.size)
axis_y=np.arange(diff_sum_y.size)
ave_x=np.sum(diff_sum_x*axis_x)/diff_sum_all
ave_y=np.sum(diff_sum_y*axis_y)/diff_sum_all
print(ave_x,ave_y)


# In[8]:


# 重心を中心とする円を描画
cv.circle(frame2, (int(ave_x),int(ave_y)), 100, (255, 0, 0), thickness=5)
plt.imshow(frame2)


# In[9]:


# ハフ変換による円検出も有効？
# 色で検出？
# いっそ教師画像頑張って作って機械学習する方が楽？


# In[ ]:




