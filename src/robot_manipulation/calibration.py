import numpy as np
import cv2

class CameraCalibration():

    def __init__(self, fpath: str) -> None:
        npz = np.load(fpath)
        self.mtx = npz['mtx']
        self.dist = npz['dist']
        self.newcameramtx = npz['newcameramtx']
        self.roi = npz['roi']

    def undistort(self, src: np.ndarray) -> np.ndarray:
        dst = cv2.undistort(src, self.mtx, self.dist, None, self.newcameramtx)
        x, y, w, h = self.roi
        dst = dst[y:y+h, x:x+w]
        return dst
