#!/usr/bin/env python
from __future__ import print_function
import time

from robot_manipulation.pydobot_ros import Dobot
from robot_manipulation.orbit import Orbit
from robot_manipulation.hit import HIT
from time import sleep

import tkinter as tk


def main():
    dobot = Dobot() #dobot名の指定
    # dobot.home() #ホーミング
    dobot.speed(800, 800) #速度設定　spped(速度，加速度)
    dobot.move_to(200, 100, 0, 0) #移動(x, y, z, r)
    # dobot.wait(1000) #待機(ms)
    time.sleep(3)
    print(dobot.pose())


def test():
    curtime = int(time.time() * 1000)
    init_points = [
        [316, 117, curtime-300],
        [308, 111, curtime-200],
        [300, 105, curtime-100]
    ]
    orbit = Orbit(0.8, 20, 1000, (1.0, 1.0), (1.0, 1.0))
    for point in init_points:
        orbit.add(point)
    orbit_predict = orbit.predict()
    
    hit = HIT()
    while(True):
        condition = hit.hitCondition(orbit_predict)
        if condition is not None:
            break
        time.sleep(10/1000)
    xyt, direction =  condition
    hit.hitHeadon(xyt, direction)
    hit.returnDobot(1)

if __name__ == "__main__":
    test()