#!/usr/bin/env python
from __future__ import print_function

from robot_manipulation.pydobot_ros import Dobot
from robot_manipulation.orbit import Orbit
from robot_manipulation.hit import HIT
from time import sleep

import tkinter as tk


def main():
    global pose_pre
    #dobot名の指定
    dobot = Dobot()
    #ホーミング
    # dobot.home()
    #速度設定　spped(速度，加速度)
    dobot.speed(400, 400)
    #移動(x, y, z, r)
    dobot.move_to(200, 0, 0, 0)
    # dobot.move_to(200, 200, 0, 0)
    #待機(ms)
    # dobot.wait(1000)
    # dobot.move_to(200, 0, 0, 0)
    # dobot.move_to(250, 0, 0, 0)
    
    sleep(3)
    print(dobot.pose())


def test():
    init_points = [
        [10.00, 200, 200],
        [10.04, 206, 218],
        [10.09, 214, 244]
    ]
    orbit = Orbit(0.8, 20, 100, (1.0, 1.0), (1.0, 1.0))
    for point in init_points:
        orbit.add(point)
    orbit_predict = orbit.predict()
    
    hit = HIT(0, 0)
    xyt, direction = hit.hitCondition(orbit_predict)
    hit.hitHeadon(xyt, direction)
    hit.returnDobot(1)


if __name__ == "__main__":
    main()