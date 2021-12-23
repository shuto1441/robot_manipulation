#!/usr/bin/env python
from __future__ import print_function

from robot_manipulation.pydobot_ros import Dobot

import tkinter as tk

if __name__ == "__main__":
    #dobot名の指定
    dobot = Dobot()
    #ホーミング
    #dobot.home()
    #速度設定　spped(速度，加速度)
    dobot.speed(800, 800)
    #移動(x, y, z, r)
    dobot.move_to(200, 200, 0, 0)
    # dobot.move_to(250, 0, 0, 0)
    #待機(ms)
    # dobot.wait(1000)
    # dobot.move_to(200, 0, 0, 0)
    # dobot.move_to(250, 0, 0, 0)
    #吸着(ON)
    # dobot.suck(1)
    # dobot.wait(3000)
    #吸着(OFF)
    # dobot.suck(0)
    #print(dobot.pose())
