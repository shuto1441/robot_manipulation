#!/usr/bin/env python
from __future__ import print_function
import tkinter as tk
from robot_manipulation import DobotClient as dc

if __name__ == "__main__":
    isJoint = False
    master = tk.Tk()
    button1 = tk.Button(master, text="foward", command=lambda: dc.set_jog_cmd(isJoint, 1))
    button1.pack()
    button2 = tk.Button(master, text="back", command=lambda: dc.set_jog_cmd(isJoint, 2))
    button2.pack()
    button3 = tk.Button(master, text="left", command=lambda: dc.set_jog_cmd(isJoint, 3))
    button3.pack()
    button4 = tk.Button(master, text="right", command=lambda: dc.set_jog_cmd(isJoint, 4))
    button4.pack()
    button5 = tk.Button(master, text="up", command=lambda: dc.set_jog_cmd(isJoint, 5))
    button5.pack()
    button6 = tk.Button(master, text="down", command=lambda: dc.set_jog_cmd(isJoint, 6))
    button6.pack()
    button7 = tk.Button(master, text="left roll", command=lambda: dc.set_jog_cmd(isJoint, 7))
    button7.pack()
    button8 = tk.Button(master, text="right roll", command=lambda: dc.set_jog_cmd(isJoint, 8))
    button8.pack()
    button9 = tk.Button(master, text="stop", command=lambda: dc.set_jog_cmd(isJoint, 0))
    button9.pack()
    button10 = tk.Button(master, text="homing", command=lambda: dc.set_home_cmd())
    button10.pack()
    button11 = tk.Button(master, text="get pose", command=lambda: print(dc.get_pose()))
    button11.pack()
    master.mainloop()
