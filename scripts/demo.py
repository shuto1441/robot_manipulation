#! /usr/bin/env python
from pydobot_ros import Dobot

Dobot = Dobot()

Dobot.set_home_cmd()
print(Dobot.pose())
Dobot.wait(1000)
Dobot.speed(100, 100)
Dobot.move_to(150, -100, 70, 0)
