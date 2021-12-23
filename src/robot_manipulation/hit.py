import numpy as np
import time
from robot_manipulation.pydobot_ros import Dobot


class HIT:
    def __init__(self, x_base, y_base):
        self.p_base = [x_base, y_base]
        # Dobot.move_to(#,#,#,#)
        self.p_init = Dobot.pose()
        self.z = self.p_init[2]
        self.r = self.p_init[3]

    def calcDistance(self, position1, position2):
        d = np.linalg.norm(np.array(position1) - np.array(position2))
        return d

    def motionRange(self, x, y):
        d_from_base = self.calcDistance([x, y], self.p_base)
        judge = False
        if d_from_base > 100 and d_from_base < 300:
            judge = True
        return judge

    def hitRange(self, x, y):
        d_from_base = self.calcDistance([x, y], self.p_base)
        judge = False
        if d_from_base > 150 and d_from_base < 200:
            judge = True
        return judge

    def hitCondition(self, route):
        xy_pre = []
        for xyt in route:
            if self.hitRange(xyt[0], xyt[1]):
                xy = xyt[:2]
                direction = (xy_pre - xy) / self.calcDistance(xy_pre, xy)
                return xyt, direction
            xy_pre = xyt[:2]

    def hitHeadon(self, xyt, direction):
        x = xyt[0]
        y = xyt[1]
        if self.motionRange(x, y):
            Dobot.speed(400, 400)  # velocity, acceleration
            Dobot.move_to(x, y, self.z, self.r)
            wait_time = xyt[2] - time.time() * 1000  # ms
            Dobot.wait(wait_time)

            x += direction[0] * 30
            y += direction[1] * 30
            if self.motionRange(x, y):
                Dobot.speed(100, 100)  # velocity, acceleration
                Dobot.move_to(x, y, self.z, self.r)

    def returnDobot(self, ratio):
        if ratio < 0 or ratio > 1:
            print('Error value of ratio')
            return None
        p_now = Dobot.pose()
        x_next = p_now[0] * (1-ratio) + self.p_init[0] * ratio
        y_next = p_now[1] * (1-ratio) + self.p_init[1] * ratio
        if self.motionRange(x_next, y_next):
            Dobot.speed(400, 400)  # velocity, acceleration
            # ratio=1 means p_init
            Dobot.move_to(x_next, y_next, self.z, self.r)
        return [x_next, y_next]
