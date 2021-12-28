import numpy as np
import time
from robot_manipulation.pydobot_ros import Dobot


class HIT:
    def __init__(self):
        self.dobot = Dobot()
        self.p_base = np.array([0, 0])
        self.dobot.move_to(155, 0, -20, 0)
        self.stop(np.array([155, 0]))
        self.p_init = self.dobot.pose()
        print(self.p_init)
        self.z = self.p_init.z
        self.r = self.p_init.r

    def calcDistance(self, position1, position2):
        d = np.linalg.norm(position1 - position2)
        return d

    def motionRange(self, x, y):
        d_from_base = self.calcDistance([x, y], self.p_base)
        judge = False
        if x > 0 and x < 300 and y > -165 and y < 165 and d_from_base > 155 and d_from_base < 300:
            judge = True
        return judge

    def hitRange(self, x, y):
        d_from_base = self.calcDistance(np.array([x, y]), self.p_base)
        judge = False
        if x > 0 and x < 300 and y > -165 and y < 165 and d_from_base > 155 and d_from_base < 200:
            judge = True
        return judge

    def stop(self, goal):
        k = 0
        while(k < 1000):
            output = self.dobot.pose()
            now = np.array([output.x, output.y])
            if self.calcDistance(now, goal) < 1:
                break
            k += 1
            time.sleep(10/1000)
        print(k)

    def hitCondition(self, orbit_predict):
        xy_pre = None
        for xyt in np.array(orbit_predict):
            if self.hitRange(xyt[0], xyt[1]):
                xy = xyt[:2]
                if xy_pre is None:
                    continue
                direction = (xy_pre - xy) / self.calcDistance(xy_pre, xy)
                wait_time = xyt[2] - int(time.time() * 1000)  # ms
                if wait_time > 3000:
                    return None
                else:
                    return xyt.tolist(), direction.tolist()
            xy_pre = xyt[:2]
        return None

    def hitHeadon(self, xyt, direction):
        x = xyt[0] - direction[0] * 70
        y = xyt[1] - direction[0] * 70
        if self.motionRange(x, y):
            self.dobot.speed(800, 800)  # velocity, acceleration
            self.dobot.move_to(x, y, self.z, self.r)
            self.stop(np.array([x, y]))
            wait_time = xyt[2] - int(time.time() * 1000)  # ms
            print(wait_time)
            if wait_time > 100:
                self.dobot.wait(wait_time - 100)

            x += direction[0] * 50
            y += direction[1] * 50
            if self.motionRange(x, y):
                self.dobot.speed(800, 800)  # velocity, acceleration
                self.dobot.move_to(x, y, self.z, self.r)
                self.stop(np.array([x, y]))

    def returnDobot(self, ratio):
        if ratio < 0 or ratio > 1:
            print('Error value of ratio')
            return None
        p_now = self.dobot.pose()
        x_next = p_now.x * (1-ratio) + self.p_init.x * ratio
        y_next = p_now.y * (1-ratio) + self.p_init.y * ratio
        if self.motionRange(x_next, y_next):
            self.dobot.speed(800, 800)  # velocity, acceleration
            # ratio=1 means p_init
            self.dobot.move_to(x_next, y_next, self.z, self.r)
            self.stop(np.array([x_next, y_next]))
        return [x_next, y_next]
