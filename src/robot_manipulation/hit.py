import numpy as np
import time
from robot_manipulation.pydobot_ros import Dobot


class HIT:
    def __init__(self, method):
        self.moving = False
        self.dobot = Dobot()
        self.p_base = [0, 0]
        self.dobot.move_to(145, 0, -20, 0)
        self.stop([145, 0])
        self.p_init = self.dobot.pose()
        self.z = self.p_init.z
        self.r = self.p_init.r
        print(self.p_init)
        self.method = method

        # move-distance when hit
        if method == 0: # tanaka method
            self.l = 60
        elif method == 1: # noda method
            self.l = 80 # almost upper limit
        

    def calcDistance(self, position1, position2):
        p1 = np.array(position1)
        p2 = np.array(position2)
        d = np.linalg.norm(p1 - p2)
        return d

    def dobotArea(self, x, y):
        d_from_base = self.calcDistance([x, y], self.p_base)
        judge = False
        if  x >= 145 and y > -150 and y < 150:
            if self.method == 0:
                if d_from_base < 280:
                    judge = True
            else:
                if x <= 265:
                    judge = True
        return judge

    def stop(self, goal):
        self.moving = True
        print('moving to x: ' + str(goal[0]) + ' y: ' + str(goal[1]))
        k = 0
        while(k < 1001):
            output = self.dobot.pose()
            now = [output.x, output.y]
            if self.calcDistance(now, goal) < 2:
                self.moving = False
                break
            if k == 1000:
                print('Error! k==1000')
            k += 1
            time.sleep(10/1000)
        print('moved ' + str(k*10) + ' ms, then stop')

    def hitCondition(self, orbit_predict):
        xy_pre = None
        for xyt in orbit_predict:
            if xy_pre is None: # first time
                xy_pre = xyt[:2]
            else:
                xy = xyt[:2]
                d = self.calcDistance(xy_pre, xy)
                if d == 0: # if pack is stop
                    direction = [1, 0] # x axis direction
                else:
                    dir = (np.array(xy_pre) - np.array(xy)) / d
                    if dir[0] > 0 and dir[1]*xy[1] > 0:
                        direction = dir.tolist()
                    else:
                        direction = [1, 0] # x axis direction

                if self.method == 0:
                    x_dobot_start = xy[0] - direction[0] * 70
                    y_dobot_start = xy[1] - direction[1] * 70
                    if self.dobotArea(x_dobot_start, y_dobot_start):
                        x_dobot_goal = xy[0] - direction[0] * (70 - self.l)
                        y_dobot_goal = xy[1] - direction[1] * (70 - self.l)
                        if self.dobotArea(x_dobot_goal, y_dobot_goal):
                            wait_time = xyt[2] - int(time.time() * 1000)  # ms
                            if wait_time > 3000:
                                return None
                            else:
                                return xyt, direction
                elif self.method == 1:
                    x_dobot_start = xy[0] - direction[0] * 70
                    y_dobot_start = xy[1] - direction[1] * 70
                    if self.dobotArea(x_dobot_start, y_dobot_start):
                        wait_time = xyt[2] - int(time.time() * 1000)  # ms
                        if wait_time > 3000:
                            return None
                        else:
                            return xyt, direction
            xy_pre = xyt[:2]
        return None

    def hit(self, xyt, direction):
        if self.method == 0: # tanaka method
            self.hitHeadon(xyt, direction)
        elif self.method == 1: # noda method
            self.hitXdirection(xyt, direction)

    def hitHeadon(self, xyt, direction): # tanaka method
        if direction[0] < -0.5: # if pack is leaving
            return
        
        print("Headon")

        x = xyt[0] - direction[0] * 70
        y = xyt[1] - direction[1] * 70
        if self.dobotArea(x, y):
            self.dobot.speed(800, 800)  # velocity, acceleration
            self.dobot.move_to(x, y, self.z, self.r)
            self.stop([x, y])
            wait_time = xyt[2] - int(time.time() * 1000)  # ms
            print(wait_time)
            if wait_time > 100:
                self.dobot.wait(wait_time - 100)

            x += direction[0] * self.l
            y += direction[1] * self.l
            if self.dobotArea(x, y):
                self.dobot.speed(800, 800)  # velocity, acceleration
                self.dobot.move_to(x, y, self.z, self.r)
                self.stop([x, y])


    def hitXdirection(self, xyt, direction): # noda method
        if direction[0] < -0.5: # if pack is leaving
            return
        
        print("Xdirection")
        direction = [1, 0]

        x = 145
        y = xyt[1] - direction[1] * 70
        if self.dobotArea(x, y):
            self.dobot.speed(800, 800)  # velocity, acceleration
            self.dobot.move_to(x, y, self.z, self.r)
            self.stop([x, y])
            wait_time = xyt[2] - int(time.time() * 1000)  # ms
            print('wait time is ' + str(wait_time))
            if wait_time > 100:
                self.dobot.wait(wait_time - 100)

            x += direction[0] * self.l
            if self.dobotArea(x, y):
                self.dobot.speed(800, 800)  # velocity, acceleration
                self.dobot.move_to(x, y, self.z, self.r)
                self.stop([x, y])
                    
                x = 145
                self.dobot.speed(800, 800)  # velocity, acceleration
                self.dobot.move_to(x, y, self.z, self.r)
                self.stop([x, y])
    
    def hitDirect(self, xyt): # direct method
        x = xyt[0]
        y = xyt[1]
        if self.dobotArea(x, y):
            print("Direct")

            self.dobot.speed(800, 800)  # velocity, acceleration
            self.dobot.move_to(x, y, self.z, self.r)
            self.stop([x, y])

            x = 145
            y = 0
            self.dobot.speed(800, 800)  # velocity, acceleration
            self.dobot.move_to(x, y, self.z, self.r)
            self.stop([x, y])
        else:
            print("Direct but out of dobotArea")
            print([x, y])

    def catchPack(self, xyt, direction): # catch pack
        x = xyt[0] - direction[0] * 70
        y = xyt[1] - direction[1] * 70
        if self.dobotArea(x, y):
            self.dobot.speed(800, 800)  # velocity, acceleration
            self.dobot.move_to(x, y, self.z, self.r)
            self.stop([x, y])
            wait_time = xyt[2] - int(time.time() * 1000)  # ms
            print(wait_time)
            if wait_time > 10:
                self.dobot.wait(wait_time - 10)

            x -= direction[0] * 50
            y -= direction[1] * 50
            if self.dobotArea(x, y):
                self.dobot.speed(800, 800)  # velocity, acceleration
                self.dobot.move_to(x, y, self.z, self.r)
                self.stop([x, y])

    def returnDobot(self, ratio):
        if ratio < 0 or ratio > 1:
            print('Error value of ratio')
            return None
        p_now = self.dobot.pose()
        x_next = p_now.x * (1-ratio) + self.p_init.x * ratio
        y_next = p_now.y * (1-ratio) + self.p_init.y * ratio
        if self.dobotArea(x_next, y_next):
            self.dobot.speed(800, 800)  # velocity, acceleration
            # ratio=1 means p_init
            self.dobot.move_to(x_next, y_next, self.z, self.r)
            self.stop([x_next, y_next])
        return [x_next, y_next]
