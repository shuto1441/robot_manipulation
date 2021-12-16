from pydobot import Dobot

class Dobot(Dobot):

    def __init__(self):
        self.x = 0
        self.y = 20
        self.z = 30
        self.r = 0
        self.velocity = 100
        self.acceleration = 100

    @classmethod
    def pose(self):
        print(f'pose:\tx={self.x}\ty={self.y}\tz={self.z}\tr={self.r}')
        return self.x, self.y, self.z, self.r

    @classmethod
    def move_to(self, x, y, z, r, wait=False):
        self.x, self.y, self.z, self.r = x, y, z, r
        print('move to:\tx={self.x}\ty={self.y}\tz={self.z}\tr={self.r}')

    @classmethod
    def speed(self, velocity=100, acceleration=100):
        self.velocity = velocity
        self.acceleration = acceleration
        print('speed:\tvelocity={self.velocity}\tacceleration={self.acceleration}')
