#! /usr/bin/env python
import rospy
from dobot.srv import *
class Dobot:
    def __init__(self):
        self.x = 0
        self.y = 20
        self.z = 30
        self.r = 0
        self.velocity = 100
        self.acceleration = 100

    def set_home_cmd(self):
        """
        Returns:
            int32 result
            uint64 queuedCmdIndex
        """
        return run(SetHOMECmd)

    def move_to(self, x, y, z, r):
        run(SetPTPCmd, 2, x, y, z, r)

    def suck(self, enable):
        run(SetEndEffectorSuctionCup, enable)

    def grip(self, enable):
        run(SetEndEffectorGripper, enable)

    def speed(self, velocity, acceleration):
        run(SetPTPCommonParams, velocity, acceleration, 1)
        run(SetPTPCoordinateParams, velocity, acceleration, velocity, acceleration, 1)

    def wait(self, ms):
        run(SetWAITCmd, ms)

    def pose(self):
        """
        Returns:
            int32 result
            float32 x
            float32 y
            float32 z
            float32 r
            float32[] jointAngle
        """
        return run(GetPose)

def run(command, *args, **kwargs):
    rospy.wait_for_service('{}'.format(command.__name__))
    cmd = rospy.ServiceProxy('{}'.format(command.__name__), command)
    response = cmd(*args, **kwargs)
    return response