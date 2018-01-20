#!/usr/bin/env python
import rospy
from move_aibo import AiboJointMover

if __name__ == "__main__":
    aibo_jointmover_object = AiboJointMover()
    aibo_jointmover_object.aibo_init_pose()

