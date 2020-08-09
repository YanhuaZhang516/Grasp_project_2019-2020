import numpy as np
import pybullet as p

if __name__ == "__main__":

    fname = "/home/ren/mypy/RobotTaskSim/robosuite/models/assets/bullet_data/darias_description/robots/darias.urdf"

    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setGravity(0, 0, -9.81)

    darias = p.loadURDF(fname)

    joint_num = p.getNumJoints(darias)


    print("joint_num ",joint_num)

    p.disconnect()

