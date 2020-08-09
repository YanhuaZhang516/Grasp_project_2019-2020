# here we use pybullt to calculate the inverse kinematics
import pybullet as p
import time
import math
from DateTime import DateTime
import random
import numpy as np
import scipy
from scipy.interpolate import interp1d

import os
from robosuite.models.base import MujocoXML
from mujoco_py import MjSim, MjRenderContextOffscreen
from robosuite.utils import SimulationError, XMLError, MujocoPyRenderer, transform_utils
from robosuite.models.arenas.table_arena import TableArena
import mujoco_py
import robosuite.utils.transform_utils as T



from mujoco_py import load_model_from_xml, MjSim, functions, mjrenderpool, MjSimState
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

if __name__ == "__main__":


    # activate pybullet here

    fname = os.getcwd() + "/robosuite/models/assets/bullet_data/darias_description/robots/darias.urdf"
    p.connect(p.DIRECT)  # physicsClient = p.connect(p.GUI)

    p.setGravity(0, 0, -9.81)
    darias = p.loadURDF(fname, [0, 0, 0], [0, 0, 0, 1])  # set the basePosition and baseOrientation
    # activate mujoco here
    fname2 = os.getcwd() + "/robosuite/models/assets/robots/darias/darias.xml"

    mujoco_darias = MujocoXML(fname2)

    model = mujoco_darias

    # merge other enviroments
    # table

    table_full_size = (0.8, 0.8, 0.8)
    table_friction = (1., 5e-3, 1e-4)
    mujoco_arena = TableArena(
        table_full_size=table_full_size,
        table_friction=table_friction
    )

    # filename_cloth = os.getcwd() + "/robosuite/models/assets/objects/cloth.xml"

    filename_supporter = os.getcwd() + "/robosuite/models/assets/objects/supporter.xml"
    # mujoco_cloth = MujocoXML(filename_cloth)

    mujoco_supporter = MujocoXML(filename_supporter)
    filename_table = os.getcwd() + "/robosuite/models/assets/arenas/table_arena.xml"
    mujoco_table = MujocoXML(filename_table)
    # model.merge(mujoco_arena)
    # model.merge(mujoco_cloth)
    model.merge(mujoco_table)
    model.merge(mujoco_supporter)

    mjpy_model = model.get_model(mode="mujoco_py")
    sim = MjSim(mjpy_model)


    # Gravity compensation
    print(len(sim.data.qpos))
    def Gravity():
        for i in range(len(sim.data.qpos)):
            sim.data.qfrc_applied[i] = sim.data.qfrc_bias[i]



    ############## pybullet part##################################
    # joint_num = p.getNumJoints(darias)
    #
    # print("the number of joint:", joint_num)
    # # print("the name of joints:", joint_name[1])
    #
    # # get the all index and joints of darias:
    # for i in range(joint_num):
    #     joint_name = p.getJointInfo(darias, i)
    #     print(joint_name[0], joint_name[1])
    #




    ##################### mujoco part ############################



    viewer = MujocoPyRenderer(sim)

    Gravity()
    initial_pos=[sim.data.qpos[x] for x in range(len(sim.data.qpos))]




    print("initial pos:", initial_pos)
    print("the number of joints:", len(sim.data.qpos))
    print("the initial pos of middle finger before:", sim.data.qpos[13:16])
    # while(True):
    #     Gravity()
    #     viewer.render()
    #     sim.step()
    #     print("the initial pos of middle finger after:", sim.data.qpos[13:16])
    #
    # print("the initial pos of middle finger after:", sim.data.qpos[13:16])


    pos1=[sim.data.qpos[x] for x in range(7)]
    pos2=[sim.data.qpos[x] for x in range(22, 29)]
    print("pos1:", pos1)
    print("pos2:", pos2)
    print("pos", np.array(np.hstack((pos1, pos2))))


    def controll1():

        desired = [-3.14 / 6, 3.14 / 6, 3.14 / 6]
        desired2 = [-0.174, 3.14 / 6, 3.14 / 7]
        current_M = sim.data.qpos[13:16]
        error_M = desired - current_M

        current_thumb = sim.data.qpos[19:22]
        error_thumb = desired2 - current_thumb

        # small finger
        sim.data.qpos[7:10] = desired2
        # ringer finger
        sim.data.qpos[10:13] = desired
        # middle finger
        sim.data.qpos[13:16] = desired
        # index finger
        sim.data.qpos[16:19] = desired
        # thumb finger
        sim.data.qpos[19:22] = desired2

        print("error_M:", error_M)
        print("current pos of M:", current_M)
        print("error_thumb", error_thumb)
        print("current")
        sim.forward()
        sim.step()


    def controll2_OPEN(i,flag):

        # # thumb, index (z-axis)
        # desired1 = [-0.2, -0.2, 0.5]
        # desired1L =[0.2, -0.2, 0.5]
        # desired2 = [-0.3, -0.2, 0.5]
        # desired2L =[0.3, -0.2, 0.5]
        # # middle
        # desired3=[0.0, -0.2, 0.5]
        # desired3L=desired3
        #
        # # Ring
        # desired4=[0.2, -0.2, 0.5]
        # desired4L=[-0.2, -0.2, 0.5]
        # # small
        # desired5=[0.3, -0.2, 0.5]
        # desired5L=[-0.3, -0.2, 0.5]
        #
        # thumb, index (z-axis)
        desired1 = [0.0, 0.5, 0.5]
        desired1L =[0.0, 0.5, 0.5]
        desired2 = [0.0, 0.5, 0.5]
        desired2L =[0.0, 0.5, 0.5]
        # middle
        desired3=[0.0, 0.5, 0.5]
        desired3L=desired3

        # Ring
        desired4=[0.0, 0.5, 0.5]
        desired4L=[0.0, 0.5, 0.5]
        # small
        desired5=[0.0, 0.5, 0.5]
        desired5L=[0.0, 0.5, 0.5]




        # middle finger
        current_M_1 = sim.data.qpos[13:16]
        error_M_1 = desired3 - current_M_1
        # index finger
        current_index_1 = sim.data.qpos[16:19]
        error_index_1 = desired2 - current_index_1
        # ringer finger
        current_R_1 = sim.data.qpos[10:13]
        error_R_1 = desired4 - current_R_1
        # thumb finger
        current_thumb_1 = sim.data.qpos[19:22]
        error_thumb_1 = desired1 - current_thumb_1
        # small finger
        current_small_1 = sim.data.qpos[7:10]
        error_small_1 = desired5-current_small_1

        # small finger
        sim.data.ctrl[7:10] = desired5
        sim.data.ctrl[29:32] = desired5L
        print("desired of right,left small finger", desired5)
        print("right small finger pos:", sim.data.qpos[7:10])
        print("left small finger pos:", sim.data.qpos[29:32])

        # ringer finger
        sim.data.ctrl[10:13] = desired4
        sim.data.ctrl[32:35] = desired4L
        print("desired of right ringer finger", desired4)
        print("right ringer finger pos:", sim.data.qpos[10:13])
        print("left ringer finger pos:", sim.data.qpos[32:35])
        # # middle finger
        sim.data.ctrl[13:16] = desired3
        sim.data.ctrl[35:38] = desired3L
        print("desired of right middle finger", desired3)
        print("right middle finger pos:", sim.data.qpos[13:16])
        print("left middle finger pos:", sim.data.qpos[35:38])

        # index finger
        sim.data.ctrl[16:19] = desired2
        sim.data.ctrl[38:41] = desired2L
        print("desired of right index finger", desired2)
        print("right index finger pos:", sim.data.qpos[16:19])
        print("left index finger pos:", sim.data.qpos[38:41])

        # thumb finger
        sim.data.ctrl[19:22] = desired1
        sim.data.ctrl[41:44] = desired1L

        print("desired of right thumb finger", desired1)
        print("right thumb finger pos:", sim.data.qpos[19:22])
        print("left thumb finger pos:", sim.data.qpos[41:44])

        # middle finger
        current_M_2 = sim.data.qpos[13:16]
        error_M_2 = desired3 - current_M_2
        # thumb finger
        current_thumb_2 = sim.data.qpos[19:22]
        error_thumb_2 = desired1 - current_thumb_2
        # small finger
        current_small_2 = sim.data.qpos[7:10]
        error_small_2 = desired5 - current_small_2
        # index finger
        current_index_2 = sim.data.qpos[16:19]
        error_index_2 = desired2 - current_index_2
        # ringer finger
        current_R_2 = sim.data.qpos[10:13]
        error_R_2 = desired4 - current_R_2



        # print("error of thumb", error_thumb_2)
        # print("current pos of ")
        # print("error of small finger", error_small_2)
        # print("error of index finger", desired1-current_index_2)
        # # print("error of middle finger", error_M_2)
        # print("error of ringer finger", desired-current_R_2)

        sim.step()

        if(error_thumb_2[0]-error_thumb_1[0] <= 1e-2) and (error_M_2[0]-error_M_1[0] <= 1e-2) and (i>=3000):
            flag=False
        else:
            flag=True

        return flag

        #
        # print("error_M:", error_M)
        # print("current pos of M:", current_M)
        # print("error_thumb", error_thumb)
        # print("current")



    def simulate():
        i=0
        flag=True

        while(flag==True):

            Gravity()
            flag=controll2_OPEN(i, flag)
            i+=1

            viewer.render()

        print(i)
        while(True):
           # sim.step()
            viewer.render()

    def simulate_without():
        while(True):
            Gravity()
            sim.step()
            viewer.render()





    print("the initial pos of thumb:", sim.data.qpos[19:22])
    print("the initial pos of middle:", sim.data.qpos[13:16])
    simulate()
    #simulate_without()

