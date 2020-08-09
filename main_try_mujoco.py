import numpy as np
import robosuite as suite
from robosuite import DariasClothGrasp
from robosuite.environments import MujocoEnv

from robosuite.models.base import MujocoXML

from mujoco_py import MjSim, MjRenderContextOffscreen
from robosuite.utils import SimulationError, XMLError, MujocoPyRenderer


from robosuite.models.tasks import ClothGrasp, UniformRandomSampler

from robosuite.models.arenas.table_arena import TableArena
from robosuite.models.objects import BoxObject
from collections import OrderedDict


import os
import xml.dom.minidom
import xml.etree.ElementTree as ET
import io
import numpy as np


from robosuite.utils import XMLError

from robosuite.models.robots import Darias

from robosuite.environments.base import MujocoEnv

import mujoco_py
import pybullet as p
import time

from robosuite.utils.mjcf_utils import string_to_array





if __name__ == "__main__":
    fname = "/home/mingyezhu/Desktop/robottasksim/robosuite/models/assets/robots/darias/darias.xml"



# if __name__ == "__main__":
#
#     fname = os.getcwd() + "/robosuite/models/assets/robots/darias/darias.xml"


    mujoco_darias = MujocoXML(fname)

    model = mujoco_darias

    table_full_size = (0.8, 0.8, 0.8)
    table_friction = (1., 5e-3, 1e-4)
    mujoco_arena = TableArena(
        table_full_size=table_full_size,
        table_friction=table_friction
    )

    # The sawyer robot has a pedestal, we want to align it with the table
    # mujoco_arena.set_origin([0.16 + table_full_size[0] / 2, 0, 0])


    filename_cloth = "/home/mingyezhu/Desktop/robottasksim/robosuite/models/assets/objects/cloth.xml"

    filename_supporter = "/home/mingyezhu/Desktop/robottasksim/robosuite/models/assets/objects/supporter.xml"
    mujoco_cloth = MujocoXML(filename_cloth)

    mujoco_supporter = MujocoXML(filename_supporter)
    filename_table = "/home/mingyezhu/Desktop/robottasksim/robosuite/models/assets/arenas/table_arena.xml"
    mujoco_table = MujocoXML(filename_table)
    # model.merge(mujoco_arena)
    model.merge(mujoco_cloth)
    model.merge(mujoco_table)
    model.merge(mujoco_supporter)

    env = suite.DariasClothGrasp
    #
    # env = suite.make(
    #     DariasClothGrasp,
    # )


    mjpy_model = model.get_model(mode="mujoco_py")
    sim = MjSim(mjpy_model)



    viewer = MujocoPyRenderer(sim)



    for i in range(50000):

        sim.step()
        viewer.render()
        if i % 100 == 0:
            a = sim.data.sensordata

            print(a)

            Rotation_matrix = sim.data.body_xmat[6].reshape([3, 3])
            Translation_matrix = sim.data.body_xpos[6].reshape([3, 1])

            print("Rotation_matrix ", Rotation_matrix)
            print('Translation_matrix ', Translation_matrix)

            T1 = np.concatenate((Rotation_matrix, Translation_matrix), axis=1)

            T2 = np.array([[0, 0, 0, 1]])
            T = np.concatenate((T1, T2), axis=0)
            print('T ', T)





    a = sim.data.sensordata

    # viewer.render()
    #
    # body_name_right = "right_endeffector_link"
    # current_position = sim.data.body_xpos[sim.model.body_name2id(body_name_right)]
    # print("the right eef pos:", current_position)
   # print(a)

