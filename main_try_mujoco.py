from robosuite.models.base import MujocoXML
from mujoco_py import MjSim, MjRenderContextOffscreen
from robosuite.utils import SimulationError, XMLError, MujocoPyRenderer
from robosuite.models.arenas.table_arena import TableArena
import os
import numpy as np
import mujoco_py


if __name__ == "__main__":

    fname = os.getcwd() + "/robosuite/models/assets/robots/darias/darias.xml"

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

    filename_cloth = os.getcwd() + "/robosuite/models/assets/objects/cloth.xml"

    filename_supporter = os.getcwd() + "/robosuite/models/assets/objects/supporter.xml"
    mujoco_cloth = MujocoXML(filename_cloth)

    mujoco_supporter = MujocoXML(filename_supporter)
    filename_table = os.getcwd() + "/robosuite/models/assets/arenas/table_arena.xml"
    mujoco_table = MujocoXML(filename_table)

    # model.merge(mujoco_cloth)
    model.merge(mujoco_table)
    # model.merge(mujoco_supporter)

    mjpy_model = model.get_model(mode="mujoco_py")
    sim = MjSim(mjpy_model)

    print('number of contacts', sim.data.ncon)

    for i in range(sim.data.ncon):
        contact = sim.data.contact[i]
        print('contact', i)
        print('dist', contact.dist)
        print('geom1', contact.geom1, sim.model.geom_id2name(contact.geom1))
        print('geom2', contact.geom2, sim.model.geom_id2name(contact.geom2))
        # There's more stuff in the data structure
        # See the mujoco documentation for more info!
        geom2_body = sim.model.geom_bodyid[sim.data.contact[i].geom2]
        print(' Contact force on geom2 body', sim.data.cfrc_ext[geom2_body])
        print('norm', np.sqrt(np.sum(np.square(sim.data.cfrc_ext[geom2_body]))))
        # Use internal functions to read out mj_contactForce
        c_array = np.zeros(6, dtype=np.float64)
        print('c_array', c_array)
        mujoco_py.functions.mj_contactForce(sim.model, sim.data, i, c_array)
        print('c_array', c_array)

    viewer = MujocoPyRenderer(sim)

    for i in range(50000):
        sim.step()

        viewer.render()

    # darias = p.loadURDF(fname)
    # joint_num = p.getNumJoints(darias)
    #
    # p.disconnect()
    #
    # print("joint_num", joint_num)
