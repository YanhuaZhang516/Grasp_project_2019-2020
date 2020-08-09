from robosuite.models.base import MujocoXML
from mujoco_py import MjSim, MjRenderContextOffscreen
from robosuite.utils import SimulationError, XMLError, MujocoPyRenderer
from robosuite.models.arenas.table_arena import TableArena
import os
import numpy as np
import mujoco_py
import time
from mujoco_py import load_model_from_xml, MjSim, functions, mjrenderpool, MjSimState
from scipy.interpolate import CubicSpline


import pybullet as p


if __name__ == "__main__":
    fname = os.getcwd() + "/robosuite/models/assets/robots/darias/darias_nohands.xml"

    mujoco_darias = MujocoXML(fname)

    model = mujoco_darias
    mjpy_model = model.get_model(mode="mujoco_py")
    sim = MjSim(mjpy_model)





    model_timestep = sim.model.opt.timestep

    print("model_timestep:", model_timestep)

    ramp_ratio=0.2
    control_freq=200


    interpolation_steps = np.floor(ramp_ratio * control_freq / model_timestep)

    print(interpolation_steps)
    print('number of contacts', sim.data.ncon)



    id_name="right_endeffector_link"
    #id_name="R_SAA"
    current_position=[]
    current_position = sim.data.body_xpos[sim.model.body_name2id(id_name)]
    print("right_endeffector_link position:", current_position)
    current_orientation_mat = sim.data.body_xmat[sim.model.body_name2id(id_name)].reshape([3, 3])
    print("right_endeffector_link orientation", current_position)
    # current_lin_velocity = sim.data.body_xvelp[sim.model.body_name2id(id_name)]
    # current_ang_velocity = sim.data.body_xvelr[sim.model.body_name2id(id_name)]
    #


    joint_position_current=[sim.data.qpos[x] for x in range(14)]
    joint_velocity_current= [sim.data.qvel[x] for x in range(14)]
    print("joint_position_current:", joint_position_current)
    print("joint_velocity_current:", joint_velocity_current)

    desired_pos = np.eye(14)*5



    #Jx = sim.data.get_body_jacp(id_name).reshape((3, -1))[:, joint_index]
    #print("Jx of joint:", Jx)
    # Jr = sim.data.get_body_jacr(id_name).reshape((3, -1))[:, joint_index]
    # J_full = np.vstack([Jx, Jr])

    mass_matrix = np.ndarray(shape=(len(sim.data.qvel) ** 2,), dtype=np.float64, order='C')
    mujoco_py.cymj._mj_fullM(sim.model, mass_matrix, sim.data.qM)

    mass_matrix2 = np.reshape(mass_matrix, (len(sim.data.qvel), len(sim.data.qvel)))

    # print("mass_matrix:", mass_matrix)
    # print("mass_matrix2:", mass_matrix2)
    #
    # print(len(mass_matrix))
    #
    # print(mass_matrix.shape)
    # print(len(sim.data.qvel))
    # print(len(sim.data.body_xpos))

    # mass_matrix test from other roboter




    def calculate_torques(current_joint_position, current_joint_velocity, desired_joint_position, kp_joint, kv_joint,
                          ):

        position_joint_error = current_joint_position - desired_joint_position

        torques = np.multiply(kp_joint, position_joint_error) - np.multiply(kv_joint, current_joint_velocity)

        return torques




    kp = np.ones(14)* 100
    kv = np.ones(14)*2
    torques= calculate_torques(joint_position_current, joint_velocity_current, desired_pos, kp, kv )


  #  _ref_joint_vel_indexes = [sim.model.get_joint_qvel_addr('L_WAA')]

    _ref_joint_vel_indexes = np.arange(14)
    # print(_ref_joint_vel_indexes)
    # print(ref_joint_vel_indexes)


    #_-------- interpolation of goal
    from scipy.interpolate import CubicSpline

    def interpolate_joint(starting_joint, last_goal_joint, interpolation_steps, current_vel, interpolation):
        # we interpolate to reach the commanded desired position in the ramp_ratio % of time we have this goal
        # joint position

        if interpolation == "cubic":
            time = [0, interpolation_steps]
            position = np.vstack((starting_joint, last_goal_joint))
            cubic_joint = CubicSpline(time, position, bc_type=((1, current_vel), (1, np.zeros(14))), axis=0)
            interpolation_joint = np.array([cubic_joint(i) for i in range(interpolation_steps)])



        if interpolation == "linear":
            delta_x_per_step = (last_goal_joint - starting_joint)/ interpolation_steps
            interpolation_joint = np.array([starting_joint + i* delta_x_per_step for i in range(1, interpolation_steps+1)])


        return interpolation_joint


    starting_joint = np.zeros(14)

    goal_joint = np.zeros(14)
    goal_joint[8]=10
    goal_joint[6]=5
    goal_joint[13]=-10

    # print("staring joint:", starting_joint)
    # print("goal joint:", goal_joint)

    position = np.vstack((starting_joint, goal_joint))
    # print("position:", position)
    # print(np.shape(position))



    interpolation_joint_cubic = interpolate_joint(starting_joint, goal_joint, 100, np.zeros(14), "cubic")
    interpolation_joint_linear = interpolate_joint(starting_joint, goal_joint, 100, np.zeros(14), "linear")
    # print(np.shape(interpolation_joint_cubic))
    # print(np.shape(interpolation_joint_linear))

    def Gravity():
        for i in range(14):
            sim.data.qfrc_applied[i] = sim.data.qfrc_bias[i]

    viewer = MujocoPyRenderer(sim)

    ctrl_range = sim.model.actuator_ctrlrange


    bias = 0.5 *(ctrl_range[:,1] + ctrl_range[:,0])
    weight = 0.5 *(ctrl_range[:,1] - ctrl_range[:,0])
    action = np.ones(14) *5
    applied_action = bias + weight*action
    print(applied_action)

    print("before the action:",sim.data.qpos[0])







    for i in range(5000):
        Gravity()

        #x = np.ones(14) *100
       # sim.data.ctrl[:] = x
       # sim.data.ctrl[6] =0.1
        sim.data.ctrl[_ref_joint_vel_indexes] = applied_action

        sim.step()
        curr_pos = sim.data.qpos[0]
        #print(curr_pos)

        viewer.render()

    print("after action:", sim.data.qpos[0])






















