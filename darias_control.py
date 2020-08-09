# here we use pybullt to calculate the inverse kinematics
import pybullet as p
import time
import math
from DateTime import DateTime
import random
import numpy as np
import scipy
from scipy.interpolate import interp1d
import copy

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

    fname = os.getcwd() + "/robosuite/models/assets/bullet_data/darias_description/robots/darias_no_hands.urdf"
    p.connect(p.DIRECT)  # physicsClient = p.connect(p.GUI)

    p.setGravity(0, 0, -9.81)
    darias = p.loadURDF(fname, [0, 0, 0], [0, 0, 0, 1])  # set the basePosition and baseOrientation

    # initial parameters:
    ramp_ratio = 0.20  # Percentage of the time between policy time-steps used for interpolation
    control_freq = 20  # control steps per second

    # lower limits for null space
    ll = [-2.967059, -2.094395, -2.967059, -2.094395, -2.967059, -2.094395, -2.967059,
          -2.967059, -2.094395, -2.967059, -2.094395, -2.967059, -2.094395, -2.094395]
    # upper limits for null space
    ul = [2.967059, 2.094395, 2.967059, 2.094395, 2.967059, 2.094395, 2.967059,
          2.967059, 2.094395, 2.967059, 2.094395, 2.967059, 2.094395, 2.094395]
    # joint ranges for null space
    jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
    # initial joint position of arms
    po = np.zeros(7)
    # joint damping coefficents
    jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

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
    # mujoco_cloth = MujocoXML(filename_cloth)

    filename_supporter = os.getcwd() + "/robosuite/models/assets/objects/supporter.xml"
    mujoco_supporter = MujocoXML(filename_supporter)

    filename_table = os.getcwd() + "/robosuite/models/assets/arenas/table_arena.xml"
    mujoco_table = MujocoXML(filename_table)

   # model.merge(mujoco_cloth)
    model.merge(mujoco_table)
    model.merge(mujoco_supporter)

    mjpy_model = model.get_model(mode="mujoco_py")

    sim = MjSim(mjpy_model)


    # Gravity compensation
    def Gravity():

        for i in range(44):
            sim.data.qfrc_applied[i] = sim.data.qfrc_bias[i]


    def square_root(vector1, vector2):
        m = len(vector1)
        diff = np.array(vector1) - np.array(vector2)
        sum = 0
        for i in range(m):
            sum += diff[i] * diff[i]

        return np.sqrt(sum)


    def inverse_kinematics1(
            target_position_right,
            target_orientation_right,
            target_position_left,
            target_orientation_left,
            joint_poses_previous,
            Index

    ):
        """
        helper function to do inverse kinematics for a given target position and orientation in the pybullet world frame

        :returns
        An array of size num_joints corresponding to the joint angle solution


        """

        # 0: disable real-time simulation, the external forces=0 after each step
        # 1: to enable

        p.setRealTimeSimulation(1)

        # the index of {right,left} end-effectors
        effector_right = 8
        effector_left = 16
        maxIter = 500
        threshold = 1e-5
        closeEnough = False
        iter = 0
        joint_poses = []
        joint_num = 17

        if (Index == 2):
            j = 0
            for i in range(joint_num):
                jointInfo = p.getJointInfo(darias, i)
                qIndex = jointInfo[3]
                if qIndex > -1:
                    p.resetJointState(darias, i, joint_poses_previous[j])
                    j = j + 1

        while (not closeEnough and iter < maxIter):
            jointPoses1 = list(
                p.calculateInverseKinematics(darias, effector_right, target_position_right, target_orientation_right,
                                             ll[0:7], ul[0:7],
                                             jr))

            jointPoses2 = list(
                p.calculateInverseKinematics(darias, effector_left, target_position_left, target_orientation_left,
                                             ll[7:14], ul[7:14],
                                             jr))
            for i in range(7, 14):
                jointPoses1[i] = jointPoses2[i]

            jointPoses = jointPoses1
            j = 0

            for i in range(joint_num):
                jointInfo = p.getJointInfo(darias, i)
                qIndex = jointInfo[3]
                if qIndex > -1:
                    p.resetJointState(darias, i, jointPoses[j])
                    j = j + 1

            ls_r = list(p.getLinkState(darias, effector_right)[4])
            ls_l = list(p.getLinkState(darias, effector_left)[4])
            dist2_r = square_root(ls_r, target_position_right)
            dist2_l = square_root(ls_l, target_position_left)

            closeEnough = (dist2_r < threshold and dist2_l < threshold)
            iter = iter + 1
        # print("iter=", iter)
        # print("dist2_l:", dist2_l)
        # print("dist2_r:", dist2_r)
        # print("pos of right end:", ls_r)
        # print("pos of left end:", ls_l)

        return np.array(jointPoses)


    def interpolate_joint(starting_joint, last_goal_joint, interpolation_steps, current_vel, interpolation):
        # we interpolate to reach the commanded desired position in the ramp_ratio % of time we have this goal
        # joint position is a (interpolation_steps, 14) matrix

        if interpolation == "cubic":
            time = [0, interpolation_steps]
            position = np.vstack((starting_joint, last_goal_joint))
            cubic_joint = CubicSpline(time, position, bc_type=((1, current_vel), (1, np.zeros(len(starting_joint)))),
                                      axis=0)
            interpolation_joint = np.array([cubic_joint(i) for i in range(interpolation_steps)])

        if interpolation == "linear":
            delta_x_per_step = (last_goal_joint - starting_joint) / interpolation_steps
            interpolation_joint = np.array(
                [starting_joint + i * delta_x_per_step for i in range(1, interpolation_steps + 1)])

        return interpolation_joint


    # set Kv and Kp
    def kp_cartisian(timestep, dimension):
        """
        input: timestep : how many steps in a command
        kp: 3-dim parameters, a cubic line connected with 5 points
        kv: kv=2*sqrt(kp)*damping
        damping: in range [0,1], here we set 0.1 for end-effector
        :return: kp, kv
        """
        damping = 0.1

        kp_cubic = []

        for dim in range(dimension):
            kp_dim = []
            random.seed(dim)
            for i in range(5):
                # here we choose the kp in range [25, 300]
                kp_dim.append(random.uniform(25, 300))

            # we sort kp from the largest to the smallest
            kp_dim.sort(reverse=True)

            x = np.linspace(0, timestep, num=5, endpoint=True)
            y = kp_dim
            f = interp1d(x, y, kind="cubic")

            # num 取值不确定
            xnew = np.linspace(0, timestep, num=timestep, endpoint=True)
            kp_cubic = np.hstack((kp_cubic, f(xnew)))
            kp_cubic = np.array(kp_cubic.reshape(dimension, timestep))

        # here we set the value of kp, which is smaller than 0, to 0
        for i in range(kp_cubic.shape[0]):
            for j in range(kp_cubic.shape[1]):
                if kp_cubic[i, j] < 0:
                    kp_cubic[i, j] = 0

        kv_cubic = 2 * np.sqrt(kp_cubic) * damping

        return kp_cubic, kv_cubic


    def kp_kv_joint(value):
        """

        :return: kp list of each joints
        """
        damping = 0.5
        kp = np.ones(14) * value

        kv = 2 * np.sqrt(kp) * damping

        return kp, kv


    def update_model_joint():
        """
        Updates the state of the robot used to compute the control command
        :param sim:
        :param joint_index:
        :param id_name:
        :return:
        """

        # calculate the position and velocity of each joint
        current_joint_position_right = [sim.data.qpos[x] for x in range(7)]
        current_joint_position_left = [sim.data.qpos[x] for x in range(22, 29)]
        current_joint_velocity_right = [sim.data.qvel[x] for x in range(7)]
        current_joint_velocity_left = [sim.data.qvel[x] for x in range(22, 29)]

        current_joint_position = np.hstack((current_joint_position_right, current_joint_position_left))
        current_joint_velocity = np.hstack((current_joint_velocity_right, current_joint_velocity_left))

        return current_joint_position, current_joint_velocity


    def update_model_eeffector():
        # calculate the position, orientation, linear-velocity and angle_velocity of the end-effector

        body_name_right = "right_endeffector_link"
        body_name_left = "left_endeffector_link"

        current_position_right = sim.data.body_xpos[sim.model.body_name2id(body_name_right)]
        current_orientation_mat_right = sim.data.body_xmat[sim.model.body_name2id(body_name_right)].reshape([3, 3])
        current_orientation_quat_right = sim.data.body_xquat[sim.model.body_name2id(body_name_right)]
        current_lin_velocity_right = sim.data.body_xvelp[sim.model.body_name2id(body_name_right)]
        current_ang_velocity_right = sim.data.body_xvelr[sim.model.body_name2id(body_name_right)]
        list_right = [current_position_right, current_orientation_quat_right, current_lin_velocity_right,
                      current_ang_velocity_right]

        current_position_left = sim.data.body_xpos[sim.model.body_name2id(body_name_left)]
        current_orientation_mat_left = sim.data.body_xmat[sim.model.body_name2id(body_name_left)].reshape([3, 3])
        current_orientation_quat_left = sim.data.body_xquat[sim.model.body_name2id(body_name_right)]
        current_lin_velocity_left = sim.data.body_xvelp[sim.model.body_name2id(body_name_left)]
        current_ang_velocity_left = sim.data.body_xvelr[sim.model.body_name2id(body_name_left)]
        list_left = [current_position_left, current_orientation_quat_left, current_lin_velocity_left,
                     current_ang_velocity_left]

        return list_right, list_left


    def update_joint_pybullet():
        """

        :return: the position of each joints in pybullet
        """

        current_joint_position = [p.getJointState(darias, x)[0] for x in range(14)][0:14]
        current_joint_velocity = [p.getJointState(darias, x)[1] for x in range(14)][0:14]

        return current_joint_position


    def update_eeffector_pybullet():
        """

        :return: the position of left and right end-effector in pybullet
        """

        right_position = p.getLinkState(darias, 8)[0]  # link world position
        left_position = p.getLinkState(darias, 16)[0]

        return right_position, left_position


    def update_eefector_pybullet_orn():
        """

        :return: the Cartesian orientation of center of mass, in quaternion [x, y, z, w]
        """
        right_orn = p.getLinkState(darias, 8)[1]  # link world position
        left_orn = p.getLinkState(darias, 16)[1]

        return right_orn, left_orn


    def calculate_torques(current_joint_position, current_vel, desired_joint_position, kp_joint, kv_joint):

        desired_joint_velocity = np.zeros(14)
        position_joint_error = - current_joint_position + desired_joint_position
        velocity_joint_error = -current_vel + desired_joint_velocity

        # kp,kv control
        torques = np.multiply(kp_joint, position_joint_error) + np.multiply(kv_joint, velocity_joint_error)

        return np.array(torques)


    def command_controll(current_pos, current_vel, desired_pos_joint, steps):
        pos = []
        errors = []

        for i in range(steps):
            # while(STEPS):
            Gravity()

            torques1 = calculate_torques(current_pos, current_vel, desired_pos_joint, kp_joint, kv_joint)

            sim.data.ctrl[0:7] = torques1[0:7]
            sim.data.ctrl[22:29] = torques1[7:14]

            controll_grippers(index="ON")

            sim.step()

            current_pos, current_vel = update_model_joint()

            error = current_pos - desired_pos_joint

            pos.append(current_pos)
            errors.append(error)

            viewer.render()

        list_right, list_left = update_model_eeffector()
        eef_right_pos = list_right[0]
        eef_left_pos = list_left[0]
        pos = np.array(pos)
        errors = np.array(errors)

        right_end = current_pos[6]
        left_end = current_pos[13]

        a = sim.data.sensordata
        # print("Force and Torque:", a[:6])

        return current_pos, current_vel


    def alignment(force, torque, current_pos, current_orn):

        matrix_force = [[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]]
        matrix_torque = [[0, 0, 0],
                         [0, 0, 0],
                         [0, 0, 0]]

        desired_force = [0, 0, 35]
        desired_torque = [0, 0, 0]
        current_force = force
        current_torque = torque
        delta_pos = np.dot(matrix_force, np.array(desired_force) - np.array(current_force))
        delta_orn = np.dot(matrix_torque, np.array(desired_torque) - np.array(current_torque))

        new_pos = current_pos + delta_pos
        new_orn = current_orn + delta_orn

        print("before pos:", current_pos)
        print("before orn:", current_orn)
        print("new pos:", new_pos)
        print("new orn:", new_orn)

        return new_pos, new_orn


    def draw_pictures_before(iter, force_R, force_L, torque_R, torque_L):
        """

        :return:
        """

        force_R = np.array(force_R)
        force_L = np.array(force_L)
        torque_R = np.array(torque_R)
        torque_L = np.array(torque_L)
        print("force_R:", force_R)

        plt.figure(num=1)

        x = np.linspace(0, iter, num=iter, endpoint=True)

        plt.plot(x, force_R[:, 0], c='blue', lw=0.5, label='Force along x-axis')
        plt.plot(x, force_R[:, 1], c='red', lw=0.5, label='Force along y-axis')
        plt.plot(x, force_R[:, 2], c='green', lw=0.5, label='Force along z-axis')
        plt.xlabel('Steps')
        plt.ylabel('Force')
        plt.legend(loc='upper right')
        plt.grid(True)

        plt.savefig("right force before 10000")

        plt.figure(num=2)

        x = np.linspace(0, iter, num=iter, endpoint=True)
        plt.plot(x, force_L[:, 0], c='blue', lw=0.5, label='Force along x-axis')
        plt.plot(x, force_L[:, 1], c='red', lw=0.5, label='Force along y-axis')
        plt.plot(x, force_L[:, 2], c='green', lw=0.5, label='Force along z-axis')
        plt.xlabel('Steps')
        plt.ylabel('Force')
        plt.legend(loc='upper right')
        plt.grid(True)

        plt.savefig("left force before 10000")

        plt.figure(num=3)
        plt.plot(x, torque_R[:, 0], c='blue', lw=0.5, label='Moment along x-axis')
        plt.plot(x, torque_R[:, 1], c='red', lw=0.5, label='Moment along y-axis')
        plt.plot(x, torque_R[:, 2], c='green', lw=0.5, label='Moment along z-axis')
        plt.xlabel('Steps')
        plt.ylabel('Moment')
        plt.legend(loc='upper right')
        plt.grid(True)
        plt.savefig("right moment before 10000")

        plt.figure(num=4)
        plt.plot(x, torque_L[:, 0], c='blue', lw=0.5, label='Moment along x-axis')
        plt.plot(x, torque_L[:, 1], c='red', lw=0.5, label='Moment along y-axis')
        plt.plot(x, torque_L[:, 2], c='green', lw=0.5, label='Moment along z-axis')
        plt.xlabel('Steps')
        plt.ylabel('Moment')
        plt.legend(loc='upper right')
        plt.grid(True)
        plt.savefig("left moment before 10000")


    def draw_pictures_after(force_after_r, force_after_l, torque_after_r, torque_after_l):
        """

        :return:
        """
        force_after_r = np.array(force_after_r)
        force_after_l = np.array(force_after_l)
        torque_after_r = np.array(torque_after_r)
        torque_after_l = np.array(torque_after_l)
        number = np.size(force_after_r[:, 0])

        plt.figure(num=5)
        x = np.linspace(10000, 10000 + number, num=number, endpoint=True)
        plt.plot(x, force_after_r[:, 0], c='blue', lw=1, label='Force along x-axis')
        plt.plot(x, force_after_r[:, 1], c='red', lw=1, label='Force along y-axis')
        plt.plot(x, force_after_r[:, 2], c='green', lw=1, label='Force along z-axis')
        plt.xlabel('Steps')
        plt.ylabel('Force')
        plt.legend(loc='upper right')
        plt.grid(True)
        # plt.savefig("right force without alignment")
        plt.savefig("right force with alignment")

        plt.figure(num=6)
        x = np.linspace(10000, 10000 + number, num=number, endpoint=True)
        plt.plot(x, force_after_l[:, 0], c='blue', lw=1, label='Force along x-axis')
        plt.plot(x, force_after_l[:, 1], c='red', lw=1, label='Force along y-axis')
        plt.plot(x, force_after_l[:, 2], c='green', lw=1, label='Force along z-axis')
        plt.xlabel('Steps')
        plt.ylabel('Force')
        plt.legend(loc='upper right')
        plt.grid(True)
        # plt.savefig("left force without alignment")
        plt.savefig("left force with alignment")

        plt.figure(num=7)
        x = np.linspace(10000, 10000 + number, num=number, endpoint=True)
        plt.plot(x, torque_after_r[:, 0], c='blue', lw=1, label='Moment along x-axis')
        plt.plot(x, torque_after_r[:, 1], c='red', lw=1, label='Moment along y-axis')
        plt.plot(x, torque_after_r[:, 2], c='green', lw=1, label='Moment along z-axis')
        plt.xlabel('Steps')
        plt.ylabel('Moment')
        plt.legend(loc='upper right')
        plt.grid(True)
        # plt.savefig("right moment without alignment")
        plt.savefig("right moment with alignment")

        plt.figure(num=8)
        x = np.linspace(10000, 10000 + number, num=number, endpoint=True)
        plt.plot(x, torque_after_r[:, 0], c='blue', lw=1, label='Moment along x-axis')
        plt.plot(x, torque_after_r[:, 1], c='red', lw=1, label='Moment along y-axis')
        plt.plot(x, torque_after_r[:, 2], c='green', lw=1, label='Moment along z-axis')
        plt.xlabel('Steps')
        plt.ylabel('Moment')
        plt.legend(loc='upper right')
        plt.grid(True)
        # plt.savefig("left moment without alignment")
        plt.savefig("left moment with alignment")


    def update_sensordata():
        a_after = sim.data.sensordata
        a_after_r_force = copy.deepcopy(a_after[0:3])
        a_after_r_torque = copy.deepcopy(a_after[3:6])
        a_after_l_force = copy.deepcopy(a_after[6:9])
        a_after_l_torque = copy.deepcopy(a_after[9:12])

        return a_after_r_force, a_after_r_torque, a_after_l_force, a_after_l_torque


    def get_orientation_error(target_orn, current_orn):
        """
        Returns the difference between two quaternion orientations as a 3 DOF numpy array.
        For use in an impedance controller / task-space PD controller.

        Args:
            target_orn: 4-dim iterable, desired orientation as a (w, x, y, z) quaternion
            current_orn: 4-dim iterable, current orientation as a (w, x, y, z) quaternion

        Returns:
            orn_error: 3-dim numpy array for current orientation error, corresponds to
                (target_orn - current_orn)
        """
        pinv = np.zeros((3, 4))
        pinv[0, :] = [-current_orn[1], current_orn[0], -current_orn[3], current_orn[2]]
        pinv[1, :] = [-current_orn[2], current_orn[3], current_orn[0], -current_orn[1]]
        pinv[2, :] = [-current_orn[3], -current_orn[2], current_orn[1], current_orn[0]]
        orn_error = 2.0 * pinv.dot(np.array(target_orn))
        return orn_error


    def draw_picture(joint_index, steps, joint_pos, desired_pos_joint):

        x = np.linspace(0, steps, num=steps, endpoint=True)
        y = joint_pos[:, joint_index]
        y_des = np.ones((steps,)) * desired_pos_joint[joint_index]
        # z = errors[:, n]
        # print(y.shape)
        # print(y_des.shape)

        plt.plot(x, y, 'o', x, y_des, '--')
        plt.savefig("position_of_joint_%d.png" % joint_index)


    def move_girppers(desired1, desired1L,
                      desired2, desired2L,
                      desired3, desired3L,
                      desired4, desired4L,
                      desired5, desired5L):
        """
        move the fingers to the desired positions
        :param desired1: right thumb
        :param desired1L: left thumb
        :param desired2: right index finger
        :param desired2L: left index finger
        :param desired3: right middle finger
        :param desired3L: left middle finger
        :param desired4: right ringer finger
        :param desired4L: left ringer finger
        :param desired5: right small finger
        :param desired5L: left small finger

        """
        # small finger
        sim.data.ctrl[7:10] = desired5
        sim.data.ctrl[29:32] = desired5L
        # print("desired of right,left small finger", desired5)
        # print("right small finger pos:", sim.data.qpos[7:10])
        # print("left small finger pos:", sim.data.qpos[29:32])

        # ringer finger
        sim.data.ctrl[10:13] = desired4
        sim.data.ctrl[32:35] = desired4L
        # print("desired of right ringer finger", desired4)
        # print("right ringer finger pos:", sim.data.qpos[10:13])
        # print("left ringer finger pos:", sim.data.qpos[32:35])
        # # middle finger
        sim.data.ctrl[13:16] = desired3
        sim.data.ctrl[35:38] = desired3L
        # print("desired of right middle finger", desired3)
        # print("right middle finger pos:", sim.data.qpos[13:16])
        # print("left middle finger pos:", sim.data.qpos[35:38])

        # index finger
        sim.data.ctrl[16:19] = desired2
        sim.data.ctrl[38:41] = desired2L
        # print("desired of right index finger", desired2)
        # print("right index finger pos:", sim.data.qpos[16:19])
        # print("left index finger pos:", sim.data.qpos[38:41])

        # thumb finger
        sim.data.ctrl[19:22] = desired1
        sim.data.ctrl[41:44] = desired1L


    # grippers control
    def controll_grippers(index):
        # thumb, index (z-axis)
        if index == "ON":
            desired1 = [-0.2, -0.3, 0.5]
            desired1L = [0.2, -0.3, 0.5]
            desired2 = [-0.3, -0.3, 0.5]
            desired2L = [0.3, -0.3, 0.5]
            # middle
            desired3 = [0.0, -0.3, 0.5]
            desired3L = desired3

            # Ring
            desired4 = [0.2, -0.3, 0.5]
            desired4L = [-0.2, -0.3, 0.5]
            # small
            desired5 = [0.3, -0.3, 0.5]
            desired5L = [-0.3, -0.3, 0.5]

            move_girppers(desired1, desired1L,
                          desired2, desired2L,
                          desired3, desired3L,
                          desired4, desired4L,
                          desired5, desired5L)

        if index == "OFF":
            # thumb
            desired1_end = [0.0, 0.5, 0.5]
            desired1_endL = [0.0, 0.5, 0.5]
            # index
            desired2_end = [0.0, 0.5, 0.5]
            desired2_endL = [0.0, 0.5, 0.5]

            # middle
            desired3_end = [0.0, 0.5, 0.5]
            desired3_endL = desired3_end

            # Ring
            desired4_end = [0.0, 0.5, 0.5]
            desired4_endL = [0.0, 0.5, 0.5]

            # small
            desired5_end = [0.0, 0.5, 0.5]
            desired5_endL = [0.0, 0.5, 0.5]

            curr_pos1 = sim.data.qpos[19:22]
            curr_vel1 = sim.data.qvel[19:22]
            curr_pos1L = sim.data.qpos[41:44]
            curr_vel1L = sim.data.qvel[41:44]

            curr_pos2 = sim.data.qpos[16:19]
            curr_vel2 = sim.data.qvel[16:19]
            curr_pos2L = sim.data.qpos[38:41]
            curr_vel2L = sim.data.qvel[38:41]

            curr_pos3 = sim.data.qpos[13:16]
            curr_vel3 = sim.data.qvel[13:16]
            curr_pos3L = sim.data.qpos[35:38]
            curr_vel3L = sim.data.qvel[35:38]

            curr_pos4 = sim.data.qpos[10:13]
            curr_vel4 = sim.data.qvel[10:13]
            curr_pos4L = sim.data.qpos[32:35]
            curr_vel4L = sim.data.qvel[32:35]

            curr_pos5 = sim.data.qpos[7:10]
            curr_vel5 = sim.data.qvel[7:10]
            curr_pos5L = sim.data.qpos[29:32]
            curr_vel5L = sim.data.qvel[29:32]

            num = 10000
            desired1 = interpolate_joint(starting_joint=curr_pos1, last_goal_joint=desired1_end,
                                         interpolation_steps=num, current_vel=curr_vel1, interpolation="cubic")

            desired1L = interpolate_joint(starting_joint=curr_pos1L, last_goal_joint=desired1_endL,
                                          interpolation_steps=num, current_vel=curr_vel1L, interpolation="cubic")

            desired2 = interpolate_joint(curr_pos2, desired2_end, interpolation_steps=num, current_vel=curr_vel2,
                                         interpolation="cubic")
            desired2L = interpolate_joint(curr_pos2L, desired2_endL, interpolation_steps=num, current_vel=curr_vel2L,
                                          interpolation="cubic")

            desired3 = interpolate_joint(curr_pos3, desired3_end, interpolation_steps=num, current_vel=curr_vel3,
                                         interpolation="cubic")
            desired3L = interpolate_joint(curr_pos3L, desired3_endL, interpolation_steps=num, current_vel=curr_vel3L,
                                          interpolation="cubic")

            desired4 = interpolate_joint(curr_pos4, desired4_end, interpolation_steps=num, current_vel=curr_vel4,
                                         interpolation="cubic")
            desired4L = interpolate_joint(curr_pos4L, desired4_endL, interpolation_steps=num, current_vel=curr_vel4L,
                                          interpolation="cubic")

            desired5 = interpolate_joint(curr_pos5, desired5_end, interpolation_steps=num, current_vel=curr_vel5,
                                         interpolation="cubic")
            desired5L = interpolate_joint(curr_pos5L, desired5_endL, interpolation_steps=num, current_vel=curr_vel5L,
                                          interpolation="cubic")

            steps = 300
            for i in range(len(desired1)):
                move_girppers(desired1[i], desired1L[i],
                              desired2[i], desired2L[i],
                              desired3[i], desired3L[i],
                              desired4[i], desired4L[i],
                              desired5[i], desired5L[i])


    def command_control_end(current_pos_joint, current_vel_joint, desired_pos_joint):

        flag = True
        iter = 0

        # the force and orn of right and left hand in the whole process

        force_R = []
        force_L = []

        torque_R = []
        torque_L = []

        # the force after 10000 steps:

        force_after_r = []
        torque_after_r = []

        force_after_l = []
        torque_after_l = []

        kp_joint, kv_joint = kp_kv_joint(500)

        # current_orn_r = [0, math.pi, math.pi]
        # current_orn_l = [0, math.pi, 0]
        # orn_right = p.getQuaternionFromEuler(current_orn_r)  # the data from updata
        # orn_left = p.getQuaternionFromEuler(current_orn_l)
        print("first desired joint:", desired_pos_joint)





        while (True):

           # calculate the current orientation of end-effector in pybullet
            orn_right, orn_left = update_eefector_pybullet_orn()
            current_orn_r = p.getEulerFromQuaternion(orn_right)
            current_orn_l = p.getEulerFromQuaternion(orn_left)

            # calculate the desired torque each time
            torque = calculate_torques(current_pos_joint, current_vel_joint, desired_pos_joint, kp_joint, kv_joint)
            sim.data.ctrl[0:7] = torque[0:7]
            sim.data.ctrl[22:29] = torque[7:]
            iter += 1

            sim.step()
            # read and update the new pos end velocity
            current_pos_joint, current_vel_joint = update_model_joint()

            # read the force and torque:
            # a_after = sim.data.sensordata
            # a_after_r_force = a_after[0:3]
            # a_after_r_torque = a_after[3:6]
            # a_after_l_force = a_after[6:9]
            # a_after_l_torque = a_after[9:12]

            # add the force and torque to the list
            force_r, torque_r, force_l, torque_l = update_sensordata()

            force_R.append(force_r)

            force_L.append(force_l)

            torque_R.append(torque_r)

            torque_L.append(torque_l)

            # print("iter:", iter)

            # print("force R:", force_r)
            # print("FORCE R:", force_R)
            # print("the size of FORCE:", np.shape(force_R))

            list_right, list_left = update_model_eeffector()
            eef_right_pos = list_right[0]
            eef_right_orn = list_left[1]
            eef_left_pos = list_left[0]
            eef_left_orn = list_left[1]
            # print("the pos of right end-effector:", eef_right_pos)
            # print("the pos of left end-effector:", eef_left_pos)
            # print("force x", force_x)

            if (iter == 10000):
                draw_pictures_before(iter, force_R, force_L, torque_R, torque_L)


            if (iter >= 8000) and (iter <= 10000):
                a_after_r_force, a_after_r_torque, a_after_l_force, a_after_l_torque = update_sensordata()

                force_after_r.append(a_after_r_force)

                force_after_l.append(a_after_l_force)

                torque_after_r.append(a_after_r_torque)

                torque_after_l.append(a_after_l_torque)

                # begin the alignment of the NEW desired pos and orn

                #adjust each 200 steps:
                if((iter-8000)%200 ==0):
                    kp_joint, kv_joint = kp_kv_joint(15)

                    # right hand adjust
                    desired_pos_r, desired_orn_r = alignment(a_after_r_force, a_after_r_torque,
                                                             eef_right_pos, current_orn_r)
                    orn_right1 = p.getQuaternionFromEuler(desired_orn_r)
                    # left hand adjust
                    desired_pos_l, desired_orn_l = alignment(a_after_l_force, a_after_l_torque,
                                                             eef_left_pos, current_orn_l)

                    orn_left1 = p.getQuaternionFromEuler(desired_orn_l)

                    desired_pos_joint = inverse_kinematics1(desired_pos_r, orn_right, desired_pos_l, orn_left,
                                                            current_pos_joint, Index=2)
                    print("desired_pos_joint after:", desired_pos_joint)

                if (iter == 10000):
                    # show the pictures after alignment:


                    draw_pictures_after(force_after_r, force_after_l, torque_after_r, torque_after_l)

            #     the hand begins to grasp the cloth:
            #
            #     controll_grippers(index="OFF")

            viewer.render()

        # x = np.linspace(0, iter, num=iter, endpoint=True)
        # a = np.array(a_after_force_right)
        # plt.plot(x,a[:,0])
        # plt.ylim(-7,0)
        # plt.savefig("force")

        #  print("force and torque", a)

        list_right, list_left = update_model_eeffector()
        eef_right_pos = list_right[0]
        eef_left_pos = list_left[0]
        print("the pos of right-end  :", eef_right_pos)
        print("the pos of left-end:", eef_left_pos)
        # print("desired pos joint:", desired_pos_joint)
        # print("current pos joint:", current_pos)


    # ----------------begin the mujoco--------------------#

    viewer = MujocoPyRenderer(sim)
    # set kp, kv to a fixed value:
    kp_joint, kv_joint = kp_kv_joint(400)

    # initialized the position and velocity of joints and end-effector:
    initial_pos = np.hstack(([sim.data.qpos[x] for x in range(7)], [sim.data.qpos[x] for x in range(22, 29)]))
    initial_vel = np.hstack(([sim.data.qvel[x] for x in range(7)], [sim.data.qvel[x] for x in range(22, 29)]))
    right_end_pos, left_end_pos = update_eeffector_pybullet()
    joint_pos_pybullet = update_joint_pybullet()

    print("the initial pos of joints in mujoco:", initial_pos)
    print("the initial pos of joints in pybullet:", joint_pos_pybullet)

    body_name_right = "right_endeffector_link"
    body_name_left = "left_endeffector_link"

    eff_pos_right_initial = sim.data.body_xpos[sim.model.body_name2id(body_name_right)]
    eff_pos_left_initial = sim.data.body_xpos[sim.model.body_name2id(body_name_left)]

    print("the initial pos of right eff in mujoco:", eff_pos_right_initial, np.shape(eff_pos_right_initial))
    print("the initial pos of left eff in mujoco:", eff_pos_left_initial)
    print("the initial pos of right eff in pybullet:", right_end_pos, np.shape(right_end_pos))
    print("the initial pos of left eff in pybullet:", left_end_pos)

    pos_mj = np.reshape(np.hstack((eff_pos_right_initial, eff_pos_left_initial)), (6, 1))
    pos_py = np.reshape(np.hstack((right_end_pos, left_end_pos)), (6, 1))

    # here we set the desired pos of right, left end-effector
    # set the desired pos and orientations of end-effector

    # set x to a fixed value in mujoco frame

    # # Trajectory 1:
    pos_right_mj = [0.54, -0.62, 1.15]
    pos_left_mj = [0.54, 0.62, 1.15]
    # pos_right_mj = [1.244, -0.764, 1.25]
    # pos_left_mj = [0.83, 0.83, 1.25]
    print("desired pos right:", pos_right_mj)
    print("desired pos left:", pos_left_mj)

    pos_right_mj2 = [0.53, -0.18, 0.96]
    pos_left_mj2 = [0.53, 0.18, 0.96]

    # Trajectory2:
    # pos_right_mj = [0.56, -0.55, 1.15]
    # pos_left_mj = [0.54, 0.62, 1.15]
    # # pos_right_mj = [1.244, -0.764, 1.25]
    # # pos_left_mj = [0.83, 0.83, 1.25]
    # print("desired pos right:", pos_right_mj)
    # print("desired pos left:", pos_left_mj)
    #
    # pos_right_mj2 = [0.7, 0.0, 0.95]
    # pos_left_mj2 = [0.56, 0.19, 0.95]

    # The trajectory 1 of right hand:
    # pos_right_mj = [0.54, -0.62, 1.15]
    # pos_right_mj2 = [0.53, -0.18, 0.96]
    # pos_left_mj = eff_pos_left_initial
    # pos_left_mj2 = eff_pos_left_initial

    # # the trajectory 2 of right hand
    # pos_right_mj = [0.54, -0.62, 1.15]
    # pos_right_mj2 = [0.53, 0.18, 1.15]
    # pos_right_mj3 = [0.53, 0.18, 0.96]
    # pos_left_mj = eff_pos_left_initial
    # pos_left_mj2 = eff_pos_left_initial
    #
    # # The trajectory 3 of right hand:
    # pos_right_mj = [0.56, -0.55, 1.15]
    # pos_right_mj2 = [0.7, 0.0, 0.95]
    # pos_left_mj = eff_pos_left_initial
    # pos_left_mj2 = eff_pos_left_initial
    #
    # # The trajectory 4 of right hand:
    # pos_right_mj = [0.48, -0.52, 1.15]
    # pos_right_mj2 = [0.49, -0.05, 0.99]
    # pos_right_mj3 = [0.49, -0.05, 0.95]
    # pos_left_mj = eff_pos_left_initial
    # pos_left_mj2 = eff_pos_left_initial

    # desired_end effector orientation points down, not up (along z-axis)
    orn_right = p.getQuaternionFromEuler([0, math.pi, math.pi])
    orn_left = p.getQuaternionFromEuler([0, math.pi, 0])

    # the desired joint pos of the arms
    last_goal_joint1 = inverse_kinematics1(pos_right_mj, orn_right, pos_left_mj, orn_left, joint_poses_previous=0,
                                           Index=1)

    # begin the command:
    current_pos = initial_pos
    current_vel = initial_vel
    interpolation_steps = 15


    # control1: with the sub-goals:
    def controll1(current_pos, current_vel):
        current_pos = current_pos
        current_vel = current_vel

        for j in range(len(sub_goals)):
            desired_joint_pos = interpolate_joint(current_pos, sub_goals[j], interpolation_steps,
                                                  current_vel, interpolation="cubic")
            print("desired joint pos:", desired_joint_pos)
            print("current vel:", current_vel)

            for i in range(interpolation_steps):
                current_pos, current_vel = command_controll(current_pos, current_vel, desired_joint_pos[i], steps=800,
                                                            grippers=True)

        command_control_end(current_pos, current_vel, sub_goals[j])


    def controll2(current_pos, current_vel, desired_goal_joint, index, interpolation_steps, steps):

        desired_joint_pos = interpolate_joint(current_pos, desired_goal_joint, interpolation_steps,
                                              current_vel, interpolation="cubic")

        for i in range(interpolation_steps):
            current_pos, current_vel = command_controll(current_pos, current_vel, desired_joint_pos[i], steps)

        if (index == 2):
            print("current pos1:", current_pos)
            command_control_end(current_pos, current_vel, desired_goal_joint)

        print("desired pos of joint:", desired_goal_joint)
        print("current pos2:", current_pos)
        return current_pos, current_vel

        list_right, list_left = update_model_eeffector()
        eef_right_pos = list_right[0]
        eef_left_pos = list_left[0]
        print("the pos of right-end :", eef_right_pos)
        print("the pos of left-end:", eef_left_pos)
        print("the orn of right-end:", list_right[1])


    # reach the first sub-goal
    current_pos1, current_vel1 = controll2(initial_pos, initial_vel, last_goal_joint1, index=1,
                                           interpolation_steps=15, steps=600)

    # test the orientation here
    orn_goal1_m = sim.data.body_xquat[sim.model.body_name2id("right_endeffector_link")]
    orn_goal1_p = p.getLinkState(darias, 8)[1]
    error = get_orientation_error(orn_right, orn_goal1_p)
    # test quat-> euler angle
    euler_goal1_p = p.getEulerFromQuaternion(orn_goal1_p)
    euler_goal1_p_des = p.getEulerFromQuaternion(orn_right)

    print("orn_goal1_m:", orn_goal1_m)
    print("orn_goal1_p:", orn_goal1_p)
    print("desired orn goal1 p:", orn_right)
    print("error:", error)
    print("euler_goal:", euler_goal1_p)
    print("desired euler goal:", euler_goal1_p_des)

    # calculate the second sub-goal
    last_goal_joint2 = inverse_kinematics1(pos_right_mj2, orn_right, pos_left_mj2, orn_left,
                                           joint_poses_previous=current_pos1, Index=2)

    # reach the second sub-goal and begin to alignment
    current_pos2, current_vel2 = controll2(current_pos1, current_vel1, last_goal_joint2, index=2,
                                           interpolation_steps=16, steps=500)

    # last_goal_joint3 = inverse_kinematics1(pos_right_mj3, orn_right, pos_left_mj2, orn_left,
    #                                        joint_poses_previous=current_pos1, Index=2)
    # current_pos3, current_vel3 = controll2(current_pos2, current_vel2, last_goal_joint3, index=2,
    #                                        interpolation_steps=16, steps=500)

    # Force_Torque= [-3.52033165, - 20.3238254 , 19.30627205,   4.72303898, - 0.7073755, 0.38755437]
    #
    # current_pos=[0.55018473 ,- 0.18667273,  0.97506269]
    # orn_right2 = p.getQuaternionFromEuler([0.1, math.pi, math.pi])
    # print(orn_right2)
    # print(orn_right)
    # print([0.1, math.pi, math.pi])
    # new_pos, new_orn = alignment(Force_Torque, current_pos,[0, math.pi, math.pi])

    # current_pos3, current_vel3= controll2(current_pos2, current_vel2,last_goal_joint3, index=2,
    #                                        interpolation_steps=16, steps=500)
    #

# print(last_goal_joint1)

# controll1(initial_pos, initial_vel)
# while(True):
#
#     Gravity()
#     #a = sim.data.sensordata
#     #print(a)
#     sim.step()
#     viewer.render()
