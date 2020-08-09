import numpy as np
import pybullet as p
import os
import math
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


if __name__ == "__main__":

    fname = os.getcwd() + "/robosuite/models/assets/bullet_data/darias_description/robots/darias.urdf"


    # physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    physicsClient = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)

    darias = p.loadURDF(fname)
   # darias = p.loadURDF(fname, [0, 0, 0], [0, 0, 0, 1])  # set the basePosition and baseOrientation
    #darias = p.loadMJCF(fname2)

    joint_num = p.getNumJoints(darias)

    print("the number of joint:", joint_num)
    # print("the name of joints:", joint_name[1])

    # get the all index and joints of darias:
    for i in range(joint_num):
        joint_name = p.getJointInfo(darias, i)
        print(joint_name[0], joint_name[1])

     # lower limits for null space
    ll = [-2.967059, -2.094395, -2.967059, -2.094395, -2.967059, -2.094395, -2.967059,
          -2.967059, -2.094395, -2.967059, -2.094395, -2.967059, -2.094395, -2.094395]
    # upper limits for null space
    ul = [2.967059, 2.094395, 2.967059, 2.094395, 2.967059, 2.094395, 2.967059,
          2.967059, 2.094395, 2.967059, 2.094395, 2.967059, 2.094395, 2.094395]
    # joint ranges for null space
    jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
    # restposes for null space
    # rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
    # joint damping coefficents
    jd = np.eye(7) * 0.1

    # for i in range(7):
    #     p.resetJointState(darias, i + 2, rp[i])
    #     p.resetJointState(darias, i + 10, rp[i])

    p.setRealTimeSimulation(1)




    def inverse_kinematics(
            target_position_right,
            target_orientation_right,
            target_position_left,
            target_orientation_left,

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
        maxIter= 100
        threshold=1e-4
        closeEnough= False
        iter = 0
        dist2 = 1e30
        joint_poses=[]

        while(not closeEnough and iter< maxIter):
            # use nullspace
            jointPoses1 = list(
                p.calculateInverseKinematics(darias, effector_right, target_position_right, target_orientation_right,
                                             ll[0:7], ul[0:7],
                                             jr))
           # print("jointPoses1:", jointPoses1)
            jointPoses1_ = list(
                p.calculateInverseKinematics(darias, effector_right, target_position_right,
                                             ll[0:7], ul[0:7],
                                             jr))

            jointPoses2 = list(
                p.calculateInverseKinematics(darias, effector_left, target_position_left, target_orientation_left,
                                             ll[7:14], ul[7:14],
                                             jr))
            for i in range(7, 14):
                jointPoses1[i] = jointPoses2[i]

            jointPoses= jointPoses1


            j=0
            for i in range(joint_num):
                jointInfo = p.getJointInfo(darias, i)
                qIndex = jointInfo[3]

                if qIndex > -1:
                    p.resetJointState(darias, i, jointPoses[j])
                    j=j+1
                   # print("index:", qIndex-i)

            ls_r =list(p.getLinkState(darias, effector_right)[4])
            ls_l =list(p.getLinkState(darias, effector_left)[4])
            diff1_r = np.array(target_position_right) - np.array(ls_r)
            diff1_l = np.array(target_position_left) - np.array(ls_l)
            dist2_r = np.sqrt(diff1_r[0] * diff1_r[0] + diff1_r[1] * diff1_r[1] + diff1_r[2] * diff1_r[2])
            dist2_l = np.sqrt(diff1_l[0] * diff1_l[0] + diff1_l[1] * diff1_l[1] + diff1_l[2] * diff1_l[2])
            closeEnough =(dist2_r < threshold and dist2_l<threshold)
            iter = iter+1

        print("iter=",iter)
        print("dist2_r:", dist2_r)
        print("dist2_l:", dist2_l)
        print("ls_r:", ls_r)
        print("ls_l:", ls_l)

        return np.array(jointPoses)


        # set x to a fixed value in mujoco frame


    pos_right = [0.48, -1.08, 1.5]
    pos_left = [0.48, 1.08, 1.5]

    # end effector points down, not up
    orn_right = p.getQuaternionFromEuler((math.pi, 0, math.pi))
    orn_left = p.getQuaternionFromEuler([0, math.pi, 0])
    print("orn_right:", np.array(orn_right))
    print("orn_left:",np.array(orn_left))
    euler_right = p.getEulerFromQuaternion(np.array(orn_right))
    axis_angle = p.getAxisAngleFromQuaternion(np.array(orn_right))
    print("axis_angle_r:", axis_angle)
    euler_left = p.getEulerFromQuaternion(np.array(orn_left))
    orn_right2 = p.getQuaternionFromEuler(euler_right)
    euler_right2=p.getEulerFromQuaternion(orn_right2)
    print("euler_right:", euler_right)
    print("euler_left:", euler_left)
    print("orn right:", orn_right2)
    print("euler_right2:", euler_right2)

    # For testing whether a number is close to zero
    _FLOAT_EPS = np.finfo(np.float64).eps
    _EPS4 = _FLOAT_EPS * 4.0

    def euler2quat(euler):
        """ Convert Euler Angles to Quaternions.  See rotation.py for notes """
        euler = np.asarray(euler, dtype=np.float64)
        assert euler.shape[-1] == 3, "Invalid shape euler {}".format(euler)

        ai, aj, ak = euler[..., 2] / 2, -euler[..., 1] / 2, euler[..., 0] / 2
        si, sj, sk = np.sin(ai), np.sin(aj), np.sin(ak)
        ci, cj, ck = np.cos(ai), np.cos(aj), np.cos(ak)
        cc, cs = ci * ck, ci * sk
        sc, ss = si * ck, si * sk

        quat = np.empty(euler.shape[:-1] + (4,), dtype=np.float64)
        quat[..., 0] = cj * cc + sj * ss
        quat[..., 3] = cj * sc - sj * cs
        quat[..., 2] = -(cj * ss + sj * cc)
        quat[..., 1] = cj * cs - sj * sc
        return quat


    def mat2euler(mat):
        """ Convert Rotation Matrix to Euler Angles.  See rotation.py for notes """
        mat = np.asarray(mat, dtype=np.float64)
        assert mat.shape[-2:] == (3, 3), "Invalid shape matrix {}".format(mat)

        cy = np.sqrt(mat[..., 2, 2] * mat[..., 2, 2] + mat[..., 1, 2] * mat[..., 1, 2])
        condition = cy > _EPS4
        euler = np.empty(mat.shape[:-1], dtype=np.float64)
        euler[..., 2] = np.where(condition,
                                 -np.arctan2(mat[..., 0, 1], mat[..., 0, 0]),
                                 -np.arctan2(-mat[..., 1, 0], mat[..., 1, 1]))
        euler[..., 1] = np.where(condition,
                                 -np.arctan2(-mat[..., 0, 2], cy),
                                 -np.arctan2(-mat[..., 0, 2], cy))
        euler[..., 0] = np.where(condition,
                                 -np.arctan2(mat[..., 1, 2], mat[..., 2, 2]),
                                 0.0)
        return euler


    def mat2quat(mat):
        """ Convert Rotation Matrix to Quaternion.  See rotation.py for notes """
        mat = np.asarray(mat, dtype=np.float64)
        assert mat.shape[-2:] == (3, 3), "Invalid shape matrix {}".format(mat)

        Qxx, Qyx, Qzx = mat[..., 0, 0], mat[..., 0, 1], mat[..., 0, 2]
        Qxy, Qyy, Qzy = mat[..., 1, 0], mat[..., 1, 1], mat[..., 1, 2]
        Qxz, Qyz, Qzz = mat[..., 2, 0], mat[..., 2, 1], mat[..., 2, 2]
        # Fill only lower half of symmetric matrix
        K = np.zeros(mat.shape[:-2] + (4, 4), dtype=np.float64)
        K[..., 0, 0] = Qxx - Qyy - Qzz
        K[..., 1, 0] = Qyx + Qxy
        K[..., 1, 1] = Qyy - Qxx - Qzz
        K[..., 2, 0] = Qzx + Qxz
        K[..., 2, 1] = Qzy + Qyz
        K[..., 2, 2] = Qzz - Qxx - Qyy
        K[..., 3, 0] = Qyz - Qzy
        K[..., 3, 1] = Qzx - Qxz
        K[..., 3, 2] = Qxy - Qyx
        K[..., 3, 3] = Qxx + Qyy + Qzz
        K /= 3.0
        # TODO: vectorize this -- probably could be made faster
        q = np.empty(K.shape[:-2] + (4,))
        it = np.nditer(q[..., 0], flags=['multi_index'])
        while not it.finished:
            # Use Hermitian eigenvectors, values for speed
            vals, vecs = np.linalg.eigh(K[it.multi_index])
            # Select largest eigenvector, reorder to w,x,y,z quaternion
            q[it.multi_index] = vecs[[3, 0, 1, 2], np.argmax(vals)]
            # Prefer quaternion with positive w
            # (q * -1 corresponds to same rotation as q)
            if q[it.multi_index][0] < 0:
                q[it.multi_index] *= -1
            it.iternext()
        return q


    def quat2mat(quat):
        """ Convert Quaternion to Euler Angles.  See rotation.py for notes """
        quat = np.asarray(quat, dtype=np.float64)
        assert quat.shape[-1] == 4, "Invalid shape quat {}".format(quat)

        w, x, y, z = quat[..., 0], quat[..., 1], quat[..., 2], quat[..., 3]
        Nq = np.sum(quat * quat, axis=-1)
        s = 2.0 / Nq
        X, Y, Z = x * s, y * s, z * s
        wX, wY, wZ = w * X, w * Y, w * Z
        xX, xY, xZ = x * X, x * Y, x * Z
        yY, yZ, zZ = y * Y, y * Z, z * Z

        mat = np.empty(quat.shape[:-1] + (3, 3), dtype=np.float64)
        mat[..., 0, 0] = 1.0 - (yY + zZ)
        mat[..., 0, 1] = xY - wZ
        mat[..., 0, 2] = xZ + wY
        mat[..., 1, 0] = xY + wZ
        mat[..., 1, 1] = 1.0 - (xX + zZ)
        mat[..., 1, 2] = yZ - wX
        mat[..., 2, 0] = xZ - wY
        mat[..., 2, 1] = yZ + wX
        mat[..., 2, 2] = 1.0 - (xX + yY)

        # For testing whether a number is close to zero
        _FLOAT_EPS = np.finfo(np.float64).eps

        return np.where((Nq > _FLOAT_EPS)[..., np.newaxis, np.newaxis], mat, np.eye(3))

    def quat2euler(quat):
        """ Convert Quaternion to Euler Angles.  See rotation.py for notes """
        return mat2euler(quat2mat(quat))


    quat=euler2quat([0, math.pi, 0])
    euler1=quat2euler(quat)
    euler2=p.getEulerFromQuaternion(quat)
    print("quat:", quat)
    print("euler1:", euler1)
    print("euler2:", euler2)

    last_goal_joint = inverse_kinematics(pos_right, orn_right, pos_left, orn_left)
    print("last_goal_joint:", last_goal_joint)






    eef_pos_in_world = np.array(p.getLinkState(darias, 8)[0])
    eef_orn_in_world = np.array(p.getLinkState(darias, 8)[1])

    eef_pos_in_local = np.array(p.getLinkState(darias, 8)[2])
    eef_orn_in_local = np.array(p.getLinkState(darias, 8)[3])

    linkframe_pos_in_world = np.array(p.getLinkState(darias, 8)[4])
    linkframe_orn_in_world = np.array(p.getLinkState(darias, 8)[5])


    print("right end-effector pos in world:", eef_pos_in_world)
    print("right orn in world:", eef_orn_in_world)
    print("right end-effector pos in local:", eef_pos_in_local)
    print("right orn in local:", eef_orn_in_local)
    print("link frame position in world:", linkframe_orn_in_world)
    print("link frame orientation in world:", linkframe_pos_in_world)



    joint_lower_limit=[]
    joint_upper_limit=[]
    joint_Max_Force=[]
    joint_Max_Velocity=[]
    joint_Damping=[]
    joint_Friction=[]
    qIndex=[]


    for i in range(joint_num):
        info = p.getJointInfo(darias, i)
        qIndex.append(info[3])
        joint_lower_limit.append(info[8])
        joint_upper_limit.append(info[9])
        joint_Max_Force.append(info[10])
        joint_Max_Velocity.append((info[11]))
        joint_Damping.append(info[6])
        joint_Friction.append(info[7])

    for i in range(joint_num):
        jointInfo = p.getJointInfo(darias, i)
        qIndex = jointInfo[3]

        # if qIndex > -1:
        #   #  p.resetJointState(darias, i, jointPoses[qIndex - i])
        #   print("i:", i)
        #   #print("qindex:",qIndex)
        #   #print("index:", qIndex - 5)




    # print("joint lower limit:", joint_lower_limit)
    # print("joint upper limit:", joint_upper_limit)
    print("qindex:", qIndex)
    ls_r = list(p.getLinkState(darias, 8)[4])
    print("ls_r:",ls_r)
    target_pos_r =[0.51, 1, 0.5]
    diff1=np.array(target_pos_r)-np.array(ls_r)
    print("diff1:", diff1)
    dist2=np.sqrt(diff1[0]*diff1[0]+diff1[1]*diff1[1]+diff1[2]*diff1[2])







    joint_current_pos=[p.getJointState(darias,x)[0] for x in range(joint_num)]
    print("joint current pos in pybullet:", joint_current_pos)

    print("joint damping:", joint_Damping)
    print("joint max force:", joint_Max_Force)
    print("joint max velocity:", joint_Max_Velocity)
    print("joint friction:", joint_Friction)


#############---------------mujoco version---------------------


    fname2 = os.getcwd() + "/robosuite/models/assets/robots/darias/darias_nohands.xml"

    mujoco_darias = MujocoXML(fname2)

    model = mujoco_darias
    mjpy_model = model.get_model(mode="mujoco_py")
    sim = MjSim(mjpy_model)


    joint_pos_mj=[sim.data.qpos[x] for x in range(14)]

    id_name = "right_endeffector_link"
    # id_name="R_SAA"
    current_position = []
    current_position = sim.data.body_xpos[sim.model.body_name2id(id_name)]
    #print("right_endeffector_link position:", current_position)
    current_orientation_mat = sim.data.body_xmat[sim.model.body_name2id(id_name)].reshape([3, 3])
    current_orn =sim.data.body_xquat[sim.model.body_name2id(id_name)]

    print("orientation:", current_orn)



    #print("right_endeffector_link orientation", current_position)

   # print("joint current pos in mujoco:", joint_pos_mj)






