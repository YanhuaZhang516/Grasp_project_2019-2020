import numpy as np
# import robosuite as suite
from robosuite import Env_SawyerRmp
from utils.transform_utils import mat_quatVel2angVel, getQuat, parseQuat, quat2mat

from rmp.rmp_base import *
from rmp.rmp_leaf import *
from robosuite.global_setting import *


def create_mappings(joint_index):
    psi = lambda q: env.f_psi(joint_index, q)
    J = lambda q: env.f_jcb(joint_index, q)
    J_dot = lambda q, dq: env.f_jcb_dot(joint_index, q, dq)

    return [psi, J, J_dot]


def create_4point_goal(pos, ori_axis, ori_angle):
    quat = getQuat(ori_axis, ori_angle)
    rot_mat = quat2mat(quat)
    d_o = np.array(pos)
    d_x = d_o + np.dot(rot_mat, [LENGTH_AXIS, 0, 0])
    d_y = d_o + np.dot(rot_mat, [0, LENGTH_AXIS, 0])
    d_z = d_o + np.dot(rot_mat, [0, 0, LENGTH_AXIS])

    return d_o, d_x, d_y, d_z


joint_index = 1

if __name__ == "__main__":

    # initialize the task
    env = Env_SawyerRmp(has_renderer=True,
                        ignore_done=True,
                        use_camera_obs=False,
                        control_freq=100, )
    env.reset()
    env.viewer.set_camera(camera_id=0)

    init_joint_pos = np.array([env.sim.data.qpos[x] for x in env._ref_joint_pos_indexes])
    init_joint_vel = np.array([env.sim.data.qvel[x] for x in env._ref_joint_vel_indexes])
    ctrl_range = env.sim.model.actuator_ctrlrange[0:7, :]

    rt = Rmp_Root('root')

    # Root and Joint limit avoidance
    root_leafs = []
    Leaf_Damper("damperRoot", rt, 7, d_gain=1)
    Leaf_JoinLimitAvoidance("joinLimitAvoidance", rt, ctrl_range)

    list_linkFrame = []

    for i in range(env.num_joint):
        m = create_mappings(i)
        link_frame = Rmp_linkFrame('linkFrame_' + str(i), rt, m[0], m[1], m[2])
        list_linkFrame.append(link_frame)

    # Collision avoidance
    list_collisionControlPoint = []
    for i in range(1, 7):
        coli_pos_cp = Rmp_posControlPoint('posControlPoint_' + str(i), list_linkFrame[i])
        list_collisionControlPoint.append(coli_pos_cp)

    list_obstaclePoint = []
    list_obstaclePoint.append(env.mujoco_obstacle["cubeA"].pos)
    list_obstaclePoint.append(env.mujoco_obstacle["cubeB"].pos)

    list_avoidLeaf = []
    for i in range(len(list_collisionControlPoint)):
        leafs = []
        # for ii in range(len(list_collisionControlPoint)):
        #     leaf = Leaf_CollisionAvoidance("collisionAvoid_cp" + str(i) + "_point" + str(j),
        #                                    list_collisionControlPoint[i],
        #                                    list_obstaclePoint[j])
        for j in range(len(list_obstaclePoint)):
            leaf = Leaf_CollisionAvoidance("collisionAvoid_cp" + str(i) + "_point" + str(j),
                                           list_collisionControlPoint[i],
                                           list_obstaclePoint[j])
            leafs.append(leaf)
        list_avoidLeaf.append(leafs)

    # Goal attraction
    list_attractionLeaf = []
    pos_goal = env.mujoco_obstacle["cubeA"].pos
    goal_pos_cp = Rmp_posControlPoint('posControlPoint_ee', list_linkFrame[6], [0, 0, 0.1])
    leaf = Leaf_GoalAttractorPosition('goalAttractionPos', goal_pos_cp, pos_goal)
    list_attractionLeaf.append(leaf)
    # Damper("damperControlPoint_Pos", goal_pos_cp, 3, d_gain=5)
    ori_axis = [0, 1, 0]
    ori_angle = 1.0 * np.pi
    ori_goal = getQuat(ori_axis, ori_angle)
    goal_ori_cp = Rmp_oriControlPoint('oriControlPoint', list_linkFrame[6])
    leaf = Leaf_GoalAttractorOritation('goalAttractionOri', goal_ori_cp, ori_goal)
    list_attractionLeaf.append(leaf)
    # Damper("damperControlPoint_Ori", goal_ori_cp, 3, d_gain=5)

    d_ddq = np.zeros_like(init_joint_pos)
    d_dq = init_joint_vel
    d_q = init_joint_pos

    # do visualization
    for i in range(50000):
        di = env.get_obv_for_planning()

        q = di["joint_pos"]
        dq = di["joint_vel"]

        d_ddq = rt.solve(q, dq)
        d_dq = d_dq + d_ddq * env.control_timestep * 10
        d_q = q + d_dq * env.control_timestep

        # action_pos = np.random.randn(env.dof)
        action_pos = np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00])
        # action_pos = np.zeros_like(init_joint_pos)
        action_vel = np.zeros_like(action_pos)
        action = np.concatenate((d_q, action_vel))

        # action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        env.step(action)

        id_name = 'right_l' + str(joint_index)
        current_position = env.sim.data.body_xpos[env.sim.model.body_name2id(id_name)]

        current_quat = env.sim.data.body_xquat[
            env.sim.model.body_name2id(id_name)]  # quaternion (w, x, y, z)
        current_rotmat = env.sim.data.body_xmat[env.sim.model.body_name2id(id_name)].reshape([3, 3])

        current_velp = env.sim.data.body_xvelp[env.sim.model.body_name2id(id_name)]
        current_velr = env.sim.data.body_xvelr[env.sim.model.body_name2id(id_name)]

        Jx = env.sim.data.get_body_jacp(id_name).reshape((3, -1))
        Jx = np.delete(Jx, [1, 8, 9], axis=1)
        Jr = env.sim.data.get_body_jacr(id_name).reshape((3, -1))
        Jr = np.delete(Jr, [1, 8, 9], axis=1)

        err_pos = pos_goal - goal_pos_cp.x
        err_ori = get_orientation_error(ori_goal, goal_ori_cp.x)

        # if norm(err_pos) < 0.05 and norm(err_ori) < 0.05:
        #     ori_angle = np.random.randint(2) * np.pi
        #     ori_goal = getQuat(ori_axis, ori_angle)
        #     list_attractionLeaf[1].update_goal(ori_goal)

        if i % 10 == 0:
            # print()
            print()

            print(err_pos, err_ori)

            cp = list_avoidLeaf[5][0]
            # print(cp.x)
            # print(cp.x_dot)
            # print(cp.M)
            # print(cp.f)

            # print("pos")
            # print(pos_goal)
            # print(goal_pos_cp.x)
            #
            # print("ori")
            # print(ori_goal)
            # print(goal_ori_cp.x)

            # print(q)

            # print(psi(q))
            # print(J(q))
            # print(link_frames[joint_index].J_dot(q, dq))

            # print('Jacobian_pos------------------------------')
            # print(current_velp)
            # print(np.dot(Jx, dq))
            # print(np.dot(J(q)[0], dq))
            # print('Jacobian_ori------------------------------')
            # print(current_velr)
            # quat_dot = np.dot(J(q), dq)[3:7]
            # quat = psi(q)[3:7]
            # angVel = np.dot(mat_quatVel2angVel(quat), quat_dot)
            # print(angVel)

            # print('ForwardKinematics_pos------------------------------')
            # print(current_position)
            # print(link_frames[joint_index].psi(q)[0:3])
            # print(psi(q)[0:3])
            # print('ForwardKinematics_ori------------------------------')
            # print(current_quat)
            # print(psi(q)[3:7])

            print()
            # print()

        env.render()
