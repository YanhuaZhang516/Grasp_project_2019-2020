import numpy as np
# import robosuite as suite
from robosuite import Env_SawyerRmp
from utils.transform_utils import mat_quatVel2angVel, getQuat, parseQuat, quat2mat

from rmp.rmp_base import *
from rmp.rmp_leaf import *
from robosuite.global_setting import *
import imageio   # install ffmpeg at first


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


def create_leaf_inner_avoid(list_tuple):
    list_leaf = []
    for i in range(len(list_tuple)):
        pos2frame = Rmp_pos2frame("pos2frame" + str(i), rt, list_tuple[i], env.f_fk)
        leaf = Leaf_CollisionAvoidance("collisionAvoid_inner" + str(i), pos2frame, [0, 0, 0])
        list_leaf.append(leaf)
    return list_leaf


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
        # link_frame = Rmp_linkFrame('linkFrame_' + str(i), rt, m[0], m[1], m[2])
        link_frame = Rmp_linkFrame('linkFrame_' + str(i), rt, i, env.f_fk)
        list_linkFrame.append(link_frame)

    # Collision avoidance
    list_collisionControlPoint = []
    for i in range(7):
        pos_cp = Rmp_posControlPoint('posControlPoint_' + str(i), list_linkFrame[i])
        list_collisionControlPoint.append(pos_cp)

    list_obstaclePoint = []
    list_obstaclePoint.append(env.mujoco_obstacle["cubeA"].pos)
    list_obstaclePoint.append(env.mujoco_obstacle["cubeB"].pos)

    list_leaf_avoid = []
    for i in range(1, 7):
        leafs = []
        for j in range(len(list_obstaclePoint)):
            leaf = Leaf_CollisionAvoidance("collisionAvoid_cp" + str(i) + "_point" + str(j),
                                           list_collisionControlPoint[i],
                                           list_obstaclePoint[j])
            leafs.append(leaf)
        list_leaf_avoid.append(leafs)

    # Inner collision avoidance
    list_leaf_avoid_inner = create_leaf_inner_avoid([[6, 0],
                                                     [6, 1],
                                                     [6, 2],
                                                     [6, 3]])

    # Goal attraction
    list_leaf_attract = []
    pos_goal = env.mujoco_obstacle["cubeA"].pos
    goal_pos_cp = Rmp_posControlPoint('posControlPoint_ee', list_linkFrame[6], [0, 0, 0.1])
    leaf = Leaf_GoalAttractorPosition('goalAttractionPos', goal_pos_cp, pos_goal)
    list_leaf_attract.append(leaf)
    # Damper("damperControlPoint_Pos", goal_pos_cp, 3, d_gain=5)
    ori_axis = [0, 1, 0]
    ori_angle = 0.5 * np.pi
    ori_goal = getQuat(ori_axis, ori_angle)
    goal_ori_cp = Rmp_oriControlPoint('oriControlPoint', list_linkFrame[6])
    leaf = Leaf_GoalAttractorOritation('goalAttractionOri', goal_ori_cp, ori_goal)
    list_leaf_attract.append(leaf)
    # Damper("damperControlPoint_Ori", goal_ori_cp, 3, d_gain=5)

    d_ddq = np.zeros_like(init_joint_pos)
    d_dq = init_joint_vel
    d_q = init_joint_pos

    # do visualization
    # create a video writer with imageio
    # video_path = "try.mp4"
    # video_writer = imageio.get_writer(video_path, fps=30)
    # frames = []
    for i in range(5000):

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

        err_pos = pos_goal - goal_pos_cp.x
        err_ori = get_orientation_error(ori_goal, goal_ori_cp.x)

        if norm(err_pos) < 0.05 and norm(err_ori) < 0.05:
            ori_angle = np.random.randint(2) * np.pi
            ori_goal = getQuat(ori_axis, ori_angle)
            list_leaf_attract[1].update_goal(ori_goal)

        # if i % 1 == 0:
        #     frame = di["image"]
        #     video_writer.append_data(frame)

        if i % 10 == 0:
            # print()
            print()

            print(list_leaf_avoid_inner[0].x)
            # print(err_pos, err_ori)
            # obs_pos = list_leaf_avoid_inner[0].ob_pos
            # print(obs_pos)

            # print(list_linkFrame[joint_index].psi(q))

            cp = list_leaf_avoid[5][0]
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

            print()
            # print()

        env.render()

    # video_writer.close()