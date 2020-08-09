import numpy as np
# import robosuite as suite
from robosuite import Env_SawyerRmp
from robosuite.utils.transform_utils import get_orientation_error, quat2mat, mat2quat

if __name__ == "__main__":

    # initialize the task
    env = Env_SawyerRmp(has_renderer=True,
                        ignore_done=True,
                        use_camera_obs=False,
                        control_freq=100, )
    env.reset()
    env.viewer.set_camera(camera_id=0)

    joint_index = 6

    psi = lambda q: env.f_psi(joint_index, q)
    J = lambda q: env.f_jcb(joint_index, q)
    dJ = lambda q, dq: env.f_jcb_dot(joint_index, q, dq)

    # do visualization
    for i in range(5000):
        action = np.random.randn(env.dof)
        action = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.0]

        obs, reward, done, _ = env.step(action)

        di = env.get_obv_for_planning()

        q = di["joint_pos"]
        dq = di["joint_vel"]

        id_name = 'right_l' + str(joint_index)
        current_position = env.sim.data.body_xpos[env.sim.model.body_name2id(id_name)]

        current_quat = env.sim.data.body_xquat[
            env.sim.model.body_name2id(id_name)]  # quaternion of mujoco is defined differently from others
        current_rotmat = env.sim.data.body_xmat[env.sim.model.body_name2id(id_name)].reshape([3, 3])

        current_velp = env.sim.data.body_xvelp[env.sim.model.body_name2id(id_name)]
        current_velr = env.sim.data.body_xvelr[env.sim.model.body_name2id(id_name)]

        Jx = env.sim.data.get_body_jacp(id_name).reshape((3, -1))
        Jx = np.delete(Jx, [1, 8, 9], axis=1)
        Jr = env.sim.data.get_body_jacr(id_name).reshape((3, -1))
        Jr = np.delete(Jr, [1, 8, 9], axis=1)

        if i % 10 == 0:
            print()
            print()

            # print(Jr)
            # print(J(q)[1])

            # print(current_velp)
            print('Jacobian_pos------------------------------')
            print(current_velp)
            print(np.dot(Jx, dq))
            print(np.dot(J(q)[0], dq))
            print('Jacobian_ori------------------------------')
            print(current_velr)
            print(np.dot(Jr, dq))
            print(np.dot(J(q)[1], dq))

            print('ForwardKinematics_pos------------------------------')
            print(current_position)
            print(psi(q)[0])
            print('ForwardKinematics_ori------------------------------')
            print(mat2quat(current_rotmat))
            print(mat2quat(psi(q)[1]))

            print()
            print()

        env.render()
