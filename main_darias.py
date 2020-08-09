import numpy as np
import robosuite as suite
from robosuite.wrappers import IKWrapper

if __name__ == "__main__":

    # initialize the task
    env = suite.make(
        env_name='SawyerStack2',
        has_renderer=True,
        ignore_done=True,
        use_camera_obs=False,
        control_freq=100,
    )
    env = IKWrapper(env)   # Task space control

    env.reset()
    env.viewer.set_camera(camera_id=0)

    # do visualization
    for i in range(5000):
        action = np.random.randn(env.dof)
        # action = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0]
        # print(action)
        obs, reward, done, _ = env.step(action)
        env.render()
