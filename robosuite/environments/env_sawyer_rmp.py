from collections import OrderedDict
import numpy as np

from utils.transform_utils import convert_quat, mat2quat, angularVelocity2matS, cross_mat, mat_angVel2quatVel
from robosuite.environments.my_sawyer import mySawyerEnv

from robosuite.models import assets_root
from robosuite.models.arenas.table_arena import TableArena
from robosuite.models.objects import BoxObject
from robosuite.models.tasks import MyTask, UniformRandomSampler

from scipy.misc import derivative
import copy
# from urdf2casadi import converter

from os.path import join as pjoin
import pybullet as p
import time


class Env_SawyerRmp(mySawyerEnv):
    """
    This class corresponds to the stacking task for the Sawyer robot arm.
    """

    def __init__(
            self,
            gripper_type=None,
            table_full_size=(0.8, 0.8, 0.8),
            table_friction=(1., 5e-3, 1e-4),
            use_camera_obs=True,
            use_object_obs=True,
            reward_shaping=False,
            placement_initializer=None,
            gripper_visualization=False,
            use_indicator_object=False,
            has_renderer=False,
            has_offscreen_renderer=True,
            render_collision_mesh=False,
            render_visual_mesh=True,
            control_freq=100,
            horizon=1000,
            ignore_done=False,
            camera_name="frontview",
            camera_height=256,
            camera_width=256,
            camera_depth=False,
    ):

        # settings for table top
        self.table_full_size = table_full_size
        self.table_friction = table_friction

        # whether to show visual aid about where is the gripper
        self.gripper_visualization = gripper_visualization

        # whether to use ground-truth object states
        self.use_object_obs = use_object_obs

        # object placement initializer
        if placement_initializer:
            self.placement_initializer = placement_initializer
        else:
            self.placement_initializer = UniformRandomSampler(
                x_range=[-0.08, 0.08],
                y_range=[-0.08, 0.08],
                ensure_object_boundary_in_range=False,
                z_rotation=True,
            )

        super().__init__(
            gripper_type=gripper_type,
            gripper_visualization=gripper_visualization,
            use_indicator_object=use_indicator_object,
            has_renderer=has_renderer,
            has_offscreen_renderer=has_offscreen_renderer,
            render_collision_mesh=render_collision_mesh,
            render_visual_mesh=render_visual_mesh,
            control_freq=control_freq,
            horizon=horizon,
            ignore_done=ignore_done,
            use_camera_obs=use_camera_obs,
            camera_name=camera_name,
            camera_height=camera_height,
            camera_width=camera_width,
            camera_depth=camera_depth,
        )

        # reward configuration
        self.reward_shaping = reward_shaping

        # information of objects
        # self.object_names = [o['object_name'] for o in self.object_metadata]
        self.object_names = list(self.mujoco_obstacle.keys())
        self.object_site_ids = [
            self.sim.model.site_name2id(ob_name) for ob_name in self.object_names
        ]

        # self.sim.data.contact # list, geom1, geom2
        self.collision_check_geom_names = self.sim.model._geom_name2id.keys()
        self.collision_check_geom_ids = [self.sim.model._geom_name2id[k] for k in self.collision_check_geom_names]

        self.f_jcb = None
        self.f_jcb_dot = None
        self.fk_dict = None
        self.setup_kinematic_tools()

        self.num_joint = 7

    def _load_model(self):
        """
        Loads an xml model, puts it in self.model
        """
        super()._load_model()
        self.mujoco_robot.set_base_xpos([0, 0, 0])

        # load model for table top workspace
        self.mujoco_arena = TableArena(
            table_full_size=self.table_full_size, table_friction=self.table_friction
        )
        if self.use_indicator_object:
            self.mujoco_arena.add_pos_indicator()

        # The sawyer robot has a pedestal, we want to align it with the table
        self.mujoco_arena.set_origin([0.16 + self.table_full_size[0] / 2, 0, 0])

        # initialize objects of interest
        cubeA = BoxObject(
            size=[0.02, 0.02, 0.02],
            pos=[0., -0.6, 1.3],
            rgba=[1, 0, 0, 1]
        )
        cubeB = BoxObject(
            size=[0.025, 0.025, 0.025],
            pos=[1.5, -0.1, 1.2],
            rgba=[0, 1, 0, 1],
        )
        self.mujoco_obstacle = OrderedDict([("cubeA", cubeA), ("cubeB", cubeB)])
        self.n_obstacle = len(self.mujoco_obstacle)

        # task includes arena, robot, and objects of interest
        self.model = MyTask(self.mujoco_arena,
                            self.mujoco_robot,
                            self.mujoco_obstacle,
                            initializer=self.placement_initializer, )
        # self.model.place_objects()

    def setup_kinematic_tools(self):

        self.num_joint = 7

        # Set up a connection to the PyBullet simulator.
        p.connect(p.DIRECT)
        p.resetSimulation()

        # get paths to urdfs
        self.r_urdf = pjoin(assets_root, "bullet_data/sawyer_description/urdf/sawyer_arm.urdf")

        base_pos = (-self.mujoco_robot.bottom_offset).tolist()
        # load the urdfs
        self.bullet_robot = p.loadURDF(self.r_urdf, base_pos, useFixedBase=1)
        # self.bullet_robot = p.loadMJCF('/home/ren/mypy/RobotTaskSim/robosuite/models/assets/robots/sawyer/robot.xml')

        self.fk_dict = []
        for index_link in range(self.num_joint):
            fk_dict = converter.from_file("right_arm_base_link",
                                          "right_l" + str(index_link),
                                          self.r_urdf)
            self.fk_dict.append(fk_dict)

        dof_joint = 7
        def_list = np.zeros(dof_joint).tolist()

        def calForwardKinematics(index_link, q):
            assert len(q) == dof_joint
            qc = np.array(q)[0:(index_link + 1)]
            fk_dict = self.fk_dict[index_link]
            T = fk_dict["T_fk"](qc)
            pos = T[:3, 3] + base_pos
            rot_mat = T[:3, :3]
            # rot_quat = mat2quat(rot_mat)
            rot_quat = fk_dict["quaternion_fk"](qc)  # # quaternion (x, y, z, w)
            rot_quat = convert_quat(rot_quat, "wxyz")
            return np.squeeze(np.vstack((pos, rot_quat)))

        def calJacobian(index_link, q):
            """
            Returns:
                Jacobian matrix: J[0] for pos, J[1] for ori
            """
            assert len(q) == dof_joint
            q = np.squeeze(np.array(q))
            J = p.calculateJacobian(self.bullet_robot,
                                    linkIndex=index_link,
                                    localPosition=[0., 0., 0.],
                                    objPositions=q.tolist(),
                                    objVelocities=def_list,
                                    objAccelerations=def_list)

            quat = calForwardKinematics(index_link, q)[3:7]
            # J_pos = np.array(J[0])
            # J_ang = np.array(J[1])
            J_pos = J[0]
            J_ang = J[1]
            J_quat = np.dot(mat_angVel2quatVel(quat), J_ang)
            return np.vstack((J_pos, J_quat))

        def calJacobian_dot(index_link, q, dq):
            assert len(q) == dof_joint
            assert len(dq) == dof_joint
            if np.linalg.norm(dq) > 1e-9:
                J = lambda q: calJacobian(index_link, q)
                J_dot = derivative(func=J, x0=q, dx=dq)
            else:
                J_dot = np.zeros((7, dof_joint))
            return J_dot

        self.f_psi = calForwardKinematics
        self.f_jcb = calJacobian
        self.f_jcb_dot = calJacobian_dot

        p.setRealTimeSimulation(1)

    def get_obv_for_planning(self):
        di = OrderedDict()

        joint_pos = np.array([self.sim.data.qpos[x] for x in self._ref_joint_pos_indexes])
        joint_vel = np.array([self.sim.data.qvel[x] for x in self._ref_joint_vel_indexes])
        di["joint_pos"] = joint_pos
        di["joint_vel"] = joint_vel

        # print(self._ref_joint_pos_indexes)

        obstacle_pos = []
        for i in self.mujoco_obstacle:
            obstacle_pos.append(self.mujoco_obstacle[i].pos)
        # cubeA_pos = np.array(self.sim.data.body_xpos[self.cubeA_body_id])
        # cubeB_pos = np.array(self.sim.data.site_xpos[self.cubeB_body_id])
        # obstacle_pos.append(cubeA_pos)
        # obstacle_pos.append(cubeB_pos)
        di["obstacle_pos"] = obstacle_pos

        return di

    def step(self, action):
        """Takes a step in simulation with control command @action."""
        if self.done:
            raise ValueError("executing action in terminated episode")

        self.timestep += 1
        self._pre_action(action)
        end_time = self.cur_time + self.control_timestep
        while self.cur_time < end_time:
            self.sim.step()
            self.cur_time += self.model_timestep
        return


    def _pre_action(self, action):
        """
        Overrides the superclass method to actuate the robot with the
        passed joint velocities and gripper control.

        Args:
            action (numpy array): The control to apply to the robot. The first
                @self.mujoco_robot.dof dimensions should be the desired
                normalized joint velocities and if the robot has
                a gripper, the next @self.gripper.dof dimensions should be
                actuation controls for the gripper.
        """

        # clip actions into valid range
        applied_action = action

        self.sim.data.ctrl[:] = applied_action

        # gravity compensation
        self.sim.data.qfrc_applied[self._ref_joint_vel_indexes] = self.sim.data.qfrc_bias[self._ref_joint_vel_indexes]

        if self.use_indicator_object:
            self.sim.data.qfrc_applied[
            self._ref_indicator_vel_low: self._ref_indicator_vel_high] = self.sim.data.qfrc_bias[
                                                                         self._ref_indicator_vel_low: self._ref_indicator_vel_high]

    def _post_action(self, action):
        """
        (Optional) does gripper visualization after actions.
        """
        ret = super()._post_action(action)
        self._gripper_visualization()
        return ret

    def _get_reference(self):
        """
        Sets up references to important components. A reference is typically an
        index or a list of indices that point to the corresponding elements
        in a flatten array, which is how MuJoCo stores physical simulation data.
        """
        super()._get_reference()
        self.cubeA_body_id = self.sim.model.body_name2id("cubeA_site")
        self.cubeB_body_id = self.sim.model.body_name2id("cubeB_site")

        self.cubeA_geom_id = self.sim.model.geom_name2id("cubeA_site")
        self.cubeB_geom_id = self.sim.model.geom_name2id("cubeB_site")

    def _reset_internal(self):
        """
        Resets simulation internal configurations.
        """
        super()._reset_internal()

        # reset positions of objects
        # self.model.place_objects()

        # reset joint positions
        init_pos = np.array([-0.5538, -0.8208, 0.4155, 1.8409, -0.4955, 0.6482, 1.9628])
        init_pos += np.random.randn(init_pos.shape[0]) * 0.02
        self.sim.data.qpos[self._ref_joint_pos_indexes] = np.array(init_pos)


    def _check_contact(self):
        """
        Returns True if gripper is in contact with an object.
        """
        collision = False
        for contact in self.sim.data.contact[: self.sim.data.ncon]:
            if (
                    self.sim.model.geom_id2name(contact.geom1) in self.finger_names
                    or self.sim.model.geom_id2name(contact.geom2) in self.finger_names
            ):
                collision = True
                break
        return collision
