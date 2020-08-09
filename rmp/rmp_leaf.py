# Leaf node RMP classes
# @author Anqi Li
# @date April 8, 2019

from rmp import *
import numpy as np
from numpy.linalg import norm
from utils.transform_utils import *
from scipy.misc import derivative


def soft_normalization(x, alpha=1):
    norm_x = norm(x)
    h = norm_x + np.log(1 + np.exp(-2 * alpha * norm_x))
    return (1 / h) * x


class Leaf_GoalAttractorPosition(RMPLeaf):
    """
    Goal Attractor RMP leaf
    """

    def __init__(self, name, parent, x_goal, w_u=10, w_l=1, sigma=1):
        assert isinstance(parent, Rmp_posControlPoint)

        self.x_goal = x_goal

        psi = lambda p_x: (p_x - self.x_goal)
        J = lambda p_x: np.eye(3)
        J_dot = lambda p_x, p_x_dot: np.zeros((3, 3))

        def RMP_func(x, x_dot):
            x_norm = norm(x)

            beta = np.exp(- x_norm ** 2 / 2 / (sigma ** 2))
            w = (w_u - w_l) * beta + w_l
            M = np.eye(3) * w

            p_gain = 10
            d_gain = 20

            d_a = -p_gain * soft_normalization(x) - d_gain * x_dot

            f = np.dot(M, d_a)

            return (f, M)

        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)

    def update_goal(self, x_goal):
        """
        update the position of the goal
        """
        self.x_goal = x_goal


class Leaf_GoalAttractorOritation(RMPLeaf):

    def __init__(self, name, parent, x_goal, w_u=5, w_l=1, sigma=1):
        # if y_g.ndim == 1:
        #     y_g = y_g.reshape(-1, 1)
        assert isinstance(parent, Rmp_oriControlPoint)

        self.x_goal = x_goal

        psi = lambda p_x: -get_orientation_error(self.x_goal, p_x)
        J = lambda p_x: mat_quatVel2angVel(p_x)

        def J_dot(p_x, p_x_dot):
            if np.linalg.norm(p_x_dot) > 1e-9:
                J_dot = derivative(func=J, x0=p_x, dx=p_x_dot)
            else:
                J_dot = np.zeros((3, 4))
            return J_dot

        def RMP_func(x, x_dot):
            x_norm = norm(x)

            beta = np.exp(- x_norm ** 2 / 2 / (sigma ** 2))
            w = (w_u - w_l) * beta + w_l
            M = np.eye(3) * w

            p_gain = 10
            d_gain = 10

            d_a = -p_gain * soft_normalization(x) - d_gain * x_dot

            f = np.dot(M, d_a)

            return (f, M)

        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)

    def update_goal(self, x_goal):
        """
        update the position of the goal
        """
        self.x_goal = x_goal


class Leaf_JoinLimitAvoidance(RMPLeaf):

    def __init__(self, name, parent, ctrl_range, gamma=1):
        assert isinstance(parent, Rmp_Root)

        dim = len(ctrl_range)

        list_l = ctrl_range[:, 0]
        list_u = ctrl_range[:, 1]
        list_m = 0.5 * (list_l + list_u)
        list_range = list_u - list_l

        psi = lambda p_x: (p_x - list_m)
        J = lambda p_x: np.eye(dim)
        J_dot = lambda p_x, p_x_dot: np.zeros((dim, dim))

        def RMP_func(x, x_dot):
            list_a = []
            for i in range(dim):
                q = parent.x[i]
                dq = parent.x_dot[i]
                s = (q - list_l[i]) / list_range[i]
                d = 4 * s * (1 - s)
                alpha_u = 1 - np.exp(-np.max((0, dq)) ** 2 / (2 * gamma ** 2))
                alpha_l = 1 - np.exp(-np.min((0, dq)) ** 2 / (2 * gamma ** 2))
                b = s * (alpha_u * d + (1 - alpha_u)) + (1 - s) * (alpha_l * d + (1 - alpha_l))
                a = b ** (-2)
                list_a.append(a)
            A = np.diag(list_a)

            p_gain = 0.2
            d_gain = 0.2

            d_a = -p_gain * x - d_gain * x_dot

            M = A
            f = np.dot(M, d_a)

            return (f, M)

        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)


class Leaf_CollisionAvoidance(RMPLeaf):
    """
    Obstacle avoidance RMP leaf
    """

    def __init__(self, name, parent, ob_pos, rw=0.3, sigma=1):
        """
            rw (float): Alert distance. Only within this distance collision avoidance will be activated.
        """
        self.ob_pos = ob_pos

        if self.ob_pos is None:
            psi = None
            J = None
            J_dot = None
            RMP_func = None
        else:
            psi = lambda p_x: norm(p_x - self.ob_pos)
            J = lambda p_x: (p_x - self.ob_pos) / norm(p_x - self.ob_pos)
            J_dot = lambda p_x, p_dx: p_dx / norm(p_x - self.ob_pos) - np.dot((p_x - self.ob_pos), p_dx) * (
                    p_x - self.ob_pos) / norm(p_x - self.ob_pos) ** 3

            def RMP_func(x, x_dot):
                x = np.max((x, 1e-6))
                w = np.max((rw - x), 0) ** 2 / x
                dw_x = 0
                if x < rw:
                    dw_x = 1 - rw ** 2 / x ** 2
                u = 0
                du_x_dot = 0
                if x_dot < 0:
                    expxx = np.exp(-x_dot ** 2 / (2 * sigma ** 2))
                    u = 1 - expxx
                    du_x_dot = expxx * x_dot / sigma ** 2

                delta = u + 0.5 * x_dot * du_x_dot

                M = w * delta

                xi = 0.5 * u * dw_x * x_dot ** 2

                p_gain = 500
                d_gain = 500
                # f = -w * (-gain / x ** 2) - xi

                d_a = p_gain * (1 / x ** 2) + d_gain * x_dot ** 2
                f = np.dot(M, d_a)

                return (f, M)

        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)

    def update_obstacle(self, ob_pos):
        """
        update the position of the goal
        """
        self.ob_pos = ob_pos


class Leaf_CollisionAvoidance_inner(Leaf_CollisionAvoidance):
    """
    Obstacle avoidance RMP leaf
    """

    def __init__(self, name, parent, ob_parent, rw=0.3, sigma=1):
        assert isinstance(parent, Rmp_posControlPoint) and isinstance(ob_parent, Rmp_posControlPoint)

        ob_pos = ob_parent.x

        Leaf_CollisionAvoidance.__init__(self, name, parent, ob_pos)


class Leaf_Damper(RMPLeaf):
    """
    Damper RMP leaf
    """

    def __init__(self, name, parent, x_dim, w=1, d_gain=5):
        psi = lambda p_x: p_x
        J = lambda p_x: np.eye(x_dim)
        J_dot = lambda p_x, p_x_dot: np.zeros((x_dim, x_dim))

        def RMP_func(x, x_dot):
            M = np.eye(x_dim) * w
            d_a = -d_gain * x_dot
            f = np.dot(M, d_a)

            return (f, M)

        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)
