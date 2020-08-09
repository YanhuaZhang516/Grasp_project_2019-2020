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


class GoalAttractorPosition(RMPLeaf):
    """
    Goal Attractor RMP leaf
    """

    def __init__(self, name, parent, x_goal, w_u=10, w_l=1, sigma=1):
        assert isinstance(parent, RMP_posControlPoint)

        psi = lambda p_x: (p_x - x_goal)
        J = lambda p_x: np.eye(3)
        J_dot = lambda p_x, p_x_dot: np.zeros((3, 3))

        def RMP_func(x, x_dot):
            x_norm = norm(x)

            beta = np.exp(- x_norm ** 2 / 2 / (sigma ** 2))
            w = (w_u - w_l) * beta + w_l
            M = np.eye(3) * w

            p_gain = 10
            d_gain = 8

            d_a = -p_gain * soft_normalization(x) - d_gain * x_dot

            f = np.dot(M, d_a)

            return (f, M)

        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)

    def update_goal(self, x_goal):
        """
        update the position of the goal
        """
        self.psi = lambda p_x: (p_x - x_goal)
        self.J = lambda p_x: np.eye(3)
        self.J_dot = lambda p_x, p_x_dot: np.zeros((3, 3))


class GoalAttractorOritation(RMPLeaf):

    def __init__(self, name, parent, x_goal, w_u=5, w_l=1, sigma=1):
        # if y_g.ndim == 1:
        #     y_g = y_g.reshape(-1, 1)
        assert isinstance(parent, RMP_oriControlPoint)

        psi = lambda p_x: -get_orientation_error(x_goal, p_x)
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
        self.psi = lambda p_x: -get_orientation_error(x_goal, p_x)
        self.J = lambda p_x: mat_quatVel2angVel(p_x)
        self.J_dot = lambda p_x, p_x_dot: derivative(func=self.J, x0=p_x, dx=p_x_dot)


class JoinLimitAvoidance(RMPLeaf):

    def __init__(self, name, parent, ctrl_range, gamma=1):
        assert isinstance(parent, RMP_Root)

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

            p_gain = 1
            d_gain = 1

            d_a = -p_gain * x - d_gain * x_dot

            M = A
            f = np.dot(M, d_a)

            return (f, M)

        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)


class CollisionAvoidance(RMPLeaf):
    """
    Obstacle avoidance RMP leaf
    """

    def __init__(self, name, parent, parent_param, c=np.zeros(2), R=1, epsilon=0.2,
                 alpha=1e-5, eta=0):

        self.R = R
        self.alpha = alpha
        self.eta = eta
        self.epsilon = epsilon

        if parent_param:
            psi = None
            J = None
            J_dot = None

        else:
            if c.ndim == 1:
                c = c.reshape(-1, 1)

            N = c.size

            psi = lambda p_x: np.array(norm(p_x - c) / R - 1).reshape(-1, 1)  # y = psi(x), forward mapping
            J = lambda p_x: 1.0 / norm(p_x - c) * (p_x - c).T / R  # y_dot = J(x_dot), Jacobi
            J_dot = lambda p_x, p_dx: np.dot(p_dx.T,
                                             (-1 / norm(p_x - c) ** 3 * np.dot((p_x - c), (p_x - c).T)
                                              + 1 / norm(p_x - c) * np.eye(N))) / R

        def RMP_func(x, x_dot):
            if x < 0:
                w = 1e10
                grad_w = 0
            else:
                w = 1.0 / x ** 4
                grad_w = -4.0 / x ** 5
            u = epsilon + np.minimum(0, x_dot) * x_dot
            g = w * u

            grad_u = 2 * np.minimum(0, x_dot)
            grad_Phi = alpha * w * grad_w
            xi = 0.5 * x_dot ** 2 * u * grad_w

            M = g + 0.5 * x_dot * w * grad_u
            M = np.minimum(np.maximum(M, - 1e5), 1e5)

            Bx_dot = eta * g * x_dot

            f = - grad_Phi - xi - Bx_dot
            f = np.minimum(np.maximum(f, - 1e10), 1e10)

            return (f, M)

        RMPLeaf.__init__(self, name, parent, parent_param, psi, J, J_dot, RMP_func)


class CollisionAvoidanceDecentralized(RMPLeaf):
    """
    Decentralized collision avoidance RMP leaf for the RMPForest
    """

    def __init__(self, name, parent, parent_param, c=np.zeros(2), R=1, epsilon=1e-8,
                 alpha=1e-5, eta=0):

        assert parent_param is not None

        self.R = R
        self.alpha = alpha
        self.eta = eta
        self.epsilon = epsilon
        self.x_dot_real = None

        psi = None
        J = None
        J_dot = None

        def RMP_func(x, x_dot, x_dot_real):
            if x < 0:
                w = 1e10
                grad_w = 0
            else:
                w = 1.0 / x ** 4
                grad_w = -4.0 / x ** 5
            u = epsilon + np.minimum(0, x_dot) * x_dot
            g = w * u

            grad_u = 2 * np.minimum(0, x_dot)
            grad_Phi = alpha * w * grad_w
            xi = 0.5 * x_dot * x_dot_real * u * grad_w

            M = g + 0.5 * x_dot * w * grad_u
            M = np.minimum(np.maximum(M, - 1e5), 1e5)

            Bx_dot = eta * g * x_dot

            f = - grad_Phi - xi - Bx_dot
            f = np.minimum(np.maximum(f, - 1e10), 1e10)

            return (f, M)

        RMPLeaf.__init__(self, name, parent, parent_param, psi, J, J_dot, RMP_func)

    def pushforward(self):
        """
        override pushforward() to update the curvature term
        """
        if self.verbose:
            print('%s: pushforward' % self.name)

        if self.psi is not None and self.J is not None:
            self.x = self.psi(self.parent.x)
            self.x_dot = np.dot(self.J(self.parent.x), self.parent.x_dot)
            self.x_dot_real = np.dot(
                self.J(self.parent.x),
                self.parent.x_dot - self.parent_param.x_dot)

    def eval_leaf(self):
        """
        override eval_leaf() to update the curvature term
        """
        self.f, self.M = self.RMP_func(self.x, self.x_dot, self.x_dot_real)

    def update_params(self):
        """
        update the position of the other robot
        """
        c = self.parent_param.x
        z_dot = self.parent_param.x_dot
        R = self.R
        if c.ndim == 1:
            c = c.reshape(-1, 1)

        N = c.size

        self.psi = lambda p_x: np.array(norm(p_x - c) / R - 1).reshape(-1, 1)
        self.J = lambda p_x: 1.0 / norm(p_x - c) * (p_x - c).T / R
        self.J_dot = lambda p_x, p_dx: np.dot(
            p_dx.T,
            (-1 / norm(p_x - c) ** 3 * np.dot((p_x - c), (p_x - c).T)
             + 1 / norm(p_x - c) * np.eye(N))) / R


class CollisionAvoidanceCentralized(RMPLeaf):
    """
    Centralized collision avoidance RMP leaf for a pair of robots
    """

    def __init__(self, name, parent, R=1, epsilon=1e-8,
                 alpha=1e-5, eta=0):

        self.R = R

        def psi(y):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            return np.array(norm(y1 - y2) / R - 1).reshape(-1, 1)

        def J(y):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            return np.concatenate((
                1.0 / norm(y1 - y2) * (y1 - y2).T / R,
                -1.0 / norm(y1 - y2) * (y1 - y2).T / R),
                axis=1)

        def J_dot(y, y_dot):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            y1_dot = y_dot[: N]
            y2_dot = y_dot[N:]
            return np.concatenate((
                np.dot(
                    y1_dot.T,
                    (-1 / norm(y1 - y2) ** 3 * np.dot((y1 - y2), (y1 - y2).T)
                     + 1 / norm(y1 - y2) * np.eye(N))) / R,
                np.dot(
                    y2_dot.T,
                    (-1 / norm(y1 - y2) ** 3 * np.dot((y1 - y2), (y1 - y2).T)
                     + 1 / norm(y1 - y2) * np.eye(N))) / R),
                axis=1)

        def RMP_func(x, x_dot):
            if x < 0:
                w = 1e10
                grad_w = 0
            else:
                w = 1.0 / x ** 4
                grad_w = -4.0 / x ** 5
            u = epsilon + np.minimum(0, x_dot) * x_dot
            g = w * u

            grad_u = 2 * np.minimum(0, x_dot)
            grad_Phi = alpha * w * grad_w
            xi = 0.5 * x_dot ** 2 * u * grad_w

            M = g + 0.5 * x_dot * w * grad_u
            M = np.minimum(np.maximum(M, - 1e5), 1e5)

            Bx_dot = eta * g * x_dot

            f = - grad_Phi - xi - Bx_dot
            f = np.minimum(np.maximum(f, - 1e10), 1e10)

            return (f, M)

        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)


class GoalAttractorUni(RMPLeaf):
    """
    Goal Attractor RMP leaf
    """

    def __init__(self, name, parent, y_g, w_u=10, w_l=1, sigma=1,
                 alpha=1, eta=2, gain=1, tol=0.005):

        # if y_g.ndim == 1:
        #     y_g = y_g.reshape(-1, 1)
        N = y_g.size
        psi = lambda y: (y - y_g)
        J = lambda y: np.eye(N)
        J_dot = lambda y, y_dot: np.zeros((N, N))

        def RMP_func(x, x_dot):
            x_norm = norm(x)

            beta = np.exp(- x_norm ** 2 / 2 / (sigma ** 2))
            w = (w_u - w_l) * beta + w_l
            s = (1 - np.exp(-2 * alpha * x_norm)) / (1 + np.exp(
                -2 * alpha * x_norm))

            G = np.eye(N) * w
            if x_norm > tol:
                grad_Phi = s / x_norm * w * x * gain
            else:
                grad_Phi = 0
            Bx_dot = eta * w * x_dot
            grad_w = - beta * (w_u - w_l) / sigma ** 2 * x

            x_dot_norm = norm(x_dot)
            xi = -0.5 * (x_dot_norm ** 2 * grad_w - 2 *
                         np.dot(np.dot(x_dot, x_dot.T), grad_w))

            M = G
            f = - grad_Phi - Bx_dot - xi

            return (f, M)

        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)

    def update_goal(self, y_g):
        """
        update the position of the goal
        """

        if y_g.ndim == 1:
            y_g = y_g.reshape(-1, 1)
        N = y_g.size
        self.psi = lambda y: (y - y_g)
        self.J = lambda y: np.eye(N)
        self.J_dot = lambda y, y_dot: np.zeros((N, N))


class FormationDecentralized(RMPLeaf):
    """
    Decentralized formation control RMP leaf for the RMPForest
    """

    def __init__(self, name, parent, parent_param, c=np.zeros(2), d=1, gain=1, eta=2, w=1):
        assert parent_param is not None
        self.d = d

        psi = None
        J = None
        J_dot = None

        def RMP_func(x, x_dot):
            G = w
            grad_Phi = gain * x * w
            Bx_dot = eta * w * x_dot
            M = G
            f = - grad_Phi - Bx_dot

            return (f, M)

        RMPLeaf.__init__(self, name, parent, parent_param, psi, J, J_dot, RMP_func)

    def update_params(self):
        """
        update the position of the other robot
        """

        z = self.parent_param.x
        z_dot = self.parent_param.x_dot

        c = z
        d = self.d
        if c.ndim == 1:
            c = c.reshape(-1, 1)

        N = c.size
        self.psi = lambda y: np.array(norm(y - c) - d).reshape(-1, 1)
        self.J = lambda y: 1.0 / norm(y - c) * (y - c).T
        self.J_dot = lambda y, y_dot: np.dot(
            y_dot.T,
            (-1 / norm(y - c) ** 3 * np.dot((y - c), (y - c).T)
             + 1 / norm(y - c) * np.eye(N)))


class FormationCentralized(RMPLeaf):
    """
    Centralized formation control RMP leaf for a pair of robots
    """

    def __init__(self, name, parent, d=1, gain=1, eta=2, w=1):
        def psi(y):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            return np.array(norm(y1 - y2) - d).reshape(-1, 1)

        def J(y):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            return np.concatenate((
                1.0 / norm(y1 - y2) * (y1 - y2).T,
                -1.0 / norm(y1 - y2) * (y1 - y2).T),
                axis=1)

        def J_dot(y, y_dot):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            y1_dot = y_dot[: N]
            y2_dot = y_dot[N:]
            return np.concatenate((
                np.dot(
                    y1_dot.T,
                    (-1 / norm(y1 - y2) ** 3 * np.dot((y1 - y2), (y1 - y2).T)
                     + 1 / norm(y1 - y2) * np.eye(N))),
                np.dot(
                    y2_dot.T,
                    (-1 / norm(y1 - y2) ** 3 * np.dot((y1 - y2), (y1 - y2).T)
                     + 1 / norm(y1 - y2) * np.eye(N)))),
                axis=1)

        def RMP_func(x, x_dot):
            G = w
            grad_Phi = gain * x * w
            Bx_dot = eta * w * x_dot
            M = G
            f = - grad_Phi - Bx_dot

            return (f, M)

        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)


class Damper(RMPLeaf):
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
