# RMPflow basic classes
# @author Anqi Li
# @date April 8, 2019

import numpy as np
from utils.transform_utils import *


class RMP_Node:
    """
    A Generic RMP node
    """

    def __init__(self, name, parent, psi, J, J_dot, verbose=False):
        self.name = name

        self.parent = parent
        self.children = []

        # connect the node to its parent
        if self.parent:
            self.parent.add_child(self)

        # forward mapping & J & J_dot for the edge from the parent to the node
        self.psi = psi
        self.J = J
        self.J_dot = J_dot

        # state
        self.x = None  # x = array([  ,  ,  ])
        self.x_dot = None  # x_dot = array([  ,  ,  ])

        # RMP
        self.f = None
        self.a = None
        self.M = None

        # print the name of the node when applying operations if true
        self.verbose = verbose

    def add_child(self, child):
        """
        Add a child to the current node
        """

        self.children.append(child)

    def pushforward(self):
        """
        apply pushforward operation recursively
        """

        if self.verbose:
            print('%s: pushforward' % self.name)

        if self.psi is not None and self.J is not None:
            self.x = self.psi(self.parent.x)
            self.x_dot = np.squeeze(np.dot(self.J(self.parent.x), self.parent.x_dot))

            # assert self.x.ndim == 2 and self.x_dot.ndim == 2

        [child.pushforward() for child in self.children]

    def pullback(self):
        """
        apply pullback operation recursively
        """

        [child.pullback() for child in self.children]

        if self.verbose:
            print('%s: pullback' % self.name)

        f = np.zeros_like(self.x)
        M = np.zeros((max(self.x.shape), max(self.x.shape)))

        for child in self.children:

            J_child = child.J(self.x)
            J_dot_child = child.J_dot(self.x, self.x_dot)

            if child.f is not None and child.M is not None:
                M_dJ_dx = dot3(child.M, J_dot_child, self.x_dot)
                f += np.squeeze(dot2(J_child.T, child.f - M_dJ_dx))
                M += dot3(J_child.T, child.M, J_child)  # Sandwich product

        self.f = f
        self.M = M


class RMP_linkFrame(RMP_Node):
    """
        parent = RMP_Root
        x = [x, y, z, qx, qy, qz, qw]
    """

    def __init__(self, name, parent, psi, J, J_dot, verbose=False):
        assert isinstance(parent, RMP_Root)

        RMP_Node.__init__(self, name, parent, psi, J, J_dot, verbose)


class RMP_posControlPoint(RMP_Node):
    """
        parent = RMP_robotLinkFrame
        x = [x, y, z]
    """

    def __init__(self, name, parent, offset=None, verbose=False):
        if offset is None:
            offset = [0., 0., 0.]
        assert isinstance(parent, RMP_linkFrame)

        self.offset = np.array(offset)

        def psi(p_x):
            p_pos = p_x[0:3]
            p_quat = p_x[3:7]
            p_rotmat = quat2mat(p_quat)
            return p_pos + np.dot(p_rotmat, self.offset)

        def J(p_x):
            return np.hstack((np.eye(3), np.zeros((3, 4))))

        def J_dot(p_x, p_dx):
            return np.zeros((3, 7))

        RMP_Node.__init__(self, name, parent, psi, J, J_dot, verbose)


class RMP_oriControlPoint(RMP_Node):
    """
        parent = RMP_robotLinkFrame
        x = [x, y, z]
    """

    def __init__(self, name, parent, verbose=False):
        assert isinstance(parent, RMP_linkFrame)

        def psi(p_x):
            p_quat = p_x[3:7]
            return p_quat

        def J(p_x):
            return np.hstack((np.zeros((4, 3)), np.eye(4)))

        def J_dot(p_x, p_dx):
            return np.zeros((4, 7))

        RMP_Node.__init__(self, name, parent, psi, J, J_dot, verbose)


class RMP_Root(RMP_Node):
    """
    A root node
    """

    def __init__(self, name):
        RMP_Node.__init__(self, name, None, None, None, None)

    def set_root_state(self, x, x_dot):
        """
        set the state of the root node for pushforward
        """

        # assert x.ndim == 1 or x.ndim == 2
        # assert x_dot.ndim == 1 or x_dot.ndim == 2
        #
        # if x.ndim == 1:
        #     x = x.reshape(-1, 1)
        # if x_dot.ndim == 1:
        #     x_dot = x_dot.reshape(-1, 1)

        self.x = x
        self.x_dot = x_dot

    def pushforward(self):
        """
        apply pushforward operation recursively
        """

        if self.verbose:
            print('%s: pushforward' % self.name)

        [child.pushforward() for child in self.children]

    def _resolve(self):
        """
        compute the canonical-formed RMP
        """

        if self.verbose:
            print('%s: resolve' % self.name)

        self.a = np.dot(np.linalg.pinv(self.M), self.f)
        return self.a

    def solve(self, x, x_dot):
        """
        given the state of the root, solve for the controls
        """

        self.set_root_state(x, x_dot)
        self.pushforward()
        self.pullback()
        return self._resolve()


class RMPLeaf(RMP_Node):
    """
    A leaf node
    """

    def __init__(self, name, parent, parent_param, psi, J, J_dot, RMP_func):
        RMP_Node.__init__(self, name, parent, psi, J, J_dot)
        self.RMP_func = RMP_func
        self.parent_param = parent_param

    def eval_leaf(self):
        """
        compute the natural-formed RMP given the state
        """
        self.f, self.M = self.RMP_func(self.x, self.x_dot)

    def pullback(self):
        """
        pullback at leaf node is just evaluating the RMP
        """

        if self.verbose:
            print('%s: pullback' % self.name)

        self.eval_leaf()

    def add_child(self, child):
        print("CANNOT add a child to a leaf node")
        pass

    def update_params(self):
        """
        to be implemented for updating the parameters
        """
        pass

    def update(self):
        self.update_params()
        self.pushforward()
