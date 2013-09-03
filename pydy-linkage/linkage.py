from __future__ import print_function, division

import abc

from sympy import symbols
from sympy.physics.mechanics import functions
from sympy.physics.mechanics import dynamicsymbols
from sympy.physics.mechanics import Point, ReferenceFrame, RigidBody

__all__ = ['Linkage', 'RevoluteJoint', 'Identity', 'TranslateX']

class Linkage(object):
    """TODO
    """

    def __init__(self, name):
        self._name = name
        self._root = RootLink()
        self._constants = dict()
        self._constant_descs = dict()

    def _get_name(self):
        return self._name
    def _set_name(self, name):
        self._name = name
    name = property(_get_name, _set_name)

    def _get_root(self):
        return self._root
    root = property(_get_root)

    def constant_new(self, name, description):
        self._constants[name] = symbols(name)
        self._constant_descs[name] = description

    def mass_matrix(self):
        self._kanes_method = KanesMethod() # TODO
        self._kanes_method.kanes_equations(self._force_list, self._body_list)
        return self._kanes_method.mass_matrix

    def _check_link_name(self):
        # TODO
        pass


class RootLink(object):
    """TODO
    """
    def __init__(self, name='root'):
        self._name = name
        self._children = dict()
        # TODO rename to avoid name conflicts, or inform the user that 'N' is
        # taken.
        self._frame = ReferenceFrame('N')
        self._origin = Point('NO')
        self._origin.set_vel(self._frame, 0)
        # TODO need to set_acc?

    def _get_name(self):
        return self._name
    name = property(_get_name)

    def _get_frame(self):
        return self._frame
    frame = property(_get_frame)

    def _get_origin(self):
        return self._origin
    origin = property(_get_origin)

    def link_new(self, name, joint):
        # TODO walk the tree to ensure a link with this name does not exist.
        self._children[name] = Link(name, joint, self)
        print('DEBUG %s' % name)
        print('DEBUG %s' % joint._transform)
        return self._children[name]


class Link(RootLink):
    """TODO
    """

    def __init__(self, name, joint, parent):
        """TODO
        """
        super(Link, self).__init__(name)
        self._joint = joint
        self._parent = parent

        # Create rigid body.
        mass = symbols(name + '_mass')
        frame = ReferenceFrame(name + '_frame')
        self._origin = Point(name + '_origin')
        joint._orient(name, parent.frame, parent.origin, frame, self._origin)

        # All expressed in body frame.
        masscenter = self._origin.locatenew(name + '_masscenter',
                symbols(name + '_mcx') * frame.x +
                symbols(name + '_mcy') * frame.y +
                symbols(name + '_mcz') * frame.z)
        inertia = functions.inertia(frame,
                symbols(name + '_ixx'),
                symbols(name + '_iyy'),
                symbols(name + '_izz'),
                symbols(name + '_ixy'),
                symbols(name + '_iyz'),
                symbols(name + '_izx'))
        # TODO allow specification of non-central inertia
        self._rigidbody = RigidBody(name + '_rigidbody',
                masscenter, frame, mass, (inertia, masscenter))

    def _get_joint(self):
        return self._joint
    joint = property(_get_joint)

    def _get_frame(self):
        return self._rigidbody.frame

    def _get_parent(self):
        return self._parent
    parent = property(_get_parent)


class StaticHomogeneousTransform(object):
    """TODO
    """

    __metaclass__ = abc.ABCMeta

    def _orient(self, parent_frame, parent_origin, frame, origin):
        self._orient_derived(parent_frame, parent_origin, frame, origin)
        frame.set_ang_vel(parent_frame, 0)
        frame.set_ang_acc(parent_frame, 0)
        origin.set_vel(frame, 0)
        origin.set_acc(frame, 0)

    @abc.abstractmethod
    def _orient_derived(self, parent_frame, parent_origin, frame, origin):
        raise NotImplementedError()


class Identity(StaticHomogeneousTransform):
    """TODO
    """
    def _orient_derived(self, parent_frame, parent_origin, frame, origin):
        frame.orient(parent_frame, 'Body', [0, 0, 0], 'XYZ')
        origin.set_pos(parent_origin, 0)


class TranslateX(StaticHomogeneousTransform):
    """TODO
    """
    def __init__(self, dist):
        self._dist = dist

    def _orient_derived(self, parent_frame, parent_origin, frame, origin):
        frame.orient(parent_frame, 'Body', [0, 0, 0], 'XYZ')
        print(self._dist)
        print(parent_frame.x)
        origin.set_pos(parent_origin, self._dist * parent_frame.x)


class Joint(object):
    """TODO
    """

    __metaclass__ = abc.ABCMeta

    def __init__(self, transform=Identity()):
        self._transform = transform

    def _orient(self, name, parent_frame, parent_origin,
            child_frame, child_origin):
        self._frame = ReferenceFrame(name + '_joint_frame')
        self._origin = Point(name + '_joint_origin')
        self._transform._orient(parent_frame, parent_origin,
                self._frame, self._origin)

        self._orient_child(child_frame, child_origin)

    @abc.abstractmethod
    def _orient_child(self, frame, origin):
        raise NotImplementedError()


class RevoluteJoint(Joint):
    """TODO
    z-axis
    """
    def __init__(self, coordinate_name, *args, **kwargs):
        super(RevoluteJoint, self).__init__(*args, **kwargs)
        # TODO where should these be stored?
        self._coordinates = [dynamicsymbols(coordinate_name)]
        self._coordinatedots = [dynamicsymbols(coordinate_name, 1)]
        self._speeds = [dynamicsymbols('%s_u' % coordinate_name)]
        self._accelerations = [dynamicsymbols('%s_u' % coordinate_name, 1)]

    def _orient_child(self, frame, origin):
        frame.orient(self._frame, 'Axis', [self._coordinates[0], self._frame.z])
        frame.set_ang_vel(self._frame, self._speeds[0] * self._frame.z)
        frame.set_ang_acc(self._frame, self._accelerations[0] * self._frame.z)
        origin.set_pos(self._origin, 0)
        origin.v2pt_theory(self._origin, self._frame, frame)
