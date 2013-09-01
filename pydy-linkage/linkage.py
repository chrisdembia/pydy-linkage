from __future__ import print_function, division

from sympy import symbols
from sympy.physics.mechanics.functions import inertia

__all__ = ['Linkage']

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

    def _get_name(self):
        return self._name
    name = property(_get_name)

    def _get_frame(self):
        return self._frame
    frame = property(_get_frame)

    def link_new(self, name, joint):
        self._children[name] = Link(name, joint, parent)


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
        joint.orient(name, parent.frame, parent.origin, frame, self._origin)

        # All expressed in body frame.
        masscenter = self._origin.locatenew(name + '_masscenter',
                symbols(name + '_mcx') * frame.x +
                symbols(name + '_mcy') * frame.y +
                symbols(name + '_mcz') * frame.z)
        inertia = inertia(frame, 
                symbols(name + '_ixx_central')
                symbols(name + '_iyy_central')
                symbols(name + '_izz_central')
                symbols(name + '_ixy_central')
                symbols(name + '_iyz_central')
                symbols(name + '_izx_central'))
        # TODO allow specification of non-central inertia
        self._rigidbody = RigidBody(name + '_rigidbody',
                masscenter, frame, mass, (inertia, masscenter))

    def _get_joint(self):
        return self._joint
    joint = property(_get_joint)

    def _get_frame(self):
        return self._rigidbody.frame

    def _get_origin(self):
        return self._origin
    origin = property(_get_origin)

    def _get_parent(self):
        return self._parent
    parent = property(_get_parent)


class Joint(object):
    """TODO
    """
    def __init__(self, transform=None):
        self._transform = transform

    def orient(self, name, parent_frame, parent_origin,
            child_frame, child_origin):
        self._frame = ReferenceFrame(name + '_joint_frame')
        self._origin = Point(name + '_joint_origin')
        if self._transform == None:
            self._frame.orient(parent_frame, 'Body', [0, 0, 0], 'XYZ')
            self._origin.set_pos(parent_origin, 0)
        else:
            self._transform.orient(parent_frame, parent_origin,
                    self._frame, self._origin)

        self.orient_child(child_frame, child_origin)

    def _orient_child(frame, origin):
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

    def _orient_child(frame, origin):
        frame.orient(self._frame, 'Axis', [self._coordinates[0], self._frame.z])
        frame.set_ang_vel(self._frame, self._speeds[0] * self._frame.z)
        frame.set_ang_acc(self._frame, self._accelerations[0] * self._frame.z)
        origin.set_pos(self._origin, 0)
        origin.v2pt_theory(self._origin, self._frame, frame)


class StaticHomogeneousTransform(object):
    """TODO
    """
    def orient(parent_frame, parent_origin, frame, origin):
        self._orient(parent_frame, parent_origin, frame, origin)
        frame.set_ang_vel(parent_frame, 0)
        frame.set_ang_acc(parent_frame, 0)
        origin.set_vel(parent_origin, 0)
        origin.set_acc(parent_origin, 0)

    def _orient(parent_frame, parent_origin, frame, origin):
        raise NotImplementedError()


class TranslateX(Transform):
    """TODO
    """
    def __init__(self, dist):
        self._dist = dist

    def _orient(parent_frame, parent_origin, frame, origin):
        origin.set_pos(parent_origin, self._dist * parent_frame.x)
        


