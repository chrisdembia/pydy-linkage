from __future__ import print_function, division

import abc

from sympy import symbols
from sympy.physics.mechanics import dynamicsymbols
from sympy.physics.mechanics import functions
from sympy.physics.mechanics import Point, ReferenceFrame, RigidBody
from sympy.physics.mechanics import KanesMethod
from sympy.physics.mechanics.essential import _check_vector

__all__ = ['Linkage', 'RevoluteJoint', 'Identity', 'TranslateX']


class Event(object):
    """TODO
    """
    def __init__(self, description):
        self._description = description
        self._subscribers = list()

    def subscriber_new(self, subscriber):
        self._subscribers.append(subscriber)

    def fire(self, *arg, **args):
        for subscriber in self._subscribers:
            subscriber(*arg, **args)


# TODO it would be fantastic if I could set up listeners/callbacks, so that I
# could cache things like the mass matrix, and invalidate it if something is
# added to the system.
class MultiBodySystem(object):


    def _get_gravity_vector(self):
        return self._gravity_vector

    def _set_gravity_vector(self, vector):
        self._gravity_vector = vector

    gravity_vector = property(_get_gravity_vector, _set_gravity_vector)


class Linkage(MultiBodySystem):
    """TODO
    """

    def __init__(self, name):
        self._name = name
        self._root = RootLink(self) # TODO maybe don't need backpointer
        self._constants = dict()
        self._constant_descs = dict()
        self._forces = dict()
        self._gravity_vector = None
        self._gravity_vector_updated = Event(
                'Update gravity forces when gravity vector is changed.')
        self._gravity_vector_updated.subscriber_new(self._manage_gravity_forces)

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
        return self._constants[name]

    def force_new(self, name, point_of_application, vec):
        """TODO
        reserved names: '_gravity'
        """
        # TODO 
        #if name in self._forces:
        #    # TODO
        #    raise Exception("Force with name '{}' already exists.".format(name))
        self._forces[name] = (point_of_application, vec)

    def force_del(self, name):
        self._forces.pop(name)

    def _manage_gravity_forces(self):
        for body in self.body_list():
            self.force_new('%s_gravity', body.masscenter, body.mass * self.gravity_vector)

    def _get_gravity_vector(self):
        return self._gravity_vector
    def _set_gravity_vector(self, vec):
        _check_vector(vec)
        self._gravity_vector = vec
        self._gravity_vector_updated.fire()
    gravity_vector = property(_get_gravity_vector, _set_gravity_vector)

    def independent_coordinates(self):
        return self.root.independent_coordinates_in_subtree()

    def independent_speeds(self):
        return self.root.independent_speeds_in_subtree()

    def kinematic_differential_equations(self):
        return self.root.kinematic_differential_equations_in_subtree()

    def body_list(self):
        return self.root.body_list_in_subtree()

    def force_list(self):
        return self._forces.values()

    #TODO def _init_kanes_method(self):
    #TODO     # TODO move the creation of Kane's Method somewhere else.
    #TODO     self._kanes_method = KanesMethod(self.independent_coordinates,
    #TODO             self.independent_speeds, self.kinematic_diffeqs)
    #TODO     # TODO must make this call to get the mass matrix, etc.?
    #TODO     self._kanes_method.kanes_equations(self.force_list, self.body_list)

    def mass_matrix(self):
        #if not (self._kanes_method and self.up_to_date):
        #    self._init_kanes_method()
        # TODO move the creation of Kane's Method somewhere else.
        self._kanes_method = KanesMethod(self.root.frame,
                q_ind=self.independent_coordinates(),
                u_ind=self.independent_speeds(),
                kd_eqs=self.kinematic_differential_equations()
                )
        # TODO must make this call to get the mass matrix, etc.?
        self._kanes_method.kanes_equations(self.force_list(), self.body_list());
        return self._kanes_method.mass_matrix

    def state_derivatives(self):
        # TODO find a way to use a cached mass matrix.
        kin_diff_eqns = self._kanes_method.kindiffdict()
        state_derivatives = self.mass_matrix.inv() * self._kanes_method.forcing
        state_derivatives = state_derivatives.subs(kin_diff_eqns)
        state_derivatives.simplify()
        return state_derivatives

    def _check_link_name(self):
        # TODO
        pass


class Link(object):

    __metaclass__ = abc.ABCMeta

    def __init__(self, linkage, name):
        # 'Backpointer' to the Linkage containing this link. TODO reconsider.
        self._linkage = linkage
        self._name = name
        self._children = dict()

    def _get_linkage(self):
        return self._linkage

    linkage = property(_get_linkage)

    def _get_name(self):
        return self._name
    name = property(_get_name)

    def _get_frame(self):
        return self._frame
    frame = property(_get_frame)

    def _get_origin(self):
        return self._origin
    origin = property(_get_origin)

    def _get_children(self):
        return self._children

    children = property(_get_children)

    def link_new(self, name, joint):
        # TODO walk the tree to ensure a link with this name does not exist.
        self._children[name] = DynamicLink(self.linkage, name, joint, self)
        return self._children[name]

    def independent_coordinates_in_subtree(self):
        indep_coords = list()
        for child in self.children.values():
            indep_coords += child.independent_coordinates_in_subtree()
        return indep_coords

    def independent_speeds_in_subtree(self):
        indep_speeds = list()
        for child in self.children.values():
            indep_speeds += child.independent_speeds_in_subtree()
        return indep_speeds

    def kinematic_differential_equations_in_subtree(self):
        kin_diffeqs = list()
        for child in self.children.values():
            kin_diffeqs += child.kinematic_differential_equations_in_subtree()
        return kin_diffeqs

    def body_list_in_subtree(self):
        body_list = list()
        for child in self.children.values():
            body_list += child.body_list_in_subtree()
        return body_list


class RootLink(Link):
    """TODO
    """
    def __init__(self, linkage):
        super(RootLink, self).__init__(linkage, 'root')
        # TODO rename to avoid name conflicts, or inform the user that 'N' is
        # taken.
        self._frame = ReferenceFrame('N')
        self._origin = Point('NO')
        self._origin.set_vel(self._frame, 0)
        # TODO need to set_acc?
        self._origin.set_acc(self._frame, 0)


class DynamicLink(Link):
    """TODO
    """

    def __init__(self, linkage, name, joint, parent):
        """TODO
        """
        super(DynamicLink, self).__init__(linkage, name)
        self._joint = joint
        # Give the joint access to the link.
        self._joint._link = self
        self._parent = parent

        # Create rigid body.
        mass = symbols(name + '_mass')
        frame = ReferenceFrame(name + '_frame')
        self._origin = Point(name + '_origin')

        # TODO explain what this method call does.
        joint._orient(name, parent.frame, parent.origin, frame, self._origin)

        # All expressed in body frame.
        masscenter = self._origin.locatenew(name + '_masscenter',
                symbols(name + '_mcx') * frame.x +
                symbols(name + '_mcy') * frame.y +
                symbols(name + '_mcz') * frame.z)
        masscenter.set_vel(frame, 0)
        masscenter.v2pt_theory(self._origin, linkage.root.frame, frame)
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

    def _get_rigidbody(self):
        return self._rigidbody
    rigidbody = property(_get_rigidbody)

    def _get_joint(self):
        return self._joint
    joint = property(_get_joint)

    def _get_frame(self):
        return self._rigidbody.frame
    # TODO must redeclare property for the above method to override the
    # superclass' method.
    frame = property(_get_frame)

    def _get_parent(self):
        return self._parent
    parent = property(_get_parent)

    def independent_coordinates_in_subtree(self):
        indep_coords = self.joint.independent_coordinates()
        for child in self.children.values():
            indep_coords += child.independent_coordinates_in_subtree()
        return indep_coords

    def independent_speeds_in_subtree(self):
        indep_speeds = self.joint.independent_speeds()
        for child in self.children.values():
            indep_speeds += child.independent_speeds_in_subtree()
        return indep_speeds

    def kinematic_differential_equations_in_subtree(self):
        kin_diffeqs = self.joint.kinematic_differential_equations()
        for child in self.children.values():
            kin_diffeqs += child.kinematic_differential_equations_in_subtree()
        return kin_diffeqs

    def body_list_in_subtree(self):
        body_list = [self.rigidbody]
        for child in self.children.values():
            body_list += child.body_list_in_subtree()
        return body_list

class StaticHomogeneousTransform(object):
    """TODO
    """

    __metaclass__ = abc.ABCMeta

    def _get_joint(self):
        return self._joint
    joint = property(_get_joint)

    def _orient(self, parent_frame, parent_origin, frame, origin):
        self._orient_derived(parent_frame, parent_origin, frame, origin)
        frame.set_ang_vel(parent_frame, 0)
        frame.set_ang_acc(parent_frame, 0)
        origin.set_vel(frame, 0)
        origin.set_acc(frame, 0)
        origin.v2pt_theory(parent_origin, self.joint.link.linkage.root.frame, frame)

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
        # TODO check to make sure dist is not a dynamicsymbol.
        self._dist = dist

    def _orient_derived(self, parent_frame, parent_origin, frame, origin):
        frame.orient(parent_frame, 'Body', [0, 0, 0], 'XYZ')
        origin.set_pos(parent_origin, self._dist * parent_frame.x)


class Joint(object):
    """TODO
    """

    __metaclass__ = abc.ABCMeta

    def __init__(self, transform=Identity()):
        self._transform = transform
        self._transform._joint = self

    def _orient(self, name, parent_frame, parent_origin,
            child_frame, child_origin):
        self._frame = ReferenceFrame(name + '_joint_frame')
        self._origin = Point(name + '_joint_origin')
        self._transform._orient(parent_frame, parent_origin,
                self._frame, self._origin)

        self._orient_child(child_frame, child_origin)

    def _get_link(self):
        return self._link
    link = property(_get_link)

    @abc.abstractmethod
    def _get_coordinates(self):
        raise NotImplementedError()
    coordinates = property(_get_coordinates)

    @abc.abstractmethod
    def _get_coordinatedots(self):
        raise NotImplementedError()
    coordinatedots = property(_get_coordinatedots)

    @abc.abstractmethod
    def _get_speeds(self):
        raise NotImplementedError()
    speeds = property(_get_speeds)

    @abc.abstractmethod
    def _get_speeddots(self):
        raise NotImplementedError()
    speeddots = property(_get_speeddots)

    @abc.abstractmethod
    def independent_coordinates(self):
        raise NotImplementedError()

    @abc.abstractmethod
    def independent_speeds(self):
        raise NotImplementedError()

    @abc.abstractmethod
    def kinematic_differential_equations(self):
        """TODO
        must be a list.
        """
        raise NotImplementedError()

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
        # TODO this would manage creating an extra speed for a quaternion.
        # TODO ensure coordinate names are not getting overwritten.
        self._rotation = dynamicsymbols(coordinate_name)
        self._rotationdot = dynamicsymbols(coordinate_name, 1)
        self._rotspeed = dynamicsymbols('%s_u' % coordinate_name)
        self._rotspeeddot = dynamicsymbols('%s_u' % coordinate_name, 1)

    def _get_coordinates(self):
        return [self._rotation]
    coordinates = property(_get_coordinates)

    def _get_coordinatedots(self):
        return [self._rotationdot]
    coordinatedots = property(_get_coordinatedots)

    def _get_speeds(self):
        return [self._rotspeed]
    speeds = property(_get_speeds)

    def _get_speeddots(self):
        return [self._rotspeeddot]
    speeddots = property(_get_speeddots)

    def independent_coordinates(self):
        return self.coordinates

    def independent_speeds(self):
        return self.speeds

    def kinematic_differential_equations(self):
        return [self._rotationdot - self._rotspeed]

    def _orient_child(self, frame, origin):
        frame.orient(self._frame, 'Axis', [self._rotation, self._frame.z])
        frame.set_ang_vel(self._frame, self._rotspeed * self._frame.z)
        frame.set_ang_acc(self._frame, self._rotspeeddot * self._frame.z)
        origin.set_pos(self._origin, 0)
        origin.set_vel(frame, 0)
        origin.v2pt_theory(self._origin, self.link.linkage.root.frame, frame)
