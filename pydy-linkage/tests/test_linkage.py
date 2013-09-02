from linkage import *
from sympy import symbols
from sympy.physics.mechanics import Point, ReferenceFrame, Dyadic, RigidBody
from sympy.physics.mechanics import dynamicsymbols

def test_rrr_manipulator():
    sys = Linkage('rrr_manipulator')

    # Quantities.
    # ===========

    # Constants.
    # ----------
    L1 = sys.constant_new('L1', 'length of link 1')
    L2 = sys.constant_new('L2', 'length of link 2')

    # Tree structure.
    # ---------------
    link1 = sys.root.link_new('link1', RevoluteJoint('q1'))
    link2 = link1.link_new('link2', RevoluteJoint('q2', TranslateX(L1)))
    link3 = link2.link_new('link3', RevoluteJoint('q3', TranslateX(L2)))

    N = sys.root.frame
    #sys.gravity_vector = -g * N.y

    print sys
    #print sys.coordinates
    #print sys.mass_matrix
    #print sys.state_derivatives

"""
def test_double_pendulum_on_cart():
    
    h = sys.constant_new('h', 'height of robot arm on cart')
    L1 = sys.constant_new('L2', 'length of link 1')
    
    cart = sys.root.link_new('cart',
            #PrismaticJoint('d1', 'z'))
            PrismaticJoint('d1'))
    link1 = cart.link_new('link1',
            RevoluteJoint('theta2', TranslateX(-h)), axis='-y')
    link2 = link1.link_new('link2',
            RevoluteJoint('theta3', TranslateZ(L2)), axis='-y')
            
    link1 = cart.link_new('link1',
            RevoluteJoint('theta2', RX(np.pi/2)*TX(-h), RX(-np.pi/2)))
    link2 = link1.link_new('link2',
            RevoluteJoint('theta3', RX(np.pi/2)*TZ(L2), RX(-np.pi/2)))
    N = sys.root.frame
    sys.gravity_vector = g * N.x

def test_babyboot():
    sys = MultiBodySystem('babyboot')

    # Quantities.
    # ===========

    # Constants.
    # ----------
    LA = sys.constant_new('LA', 'length of link A')
    LB = sys.constant_new('LB', 'length of second link')
    mA = sys.constant_new('mA', 'mass of link A')
    mB = sys.constant_new('mB', 'mass of second link')
    IAx = sys.constant_new('IAx', 'x measure of moment of inertia for link A')
    IBx = sys.constant_new('IBx', 'x measure of moment of inertia for link B')
    IBy = sys.constant_new('IBy', 'y measure of moment of inertia for link B')
    IBz = sys.constant_new('IBz', 'z measure of moment of inertia for link B')

    # Variable.
    # ---------
    qA = sys.variable_new('qA', 'rotation of lace', 1)
    qB = sys.variable_new('qB', 'rotation of shoe', 1)
    uA = sys.variable_new('uA', 'time derivative of rotation of lace', 1)
    uB = sys.variable_new('uB', 'time derivative of rotation of shoe', 1)

    
    # Declare rigid bodies.
    # =====================
    lace = sys.rigid_body_new('lace')
    shoe = sys.rigid_body_new('shoe')

    # Rotational kinematics.
    # ======================
    N = sys.reference_frame_new('N', desc='Newtonian')
    A = lace.frame
    A.orient(AxisOrientation(N, N.x, qA))
    B = shoe.frame
    B.orient(AxisOrientation(A, A.z, qB))

    A.set_ang_vel(N, uA * N.x)
    B.set_ang_vel(A, uB * A.z)

    sys.constant_specific_gravity_force_is(-g * N.z)


# -- Set up geometry.
# Fixed frame at base of upper rod.
N = ReferenceFrame('N')
# Frame tracking the upper rod.
frameA = N.orientnew('frameA', 'Axis', [qA, N.x])
frameB = frameA.orientnew('frameB', 'Axis', [qB, frameA.z])

# TODO why do we need to do this?
frameA.set_ang_vel(N, uA * N.x)
frameA.set_ang_acc(N, frameA.ang_vel_in(N).dt(N)) # TODO

frameB.set_ang_vel(frameA, uB * frameA.z)
frameB.set_ang_acc(frameA, frameB.ang_vel_in(frameA).dt(frameA))


# Origin.
NO = Point('NO')
NO.set_vel(N, 0)

# Center of mass of upper rod.
Acm = NO.locatenew('Acm', -LA * frameA.z)
Acm.v2pt_theory(NO, N, frameA) # Don't need this in MotionGenesis.
Bcm = NO.locatenew('Bcm', - LB * frameA.z)
Bcm.v2pt_theory(NO, N, frameA) # Don't need this in MotionGenesis.

# Inertia dyadic.
# TODO inertia dyadics are about a specific point, right? are we requiring the user to define it about the COM always or something? idk if this info should be separate. we have inertia info floating about in 2 different locations.
IA = inertia(frameA, IAx, 0, 0)
IB = inertia(frameB, IBx, IBy, IBz)

# Create rigid bodies.
BodyA = RigidBody('BodyA', Acm, frameA, mA, (IA, Acm))
BodyB = RigidBody('BodyB', Bcm, frameB, mB, (IB, Bcm))
BodyList = [BodyA, BodyB]

# Forces.
# Would be nice to have a method that applies gravity force to all objects.
ForceList = [(Acm, - mA * g * N.z), (Bcm, - mB * g * N.z)]

# Kinematic differential equations. TODO necessary?
kd = [qAd -uA, qBd - uB] # TODO constrain upper rod to O?

KM = KanesMethod(N, q_ind=[qA, qB], u_ind=[uA, uB], kd_eqs=kd)
(fr, frstar) = KM.kanes_equations(ForceList, BodyList)

# Get equations of motion.
MM = KM.mass_matrix
forcing = KM.forcing
rhs = MM.inv() * forcing
kdd = KM.kindiffdict()
rhs = rhs.subs(kdd)
rhs.simplify()
mprint(rhs)
"""
