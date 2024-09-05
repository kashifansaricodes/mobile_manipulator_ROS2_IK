#ik_solver.py

import sympy as sp
import numpy as np

q1 , q2 , q3 , q4  = sp.symbols('q1 q2 q3 q4')

def DH_trans_matrix(params):
    theta, alpha, a, d = params
    mat = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                    [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                    [0, sp.sin(alpha), sp.cos(alpha), d],
                    [0, 0, 0, 1]])
    return mat

def joint_transforms(DH_params):
    transforms = []

    transforms.append(sp.eye(4))  # Assuming the first joint is at the origin

    for el in DH_params:
        transforms.append(DH_trans_matrix(el))

    return transforms

def jacobian_expr(DH_params):
    transforms = joint_transforms(DH_params)

    trans_EF = transforms[0]

    for mat in transforms[1:]:
        trans_EF = trans_EF * mat

    pos_EF = trans_EF[0:3, 3]

    DOF = len(DH_params)
    J = sp.zeros(6, DOF)

    for joint in range(DOF):
        trans_joint = transforms[0]

        for mat in transforms[1:joint+1]:
            trans_joint = trans_joint * mat

        z_axis = trans_joint[0:3, 2]
        pos_joint = trans_joint[0:3, 3]

        Jv = z_axis.cross(pos_EF - pos_joint)
        Jw = z_axis

        J[0:3, joint] = Jv
        J[3:6, joint] = Jw

    J = sp.simplify(J)
    return J

def jacobian_subs(joints, jacobian_sym):
    # Convert to list if it's an ndarray
    if isinstance(joints, np.ndarray):
        joints = joints.flatten().tolist()

    J_l = jacobian_sym

    # Assuming q1, q2, q3, q4 are already defined as symbolic variables
    J_l = J_l.subs(q1, joints[0])
    J_l = J_l.subs(q2, joints[1])
    J_l = J_l.subs(q3, joints[2])
    J_l = J_l.subs(q4, joints[3])

    return J_l

def trans_EF_eval(joints, DH_params):
    # Convert to list if it's an ndarray
    if isinstance(joints, np.ndarray):
        joints = joints.flatten().tolist()

    transforms = joint_transforms(DH_params)

    trans_EF = transforms[0]

    for mat in transforms[1:]:
        trans_EF = trans_EF * mat

    trans_EF_cur = trans_EF

    trans_EF_cur = trans_EF_cur.subs(q1, joints[0])
    trans_EF_cur = trans_EF_cur.subs(q2, joints[1])
    trans_EF_cur = trans_EF_cur.subs(q3, joints[2])
    trans_EF_cur = trans_EF_cur.subs(q4, joints[3])

    return trans_EF_cur

def plot_pose(joints, DH_params):

    # Convert to list if it's an ndarray
    if (isinstance(joints, np.ndarray)):
        joints = joints.flatten().tolist()

    transforms = joint_transforms(DH_params)

    trans_EF = trans_EF_eval(joints, DH_params)

    pos_EF = trans_EF[0:3,3]

    xs = []
    ys = []
    zs = []

    J = sp.zeros(6, DOF)

    for joint in range(DOF):

        trans_joint = transforms[0]

        for mat in transforms[1:joint+1]:

            trans_joint = trans_joint*mat

        pos_joint = trans_joint[0:3,3]

        pos_joint = pos_joint.subs(q1, joints[0])
        pos_joint = pos_joint.subs(q2, joints[1])
        pos_joint = pos_joint.subs(q3, joints[2])
        pos_joint = pos_joint.subs(q4, joints[3])

        xs.append(pos_joint[0])
        ys.append(pos_joint[1])
        zs.append(pos_joint[2])

    xs.append(pos_EF[0])
    ys.append(pos_EF[1])
    zs.append(pos_EF[2])

    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim3d(-60,60)
    ax.set_ylim3d(-60,60)
    ax.set_zlim3d(0, 120)

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')

    ax.plot(xs, ys, zs)

def joint_limits(joints):

    # Joint 1
    if (joints[0][0] < -sp.pi/2):

        joints[0][0] = -sp.pi/2

    elif (joints[0][0] > sp.pi/2):

        joints[0][0] = sp.pi/2


    # Joint 2
    if (joints[1][0] < -sp.pi/2):

        joints[1][0] = -sp.pi/2

    elif (joints[1][0] > sp.pi/2):

        joints[1][0] = sp.pi/2

    # Joint 3
    if (joints[2][0] < -sp.pi/2):

        joints[2][0] = -sp.pi/2

    elif (joints[2][0] > sp.pi/2):

        joints[2][0] = sp.pi/2

    # Joint 4
    if (joints[3][0] < -sp.pi/2):

        joints[3][0] = -sp.pi/2

    elif (joints[3][0] > sp.pi/2):

        joints[3][0] = sp.pi/2

    return joints

