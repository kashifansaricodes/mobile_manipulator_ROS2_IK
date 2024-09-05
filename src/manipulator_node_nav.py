import numpy as np
from numpy import cos, sin, pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sympy as sp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import sympy as sp
import matplotlib as plt
import time

from mobilearm.src.ik_solver_nav import DH_trans_matrix , joint_transforms , joint_limits, jacobian_expr, jacobian_subs, trans_EF_eval

#robot_control_node.py
class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Initialize your DH parameters
        self.DOF = 4
        self.q1, self.q2, self.q3, self.q4 = sp.symbols('q1 q2 q3 q4')
        self.DH_params = []

        self.DH_params.append([self.q1, sp.pi/2, 0, 2.5])
        self.DH_params.append([self.q2, 0, 12.000, 0])
        self.DH_params.append([self.q3, 0, 8, 0])
        self.DH_params.append([self.q4, 0, 12.4, 0])

        self.joint_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.pos_container = Float64MultiArray ()
        #self.pose_sub = self.create_subscription(PoseStamped, 'target_pose', self.pose_callback, 10)

    # ... rest of the class ...

    def compute_inverse_kinematics(self, target_pose):
        # Call your inverse kinematics solver with DH parameters
        # Modify this part based on your specific implementation
        joints_init = [0 ,0 , 0 , 0]  # Assuming 4 DOF for the given DH parameters
        joints, e_trace  , x_pos , y_pos , z_pos  = self.i_kine(joints_init, target_pose, self.DH_params, error_trace=True, no_rotation=True, joint_lims=False)
        #joints = [float(j.evalf()) for j in joints]


    def i_kine(self , joints_init, target, DH_params, error_trace=False, no_rotation=True, joint_lims=True):

        x_pos = []
        y_pos = []
        z_pos = []
        joints = joints_init

        xr_desired = target[0:3,0:3]
        xt_desired = target[0:3,3]

        x_dot_prev = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        e_trace = []

        iters = 0

        #print("Finding symbolic jacobian")

        # We only do this once since it's computationally heavy
        jacobian_symbolic = jacobian_expr(DH_params)

        #print("Starting IK loop")

        final_xt = 0

        while(1):

            jac = jacobian_subs(joints, jacobian_symbolic)

            jac = np.array(jac).astype(np.float64)

            trans_EF_cur = trans_EF_eval(joints, DH_params)

            trans_EF_cur = np.array(trans_EF_cur).astype(np.float64)



            xr_cur = trans_EF_cur[0:3,0:3]
            xt_cur = trans_EF_cur[0:3,3]

            x_pos.append(xt_cur[0])
            y_pos.append(xt_cur[1])
            z_pos.append(xt_cur[2])

            final_xt = xt_cur

            xt_dot = xt_desired - xt_cur


            # Find error rotation matrix
            R = xr_desired @ xr_cur.T


            # convert to desired angular velocity
            v = np.arccos((R[0,0] + R[1,1] + R[2,2] - 1)/2)
            r = (0.5 * sin(v)) * np.array([[R[2,1]-R[1,2]],
                                        [R[0,2]-R[2,0]],
                                        [R[1,0]-R[0,1]]])


            # The large constant just tells us how much to prioritize rotation
            xr_dot = 200 * r * sin(v)

            # use this if you only care about end effector position and not rotation
            if (no_rotation):

                xr_dot = 0 * r

            xt_dot = xt_dot.reshape((3,1))

            x_dot = np.vstack((xt_dot, xr_dot))

            x_dot_norm = np.linalg.norm(x_dot)

            #print(x_dot_norm)

            if (x_dot_norm > 25):

                x_dot /= (x_dot_norm/25)

            x_dot_change = np.linalg.norm(x_dot - x_dot_prev)

            # This loop now exits if the change in the desired movement stops changing
            # This is useful for moving close to unreachable points
            if (x_dot_change < 0.005):

                break

            x_dot_prev = x_dot

            e_trace.append(x_dot_norm)

            Lambda = 12
            Alpha = 0.5

            joint_change = Alpha * np.linalg.inv(jac.T@jac + Lambda**2*np.eye(self.DOF)) @ jac.T @ x_dot



            for i in range (100) :
                time.sleep(1)
                joints += joint_change
                print("publishing ",joints[0][0] , joints[1][0] , joints[2][0] , joints[3][0] )
                self.pos_container = Float64MultiArray(data = [joints[0][0] , joints[1][0] , joints[2][0] , joints[3][0]])
                self.joint_pub.publish(self.pos_container)

            if (joint_lims): joints = joint_limits(joints)

            iters += 1

        #print("Done in {} iterations".format(iters))

        #print("Final position is:")
        #print(final_xt)

        return (joints, e_trace  , x_pos , y_pos , z_pos ) if error_trace else (joints , x_pos , y_pos , z_pos )

    # ... rest of the class ...

def main(args=None):
    rclpy.init(args=args)
    robot_control_node = RobotControlNode()
    target_pose = np.array([[1, 0, 0, 32],
                   [0, 0, -1, 0],
                   [0, 1, 0, 2.5],
                   [0, 0, 0, 1]])
    robot_control_node.compute_inverse_kinematics(target_pose)
    rclpy.spin(robot_control_node)
    robot_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
