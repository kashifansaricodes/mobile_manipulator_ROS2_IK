import rclpy
from rclpy.node import Node
from ik_solver import  YourManipulatorController
import sympy as sp
from std_msgs.msg import Float64MultiArray 

class YourManipulatorNode(Node):
    def __init__(self):
        super().__init__('your_manipulator_node')
        self.manipulator_controller = YourManipulatorController()
        self.publisher_pos = self.create_publisher(Float64MultiArray , '/position_controller/commands' , 10)
    def generate_ideal_trajectory(self):
        x_y_z_values = []
        x = 32.4
        while x >= 20:
            z = ((75/124) * (x - 32.4)) + (2.5)
            y = 0
            temp = [x, y, z]
            x_y_z_values.append(temp)
            x = x - 0.124

        return x_y_z_values

    # Other methods...
    def execute_inverse_kinematics_trajectory(self):
        x_y_z_values = self.generate_ideal_trajectory()
        trajectory_points = []
        init_coord = [32.4, 0, 2.5]
        q = sp.Matrix([0, 0, 0, 0, 0])
        y_values_final = []
        x_values_final = []
        z_values_final = []
        θ1, θ2, θ3, θ4, θ5 = sp.symbols('θ1 θ2 θ3 θ4 θ5')
        J = self.manipulator_controller.calculate_jacobian(θ1, θ2, θ3, θ4, θ5)
        for coord in x_y_z_values:
            x_dot = (coord[0] - init_coord[0]) / 1
            y_dot = (coord[1] - init_coord[1]) / 1
            z_dot = (coord[2] - init_coord[2]) / 1

            X_dot = sp.Matrix([
                [x_dot],
                [y_dot],
                [z_dot],
                [0],
                [0],
                [0]
            ])
            J_0 = J.subs({
                    θ1: q[0], θ2: q[1], θ3: q[2], θ4: q[3], θ5: q[4]
                        })
            q_dot = J_0.pinv() * X_dot
            q += q_dot * 1
            q = sp.Matrix([element % (2 * sp.pi) for element in q])

            T1, T2, T20, T3, T30, T4, T40, T5, T50 = self.manipulator_controller.calculate_transformation_matrices(q[0], q[1], q[2], q[3], q[4])
            T_updated = T1 * T2 * T3  * T4 * T5
            x_values_final.append(T_updated[0, -1])
            y_values_final.append(T_updated[1, -1])
            z_values_final.append(T_updated[2, -1])

            if T_updated[0, -1] == 20 or T_updated[2, -1] == -5:
                print('here')
                break

            trajectory_points.append((float(T_updated[0, -1]), float(T_updated[1, -1]), float(T_updated[2, -1])))
            init_coord = coord

            # Example: Publish joint angles to the controllers
            joint_angles = Float64MultiArray(data=[q[0], q[1] , q[2] , q[3]])
            self.publisher_pos.publish(joint_angles)


def main(args=None):
    rclpy.init(args=args)
    node = YourManipulatorNode()
    # Example: Execute the ideal trajectory
    node.generate_ideal_trajectory ()
    node.execute_inverse_kinematics_trajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
