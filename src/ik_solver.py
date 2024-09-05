import sympy as sp
class YourManipulatorController:
    def __init__(self):
        pass

    def calculate_transformation_matrices(self, θ1, θ2, θ3, θ4, θ5):
        # Calculate individual transformation matrices
        T1 = self.T1(θ1)
        T2 = self.T2(θ2)
        T20 = T1 * T2
        T3 = self.T3(θ3)
        T30 = T1 * T2 * T3
        T4 = self.T4(θ4)
        T40 = T1 * T2 * T3 * T4
        T5 = self.T5(θ5)
        T50 = T1 * T2 * T3 * T4 * T5
        return T1, T2, T20, T3, T30, T4, T40, T5, T50

    def T1(self, θdeg1):
        return self.Transform_matrix(θdeg1, 0, 0, 0)

    def T2(self, θdeg2):
        return self.Transform_matrix(θdeg2, sp.pi/2, 0, 2.5)

    def T3(self, θdeg3):
        return self.Transform_matrix(θdeg3, 0, 12.000, 0)

    def T4(self, θdeg4):
        return self.Transform_matrix(θdeg4, 0, 8, 0)

    def T5(self, θdeg5):
        return self.Transform_matrix(θdeg5, 0, 12.4, 0)

    def Transform_matrix(self, θrad, αrad, a, d):
        return sp.Matrix([
            [sp.cos(θrad), -sp.sin(θrad) * sp.cos(αrad), sp.sin(θrad) * sp.sin(αrad), a * sp.cos(θrad)],
            [sp.sin(θrad), sp.cos(θrad) * sp.cos(αrad), -sp.cos(θrad) * sp.sin(αrad), a * sp.sin(θrad)],
            [0, sp.sin(αrad), sp.cos(αrad), d],
            [0, 0, 0, 1]
        ])

    def calculate_jacobian(self, θ1, θ2, θ3, θ4, θ5):
        # Calculate individual transformation matrices
        T1, T2, T20, T3, T30, T4, T40, T5, T50 = self.calculate_transformation_matrices(θ1, θ2, θ3, θ4, θ5)

        # Calculate end-effector position
        Points = T50[:3, -1]

        # Declaring all the partial derivatives of the Position of end effector wrt the joint angles
        dO_dθ1 = Points.diff(θ1)
        dO_dθ2 = Points.diff(θ2)
        dO_dθ3 = Points.diff(θ3)
        dO_dθ4 = Points.diff(θ4)
        dO_dθ5 = Points.diff(θ5)

        # Obtaining the axis vector Z for each joint
        Z1 = T1[:3, 2]
        Z2 = T20[:3, 2]
        Z3 = T30[:3, 2]
        Z4 = T40[:3, 2]
        Z5 = T50[:3, 2]

        # Jacobian matrix using the second method
        J = sp.Matrix([
            [dO_dθ1, dO_dθ2, dO_dθ3, dO_dθ4, dO_dθ5],
            [Z1, Z2, Z3, Z4, Z5]
        ])

        return J
