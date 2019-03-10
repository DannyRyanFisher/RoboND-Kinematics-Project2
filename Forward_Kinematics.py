import pickle
from sympy import *
from mpmath import radians
from time import time

start_time = time()

# Define DH param symbols
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offset
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link length
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #twist angle
# Joint anfle symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

#Modified DH params
DH_Table = {alpha0:      0,  a0:      0,  d1:  0.75,  q1:          q1,
            alpha1: -pi/2.,  a1:   0.35,  d2:     0,  q2: -pi/2. + q2,
            alpha2:      0,  a2:   1.25,  d3:     0,  q3:          q3,
            alpha3: -pi/2.,  a3: -0.054,  d4:   1.5,  q4:          q4,
            alpha4:  pi/2.,  a4:      0,  d5:     0,  q5:          q5,
            alpha5: -pi/2.,  a5:      0,  d6:     0,  q6:          q6,
            alpha6:      0,  a6:      0,  d7: 0.303,  q7:           0}

# Define Modified DH Transformation matrix
def TF_Matrix(alpha,a,d,q):
	TF = Matrix([   [            cos(q),           -sin(q),           0,             a],
                        [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                        [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                        [                 0,                 0,           0,             1]])

	return TF

# Create individual transformation matrices
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

end_time = time()
duration = end_time - start_time

### FK class
class Forward_Kinematics():
	def __init__(self, T0_EE, T0_1, T1_2, T2_3, q1, q2, q3,q4, q5, q6):
		self.T0_EE = T0_EE
		self.T0_1 = T0_1
		self.T1_2 = T1_2
		self.T2_3 = T2_3
		self.q1 = q1
		self.q2 = q2
		self.q3 = q3
		self.q4 = q4
		self.q5 = q5
		self.q6 = q6

FK = Forward_Kinematics(T0_EE, T0_1, T1_2, T2_3, q1, q2, q3, q4, q5, q6)

### Duration calculation
#print("Time to process was: ", duration, T0_EE)
