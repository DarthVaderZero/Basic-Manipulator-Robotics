#!/usr/bin/env python
import sympy as sp
import numpy as np
from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax', pretty_print=False)

from sympy.physics.mechanics import dynamicsymbols
theta1, theta2, theta3, l1, l2, l3, theta, alpha, a, d = dynamicsymbols('theta1 theta2 theta3 l1 l2 l3 theta alpha a d')

m_iprevious_i = sp.Matrix([[sp.cos(theta),-sp.sin(theta),0,a], [sp.sin(theta)*sp.cos(alpha),sp.cos(theta)*sp.cos(alpha),-sp.sin(alpha),-sp.sin(alpha)*d], [sp.sin(theta)*sp.sin(alpha),sp.cos(theta)*sp.sin(alpha),sp.cos(alpha),sp.cos(alpha)*d], [0,0,0,1]])

m_0_1 = m_iprevious_i.subs({alpha:0, a:0, d:0, theta:theta1})
m_1_2 = m_iprevious_i.subs({alpha:-(sp.pi)/2, a:l1, d:0, theta:theta2})
m_2_3 = m_iprevious_i.subs({alpha:(sp.pi)/2, a:l2, d:0, theta:theta3})
m_3_4 = m_iprevious_i.subs({alpha:0, a:l3, d:0, theta:0})

m_0_2 = (m_0_1)*(m_1_2)
m_0_3 = (m_0_2)*(m_2_3)
m_0_4 = (m_0_3)*(m_3_4)

p_end = m_0_4[:3,3] #p_end[0] is x, p_end[1] is y, p_end[2] is z
p_joint3 = m_0_3[:3,3]
p_joint2 = m_0_2[:3,3]
p_joint1 = m_0_1[:3,3]

p_end_arm = p_end.subs({l1:1, l2:1, l3:1})
p_joint3_arm = p_joint3.subs({l1:1, l2:1, l3:1})
p_joint2_arm = p_joint2.subs({l1:1, l2:1, l3:1})
p_joint1_arm = p_joint1.subs({l1:1, l2:1, l3:1})

def Inverse(x,y,z,accuracy):  #Newtons Method
    f1 = p_end_arm[0]-x 
    f2 = p_end_arm[1]-y
    f3 = p_end_arm[2]-z
    f1_val = sp.lambdify([theta1,theta2,theta3], f1, "numpy")
    f2_val = sp.lambdify([theta1,theta2,theta3], f2, "numpy")
    f3_val = sp.lambdify([theta1,theta2,theta3], f3, "numpy")
    F = sp.Matrix([f1,f2,f3]) #the matrix who's jacobian we find 
    values_mat = sp.Matrix([0,0,sp.pi/4])
    values = np.array(values_mat).astype(np.float64)
    for i in range(3):
        values_mat[i] = values[i] 
    DF = sp.simplify(F.jacobian([theta1,theta2,theta3]))
    DF_num=DF.subs({theta1:values_mat[0],theta2:values_mat[1],theta3:values_mat[2]})
    sp.pprint(DF_num)
#driver
x_in = int(input("x coordinate: "))
y_in = int(input("y coordinate: "))
z_in = int(input("z coordinate: "))
Inverse(x_in,y_in,z_in,0.001)

