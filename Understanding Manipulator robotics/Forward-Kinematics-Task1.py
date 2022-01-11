#!/usr/bin/env python
import sympy as sp

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

p_end = m_0_4[:3,3]
p_joint3 = m_0_3[:3,3]
p_joint2 = m_0_2[:3,3]
p_joint1 = m_0_1[:3,3]

x_data, y_data, z_data = [],[],[]

values = {l1:1,l2:1,l3:1,theta1:0,theta2:0,theta3:0}

end = [sp.pi*2, sp.pi/2, sp.pi/2]
i=0
for j in range(0,3):
    i=0
    while(i<end[j]):
        if(j==0):
            values[theta1] = i
        elif(j==1):
            values[theta2] = i
        else:
            values[theta3] = i
        i += 0.05
        x3 = p_end.subs(values)[0]
        y3 = p_end.subs(values)[1]
        z3 = p_end.subs(values)[2]
        x2 = p_joint3.subs(values)[0]
        y2 = p_joint3.subs(values)[1]
        z2 = p_joint3.subs(values)[2]
        x1 = p_joint2.subs(values)[0]
        y1 = p_joint2.subs(values)[1]
        z1 = p_joint2.subs(values)[2]
        x_data.append([x1,x2,x3])
        y_data.append([y1,y2,y3])
        z_data.append([z1,z2,z3])

from plotting import animation
arm = animation(x_data, y_data, z_data)
arm.plot_3D()
