#!/usr/bin/env python
import sympy as sp

from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax', pretty_print=False)

from sympy import symbols
u = symbols('u')

H = sp.Matrix([[2,-2,1,1],[-3,3,-2,-1],[0,0,1,0],[1,0,0,0]])
X_coeff = sp.Matrix([0,0,0,0])
Y_coeff = sp.Matrix([0,0,0,0])
start_point = [1,1]
end_point = [3,3]
tangent_point = [4,2]
X_bounds = sp.Matrix([start_point[0],end_point[0],tangent_point[0]-start_point[0],-end_point[0]+tangent_point[0]])
Y_bounds = sp.Matrix([start_point[1],end_point[1],tangent_point[1]-start_point[1],-end_point[1]+tangent_point[1]])

X_coeff = H*X_bounds
Y_coeff = H*Y_bounds

expr_x = X_coeff[0]*(u**3)+X_coeff[1]*(u**2)+X_coeff[2]*(u)+X_coeff[3]
expr_y = Y_coeff[0]*(u**3)+Y_coeff[1]*(u**2)+Y_coeff[2]*(u)+Y_coeff[3]
print("\n Parametric equation from {} to {} with tangents at {} is:".format(start_point,end_point,tangent_point))
print("(",expr_x," , ",expr_y,")")