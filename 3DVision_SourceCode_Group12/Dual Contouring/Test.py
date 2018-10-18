# Date Created: 01/05/2017
# Author: Kunal Shrivastava
# This code takes a scalar signed distance grid as a text file, converts it to a function (over 3D coordinates)
# and its derivative and runs dual contouring algorithm (implemented in Dual_Contour1.pyc)).
# The resulting mesh is stored as a mayavi mesh.

import mayavi.mlab as mlab
import numpy as np
import Dual_Contour1 as DC
import math as m
from scipy.interpolate import RegularGridInterpolator

f_ = np.loadtxt('u.txt',dtype=float)
n=int(round(m.pow(f_.shape[0],1./3)))

n1=n
n2=n
n3=n

# if the resolution is not the same in all directions, then manually enter the reolution.
# n1=
# n2=
# n3=

f=f_.reshape(n1,n2,n3)


X = np.linspace(0,n1-1,n1)
Y = np.linspace(0,n2-1,n2)
Z = np.linspace(0,n3-1,n3)


my_interpol_f = RegularGridInterpolator((X,Y,Z),f,method='linear',bounds_error=False)

# This is just to test any implicit function. Leave this commented.

##center = np.array([16,16,16])
##radius = 10
##
##def test_f(x):
##    d = x-center
##    return np.dot(d,d) - radius**2
##
##def test_grad_f(x):
##    d = x-center
##    return d / np.sqrt(np.dot(d,d))


def test_f(x):
    if m.isnan(my_interpol_f(x)) == True:
        return 0
    else:
        return float(my_interpol_f(x))


d=0.5 # this parameter decides distance along each axis, over which we compute the gradient.
def test_grad_f(x):
    if x[0]-d >= 0 and x[1] - d >= 0 and x[2] - d >=0 -d and x[0]+d <=f.shape[0] and x[1]+d <=f.shape[1] and x[2]+d <=f.shape[2]:
        grad_x=test_f(np.array([x[0]+d,x[1],x[2]]))-test_f(np.array([x[0]-d,x[1],x[2]]))
        grad_y=test_f(np.array([x[0],x[1]+d,x[2]]))-test_f(np.array([x[0],x[1]-d,x[2]]))
        grad_z=test_f(np.array([x[0],x[1],x[2]+d]))-test_f(np.array([x[0],x[1],x[2]-d]))
    else:
        grad_x=1
        grad_y=1
        grad_z=1

    grad=np.array([grad_x,grad_y,grad_z])
    return grad/np.linalg.norm(grad)
                                                        
                                                                

verts, tris = DC.dual_contour(test_f, test_grad_f, f.shape[0], f.shape[1], f.shape[2]) 


mlab.triangular_mesh( 
            [ v[0] for v in verts ],
            [ v[1] for v in verts ],
            [ v[2] for v in verts ],
            tris)
mlab.savefig('test_result.wrl')
