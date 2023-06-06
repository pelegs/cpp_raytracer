import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


fig = plt.figure()
ax = fig.add_subplot(projection='3d')

points = np.loadtxt("random_vectors_solid_angle_rotation.data")
xs = points[:,0]
ys = points[:,1]
zs = points[:,2]

ax.scatter(xs, ys, zs)
ax.axes.set_xlim3d([-2, 2])
ax.axes.set_ylim3d([-2, 2])
ax.axes.set_zlim3d([-2, 2])
ax.set_aspect('equal')
u, v, w = 0., 0., 1.0
ax.quiver(0., 0., 0., u, v, w)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

plt.show()
