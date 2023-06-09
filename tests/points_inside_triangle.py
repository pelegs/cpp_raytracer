import matplotlib.pyplot as plt
import numpy as np


fig = plt.figure()

points = np.loadtxt("tests/points_inside_triangle.data")
xs = points[:, 0]
ys = points[:, 1]
cs = points[:, 2]

plt.scatter(xs, ys, c=cs, s=10)
plt.show()
