import numpy as np

import matplotlib.pyplot as plt
import matplotlib
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

fig, ax = plt.subplots()
patches = []
num_polygons = 5
num_sides = 5

# for i in range(num_polygons):
polygon = Polygon([[0,0],[3,1],[3,3]], True)
patches.append(polygon)


p = PatchCollection(polygon, cmap=matplotlib.cm.jet, alpha=0.4)

colors = 100*np.random.rand(len(patches))
p.set_array((1,1,1))

ax.add_collection(p)

plt.show()