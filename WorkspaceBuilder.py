import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

class Workspace:
  def __init__(self, size, objects, start, targets):
        self.size = np.array(size)
    self.objects = np.array(objects)
    self.start = np.array(start)
    self.targets = np.array(targets)
    print(f"Workspace has {self.targets.size} targets and {self.objects.size} objects")

  def plotWorkspace(self):
    plt.figure()
    # Set border and plot size 
    currentAxs = plt.gca()
    currentAxs.add_patch(Rectangle((0, 0), self.size[0], self.size[1], fill=None, alpha=1))
    plt.xlim(-self.size[0]/10, self.size[0]*1.1)
    plt.ylim(-self.size[1]/10, self.size[1]*1.1)
    plt.axis('equal')
    # plt.grid()

    # Plot starting point
    plt.plot([self.start[0]], [self.start[1]], marker='o', markersize=5, color="green")

    # Plot all the objects
    for (x, y, size_x, size_y) in self.objects:
        currentAxs.add_patch(Rectangle((x, y), size_x, size_y, fill=True, alpha=1))
        self.fillSquares(x, y, size_x, size_y)
        
    for (x, y, size_x, size_y) in self.objects:
        currentAxs.add_patch(Rectangle((x, y), size_xx+0.2, size_yx+0.2, fill=True, alpha=0.5, color='r'))
        self.fillSquares(x, y, size_x, size_y)
        

    # Plot all targets
    for target in self.targets:
        plt.plot([target[0]], [target[1]], marker='o', markersize=5, color="red")

    plt.show()

  def fillSquares(self, x, y, size_x, size_y):
    plt.plot([x, y], marker='o', markersize=size_x, color="red")

