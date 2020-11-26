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
    for object in self.objects:
        currentAxs.add_patch(Rectangle((object[0], object[1]), object[2], object[3], fill=True, alpha=1))

    # Plot all targets
    for target in self.targets:
        plt.plot([target[0]], [target[1]], marker='o', markersize=5, color="red")

    plt.show()

size = [10,11] #[X, Y]
objects =  [[1,1,3,1], 
            [1,3,3,1],
            [1,5,3,1],
            [1,7,3,1],
            [1,9,3,1],
            [6,1,3,1], 
            [6,3,3,1],
            [6,5,3,1],
            [6,7,3,1],
            [6,9,3,1]] # [X, Y, X_size, Y_size]

start = [0.5,0.5]
targets =  [[2.5,4.5],
            [7.5,2.5],
            [4, 8.5]]

ws = Workspace(size, objects, start, targets)
ws.plotWorkspace()