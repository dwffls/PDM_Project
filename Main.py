from WorkspaceBuilder import Workspace

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