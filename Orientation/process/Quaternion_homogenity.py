# import packages

import numpy as np
import random
import pandas as pd
import plotly.graph_objs as go

# print vectors

def printv(V):
    print("p =\t", V[0][0])
    print('\t', V[1][0])
    print('\t', V[2][0])



# variables

c = 10000     # number of loops
res = 6     # parameter: number of decimals
l = []      # list of transformed vectors
v = [[2],   # column vector
     [1],
     [3]]
     


# transform v vector c times with a random SO(3) matrix

for i in range (0, c):
    
    # random quaternion generator
    
    u1 = float(random.randint(0, 1000) / 1000)
    u2 = float(random.randint(0, 1000) / 1000)
    u3 = float(random.randint(0, 1000) / 1000)

    qr = np.sqrt(1-u1)*np.sin(2*np.pi*u2)
    qi = np.sqrt(1-u1)*np.cos(2*np.pi*u2)
    qj = np.sqrt(u1)*np.sin(2*np.pi*u3)
    qk = np.sqrt(u1)*np.cos(2*np.pi*u3)
    
    # quaternion to SO(3) matrix
    
    xx = 1 - 2*(qj**2 + qk**2)
    xy = 2*(qi*qj - qk*qr)
    xz = 2*(qi*qk + qj*qr)
    yx = 2*(qi*qj + qk*qr)
    yy = 1 - 2*(qi**2 + qk**2)
    yz = 2*(qj*qk - qi*qr)
    zx = 2*(qi*qk - qj*qr)
    zy = 2*(qj*qk+qi*qr)
    zz = 1 - 2*(qi**2 + qj**2)

    R = [[xx, xy, xz],
         [yx, yy, yz],
         [zx, zy, zz]]
    
    # transform vector with SO(3) matrix
    
    l.append(np.dot(R, v))
    printv(l[i])

# create pandas dataframe 

df = pd.DataFrame(list(map(np.ravel, l)))



# plot vectors

fig = go.Figure(go.Scatter3d(
    x = df[0],
    y = df[1],
    z = df[2],
    
    mode = 'markers',
    
    marker = dict(
        #color = 'rgb(128, 128, 128)',
        size = 1,
        symbol = 'circle'
    )
))

fig.update_layout(
    scene = {
        "xaxis": {"nticks": 4, "range": [-5, 5]},
        "yaxis": {"nticks": 4, "range": [-5, 5]},
        "zaxis": {"nticks": 4, "range": [-5, 5]},
        "aspectratio": {"x": 1, "y": 1, "z": 1}
    }
)

fig.show()
