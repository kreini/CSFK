# import packages

import numpy as np
import pandas as pd
import plotly.express as px

# print vectors

def printv(v):
    print("v =\t", v[0])
    print('\t', v[1])
    print('\t', v[2])
    


# import data

df = pd.read_csv("data.txt", sep = " ")



index = 0

# The position of the satellite relative to the Earth

v_es = [df['ECI_x'][index], df['ECI_y'][index], df['ECI_z'][index]]
printv(v_es)

# The position of the Sun relative to the Earth

v_ss = [df['ECI_sun_x'][index], df['ECI_sun_y'][index], df['ECI_sun_z'][index]]
printv(v_ss)



# angle in radian

def angler(v1, v2):
    v1n = np.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)
    v2n = np.sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2)
    theta = np.arccos(np.dot(v_es, v_ss) / (v1n * v2n))
    print("Theta =\t", theta, "rad")
    return theta
    
# angle in degrees
    
def angled(v1, v2):
    v1n = np.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)
    v2n = np.sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2)
    theta = np.arccos(np.dot(v_es, v_ss) / (v1n * v2n))
    theta = theta * (180 / np.pi)
    print("Theta =\t", theta, "deg")
    return theta

angler(v_ss, v_es)
angled(v_ss, v_es)



# vector to unit vector

def unit(v):
    vn = np.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    vu = v / vn
    printv(vu)
    return vu

unit(v_es)
unit(v_ss)



x = [0, unit(v_es)[0], 0, unit(v_ss)[0]]
y = [0, unit(v_es)[1], 0, unit(v_ss)[1]]
z = [0, unit(v_es)[2], 0, unit(v_ss)[2]]

fig = px.line_3d(
    x = x,
    y = y,
    z = z
)

fig.update_layout(
    scene = {
        "xaxis": {"nticks": 4, "range": [-1, 1]},
        "yaxis": {"nticks": 4, "range": [-1, 1]},        
        "zaxis": {"nticks": 4, "range": [-1, 1]},
        "camera_eye": {"x": 1, "y": 1, "z": 1},
        "aspectratio": {"x": 1, "y": 1, "z": 1}
    }
)

fig.show()
