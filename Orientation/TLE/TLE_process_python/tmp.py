import plotly.graph_objs as go

x = [0, 1]
y = [0, 2]
z = [0, 3]

fig = go.Figure(go.Scatter3d(
    x = x,
    y = y,
    z = z,
    
    mode = 'lines'
))

fig.update_layout(
    scene = {
        "xaxis": {"nticks": 4, "range": [-1, 1]},
        "yaxis": {"nticks": 4, "range": [-1, 1]},        
        "zaxis": {"nticks": 4, "range": [-1, 1]},
        "camera_eye": {"x": 1, "y": 0.8, "z": 0.8},
        "aspectratio": {"x": 1, "y": 1, "z": 1}
    }
)

fig.show()
