import numpy as np

theta = np.radians(45)
Plane_Pos = [6.35,0,2.18]       # [X_W, Y_W, Z_W]
Offset = [0,0,+0.028]                # [t_x, t_y, n_p]

R_PW = np.array([
            [ np.cos(theta), 0, np.sin(theta)],
            [     0,         1,       0      ],
            [-np.sin(theta), 0, np.cos(theta)]
        ])




Pos_new = Plane_Pos + R_PW.dot(Offset)
print(np.round(Pos_new,3))