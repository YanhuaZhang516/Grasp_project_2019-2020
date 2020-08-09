import numpy as np
import math

a=[]
for i in range(10):
    a.append(i)

print(a)
matrix_force = np.eye(3) * 0.001
print(matrix_force)
print(np.shape(matrix_force))

matrix_force2=[[0.001, 0, 0],
               [0, 0.001, 0],
               [0, 0, 0.001]]
print(matrix_force2)
print(np.shape(matrix_force2))


def alignment(force, torque, current_pos, current_orn):
    matrix_force = [[0.001, 0, 0],
                    [0, 0.001, 0],
                    [0, 0, 0.001]]
    matrix_torque = [[0.001, 0, 0],
                     [0, 0.001, 0],
                     [0, 0, 0.001]]

    desired_force = [0, 0, 25]
    desired_torque = [0, 0, 0]
    current_force = force
    current_torque = torque
    delta_pos = np.dot(matrix_force, np.array(desired_force) - np.array(current_force))
    delta_orn = np.dot(matrix_torque, np.array(desired_torque) - np.array(current_torque))

    new_pos = current_pos + delta_pos
    new_orn = current_orn + delta_orn

    print("before pos:", current_pos)
    print("before orn:", current_orn)
    print("new pos:", new_pos)
    print("new orn:", new_orn)

    return new_pos, new_orn

force=[-5, -18, 25 ]
torque = [4, 0.8, -1]
current_pos=[0.54, 0.18, 0.95]
current_orn=[0, math.pi, math.pi]

new_pos, new_orn =alignment(force, torque, current_pos, current_orn)