from time import sleep
import numpy as np
import PyKDL
import dvrk

def rot_matrix(yaw, pitch, roll):
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(yaw), -np.sin(yaw)],
        [0, np.sin(yaw), np.cos(yaw)]
    ])

    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    R_z = np.array([
        [np.cos(roll), -np.sin(roll), 0],
        [np.sin(roll), np.cos(roll), 0],
        [0, 0, 1]
    ])

    # combine rotations by using matrix multiplication
    return R_x.dot(R_y.dot(R_z))


def np2kdl(arr):
    return PyKDL.Rotation(arr[0, 0], arr[0, 1], arr[0, 2],
                          arr[1, 0], arr[1, 1], arr[1, 2],
                          arr[2, 0], arr[2, 1], arr[2, 2])


def kdl2np(r):
    # Stupid workaround
    return np.array(eval(str(r).replace(';', ','))).reshape(3, 3)


if __name__ == "__main__":
    ROT_MATRIX = PyKDL.Rotation(
        1.,    0.,    0.,
        0.,   -1.,    0.,
        0.,    0.,   -1.
    )
    # Angle of plane to x-axis
    plane_x = 0
    # Angle of plane to y-axis
    plane_y = 0
    pitch = np.pi # 15 degrees
    
    arm = dvrk.arm("PSM3")
    print("Starting home")
    arm.home()
    goal = np.copy(arm.get_current_joint_position())
    goal[0] = 0.
    goal[1] = 0.
    goal[2] = .12
    arm.move_joint(goal)
    arm.move(ROT_MATRIX)
    current_rot = arm.get_current_position().M
    offset_rot = rot_matrix(0, np.pi/4, 0)
    current_rot_arr = kdl2np(current_rot)
    new_rot = offset_rot.dot(current_rot_arr)
    arm.move(np2kdl(new_rot))