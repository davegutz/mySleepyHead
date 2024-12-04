import math
import numpy as np


def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles (roll, pitch, yaw) to a quaternion.
    Args:
        roll (float): Rotation around the x-axis in radians.
        pitch (float): Rotation around the y-axis in radians.
        yaw (float): Rotation around the z-axis in radians.

    Returns:
        list: A quaternion represented as [w, x, y, z].
    """

    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, y, z]


def g_to_euler(g_vector):
    """
   Converts a g-force vector to Euler angles (roll, pitch, yaw) using a ZYX sequence.
    Args:
        g_vector: A numpy array representing the g-force vector (g_x, g_y, g_z).
    Returns:
        A numpy array containing the Euler angles (roll, pitch, yaw) in radians.
    """
    g_x, g_y, g_z = g_vector

    # Calculate rotation matrix from g-force
    R = np.array([
        [g_x / np.linalg.norm(g_vector), g_y / np.linalg.norm(g_vector), g_z / np.linalg.norm(g_vector)],
        [0, 1, 0],
        [0, 0, 1]
    ])

    # Extract Euler angles from rotation matrix (ZYX sequence)
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = np.arcsin(-R[2, 0])
    yaw = np.arctan2(R[1, 0], R[0, 0])

    return np.array([roll, pitch, yaw])


def main():


# import cProfile
# if __name__ == '__main__':
#     cProfile.run('main()')
#

if __name__ == '__main__':
    main()
