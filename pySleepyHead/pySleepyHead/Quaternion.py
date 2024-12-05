import math
import numpy as np


def angles_to_quaternion(euler_angles):
    """
    Converts Euler angles  to a quaternion.
    Args: euler_angles = [roll, pitch, yaw]

    Returns:
        list: A quaternion represented as [w, x, y, z].
    """

    roll, pitch, yaw = euler_angles

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

def g_to_angles(g_vector):
    """
   Converts a g-force vector to Euler angles (roll, pitch, yaw) using a ZYX sequence.
    Args:
        g_vector: A numpy array representing the g-force vector (g_x, g_y, g_z).
    Returns:
        A numpy array containing the Euler angles (roll, pitch, yaw) in radians.
    """
    g_norm = np.linalg.norm(g_vector)
    g_vector_norm = g_vector / g_norm
    g_x, g_y, g_z = g_vector_norm

    # Extract Euler angles from rotation matrix (ZYX sequence)
    roll =  np.arctan2(g_y, g_z )
    pitch = -np.arctan2(g_x, g_z )
    yaw = 0.

    return np.array([roll, pitch, yaw])

def quaternion_to_angles(quaternion):
    """
    Converts quaternion to Euler angles
        arg: A quaternion represented as [x, y, z, w].

    Returns:
        list: euler_angles = [roll, pitch, yaw]
    """
    w, x, y, z = quaternion

    roll = np.arctan2(w*x + y*z, 0.5 - x*x - y*y);
    pitch = np.arcsin(-2.0 * (x*z - w*y))
    # yaw = np.arctan2(x*y + w*z, 0.5 - y*y - z*z);
    yaw = 0.;

    return np.array([roll, pitch, yaw])

def main():
    g_vec = np.array([1, 1, 1])
    angles_vec = g_to_angles(g_vec)
    quat = angles_to_quaternion(angles_vec)
    angles_vec_check_deg = quaternion_to_angles(quat) * 180. / np.pi
    angles_vec_deg = angles_vec * 180. / np.pi
    print(f"{g_vec=} {angles_vec=} {quat=} \n{angles_vec_deg=} \n{angles_vec_check_deg=}")


# import cProfile
# if __name__ == '__main__':
#     cProfile.run('main()')
#

if __name__ == '__main__':
    main()
