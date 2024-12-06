import math
import numpy as np
from pyquaternion import Quaternion as Qu

def quaternion_to_angles(quaternion):
    """
    Converts quaternion to Euler angles
        arg: A quaternion represented as [x, y, z, w].
    Returns:
        list: euler_angles = [roll, pitch, yaw]
    """
    w, x, y, z = quaternion
    roll = np.arctan2(w*x + y*z, 0.5 - x*x - y*y)
    pitch = np.arcsin(-2.0 * (x*z - w*y))
    # yaw = np.arctan2(x*y + w*z, 0.5 - y*y - z*z)
    yaw = 0.
    return np.array([roll, pitch, yaw])


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

def angles_to_quaternion(angles):
    """
    Converts Euler angles  to a quaternion.
    Args: euler_angles = [roll, pitch, yaw]

    Returns:
        list: A quaternion represented as [w, x, y, z].
    """
    roll, pitch, yaw = angles
    cr = math.cos(roll / 2.)
    sr = math.sin(roll / 2.)
    cp = math.cos(pitch / 2.)
    sp = math.sin(pitch / 2.)
    cy = math.cos(yaw / 2.)
    sy = math.sin(yaw / 2.)
    quat = Qu([0., 0., 0., 0.])
    quat[0] = cr * cp * cy + sr * sp * sy
    quat[1] = sr * cp * cy - cr * sp * sy
    quat[2] = cr * sp * cy + sr * cp * sy
    quat[3] = cr * cp * sy - sr * sp * cy
    return quat


