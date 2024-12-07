import math
import numpy as np
from pyquaternion import Quaternion as Qu

def euler321_to_g(euler321):
    """
    Converts Euler 3-2-1 angles in radians  to a g.
    Args: euler321_angles = [roll, pitch, yaw], radians

    Returns:
        list: A vector accelerometer represented as [gx, gy, gz]
    """
    q = euler321_to_quaternion(euler321)
    g = quaternion_to_g(q)
    return g

def euler321_to_quaternion(euler321):
    """
    Converts 3-2-1 Euler angles  to a quaternion.
    Args: euler321_angles = [roll, pitch, yaw]

    Returns:
        list: A quaternion represented as [w, x, y, z].
    """
    roll, pitch, yaw = euler321
    cr = math.cos(roll / 2.)
    sr = math.sin(roll / 2.)
    cp = math.cos(pitch / 2.)
    sp = math.sin(pitch / 2.)
    cy = math.cos(yaw / 2.)
    sy = math.sin(yaw / 2.)
    quat = Qu([ cr*cp*cy + sr*sp*sy,
                sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*cy ])
    return quat

def g_to_euler321(g_vector):
    """
   Converts a g-force vector to Euler 3-2-1 angles (roll, pitch, yaw) using a ZYX sequence.
    Args:
        g_vector: A numpy array representing the g-force vector (g_x, g_y, g_z).
    Returns:
        A numpy array containing the Euler 3-2-1 angles (roll, pitch, yaw) in radians.
    """
    fXg = g_vector[0] / 2.
    fYg = g_vector[1] / 2.
    fZg = g_vector[2] / 2.
    roll = math.atan2(fYg, fZg)
    pitch = -math.atan2(fXg, math.sqrt(fYg * fYg + fZg * fZg))
    # yaw = math.atan(fZg / math.sqrt(fXg * fXg + fZg * fZg))
    yaw = 0.
    return np.array([roll, pitch, yaw])

def pp7(accelerometer, quat, sample_period=None, label=""):
    euler321_vec_deg = quaternion_to_euler321(quat) * np.array(180.) / np.pi
    # print(f"pp7 Mahony AHRS {g_vec=} {euler321_vec=} {quat=} {euler321_vec_deg=}")
    print(f"{label} {(sample_period * 100.):.3f}", end='')
    print(f"\tx_raw: {accelerometer[0]:.3f}", end='')
    print(f"\ty_raw: {accelerometer[1]:.3f}", end='')
    print(f"\tz_raw: {accelerometer[2]:.3f}", end='')
    # print(f"\tx_raw*200:"); print(x_raw*200+200, 3);
    # print(f"\ty_raw*200:"); print(y_raw*200+200, 3);
    # print(f"\tz_raw*200:"); print(z_raw*200+200, 3);
    # print(f"\ta_raw*200:"); print(a_raw*200+200, 3);
    # print(f"\tb_raw*200:"); print(b_raw*200+200, 3);
    # print(f"\tc_raw*200:"); print(c_raw*200+200, 3);
    # print(f"\troll_filt:"); print(roll_filt, 3);
    # print(f"\tpitch_filt:"); print(pitch_filt, 3);
    # print(f"\tyaw_filt:"); println(yaw_filt, 3);
    print(f"\troll_filt: {euler321_vec_deg[0]:.3f}", end='')
    print(f"\tpitch_filt: {euler321_vec_deg[1]:.3f}", end='')
    print(f"\tyaw_filt: {euler321_vec_deg[2]:.3f}", end='')
    # print(f"\tex:"); print(e[0], 3)
    # print(f"\tey:"); print(e[1], 3)
    # print(f"\tez:"); print(e[2], 3)
    print(f"\tq0: {quat[0]:.3f}", end='')
    print(f"\tq1: {quat[1]:.3f}", end='')
    print(f"\tq2: {quat[2]:.3f}", end='')
    print(f"\tq3: {quat[3]:.3f}")

def quaternion_to_euler321(quaternion):
    """
    Converts quaternion to Euler 3-2-1 angles
        arg: A quaternion represented as [w, x, y, z].
    Returns:
        list: euler321_angles = [roll, pitch, yaw]
    """
    w, x, y, z = quaternion
    roll = np.arctan2(w*x + y*z, 0.5 - x*x - y*y)
    sp = -2.0 * (x*z - w*y)
    if sp >= 1.0:
        pitch = np.pi / 2.
    elif sp <= -1.0:
        pitch = -np.pi / 2.
    else:
        pitch = np.arcsin(sp)

    yaw = np.arctan2(x*y + w*z, 0.5 - y*y - z*z)
    # yaw = 0.
    return np.array([roll, pitch, yaw])

def quaternion_to_g(q):
    """
    Converts quaternion to Euler 3-2-1 angles
        arg: A quaternion represented as [x, y, z, w].
    Returns:
        g_vector: A numpy array representing the g-force vector (g_x, g_y, g_z)
    """
    gx = 2. * (q[1]*q[3] - q[0]*q[2])
    gy = 2. * (q[0]*q[1] - q[2]*q[3])
    gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]

    return np.array([gx, gy, gz])
