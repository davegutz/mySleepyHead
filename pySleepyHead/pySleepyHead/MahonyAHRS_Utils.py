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
    # cr = math.cos(roll / 2.)
    # sr = math.sin(roll / 2.)
    # cp = math.cos(pitch / 2.)
    # sp = math.sin(pitch / 2.)
    # cy = math.cos(yaw / 2.)
    # sy = math.sin(yaw / 2.)
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    quat = Qu([ cr*cp*cy + sr*sp*sy,
                sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*cy ])
    # quat = Qu([ cr*cp*sy - sr*sp*cy,
    #             cr*cp*cy + sr*sp*sy,
    #             sr*cp*cy - cr*sp*sy,
    #             cr*sp*cy + sr*cp*sy ])
    return quat

def g_to_euler321(g_vector):
    """
   Converts a g-force vector to Euler 3-2-1 angles (roll, pitch, yaw) using a ZYX sequence.
    Args:
        g_vector: A numpy array representing the g-force vector (g_x, g_y, g_z).
    Returns:
        A numpy array containing the Euler 3-2-1 angles (roll, pitch, yaw) in radians.
    """
    norm_g = np.linalg.norm(g_vector)
    if norm_g > 1e-6:
        g_vector /= norm_g
    else:
        print("norm error in g_to_quaternion")
        exit(1)
    q = g_to_quaternion(g_vector)
    angles_euler321 = quaternion_to_euler321(q)
    rpy = angles_euler321 * 180. /  np.pi
    return angles_euler321

def g_to_quaternion(g_vector):
    """
   Converts a g-force vector to quaternion
    Args:
        g_vector: A numpy array representing the g-force vector (g_x, g_y, g_z).
    Returns:
        quaternion
    """
    norm_g = np.linalg.norm(g_vector)
    if norm_g > 1e-6:
        g_vector /= norm_g
    else:
        print("norm error in g_to_quaternion")
        exit(1)
    gx, gy, gz = g_vector
    cos_theta = 1.0 * gz

    # half_cos = np.sqrt(0.5*(1.0 + cos_theta))
    half_cos = 0.7071*np.sqrt(1.0 + cos_theta)
    q0 = half_cos
    # temp = 1/(2.0*half_cos)
    temp = 0.5 / half_cos
    q1 = gy * temp
    q2 = -gx * temp
    q3 = 0.0
    return Qu([q0, q1, q2, q3])

def pp7(accelerometer, euler321_vec_deg, quat, sample_period=None, label=""):
    # euler321_vec_deg = quaternion_to_euler321(quat) * np.array(180.) / np.pi
    # print(f"pp7 Mahony AHRS {g_vec=} {euler321_vec=} {quat=} {euler321_vec_deg=}")
    print(f"{label} {(sample_period * 100.):.3f};  ", end='')
    print(f"\taccel_raw: [ {accelerometer[0]:6.3f}, {accelerometer[1]:6.3f}, {accelerometer[2]:6.3f} ], g's; ", end='')
    # print(f"\tx_raw*200:"); print(x_raw*200+200, 3);
    # print(f"\ty_raw*200:"); print(y_raw*200+200, 3);
    # print(f"\tz_raw*200:"); print(z_raw*200+200, 3);
    # print(f"\ta_raw*200:"); print(a_raw*200+200, 3);
    # print(f"\tb_raw*200:"); print(b_raw*200+200, 3);
    # print(f"\tc_raw*200:"); print(c_raw*200+200, 3);
    # print(f"\troll_filt:"); print(roll_filt, 3);
    # print(f"\tpitch_filt:"); print(pitch_filt, 3);
    # print(f"\tyaw_filt:"); println(yaw_filt, 3);
    print(f"\teuler321_filt: [ {euler321_vec_deg[0]:6.2f}, {euler321_vec_deg[1]:6.2f}, {euler321_vec_deg[2]:6.2f} ], deg; ", end='')
    # print(f"\tex:"); print(e[0], 3)
    # print(f"\tey:"); print(e[1], 3)
    # print(f"\tez:"); print(e[2], 3)
    print(f"\tquat: [ {quat[0]:6.3f}, {quat[1]:6.3f}, {quat[2]:6.3f}, {quat[3]:6.3f} ]")


def quaternion_to_euler321(quaternion):
    """
    Converts quaternion to Euler 3-2-1 angles
        arg: A quaternion represented as [w, x, y, z].
    Returns:
        list: euler321_angles = [roll, pitch, yaw]
    """
    q0, q1, q2, q3 = quaternion
    roll = np.arctan2(2.*(q0*q1 + q2*q3), 1. - 2.*(q1*q1 + q2*q2))
    sp = -2.0 * (q0*q2 - q1*q3)
    if sp >= 1.0:
        pitch = np.pi / 2.
    elif sp <= -1.0:
        pitch = -np.pi / 2.
    else:
        pitch = np.arcsin(sp)

    yaw = np.arctan2(2.*(q1*q2 + q0*q3), 1. - 2.*(q2*q2 + q3*q3))
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
