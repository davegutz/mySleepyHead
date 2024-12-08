import math
import numpy as np
from pyquaternion import Quaternion as Qu

# def euler321_to_g(euler321):
#     """
#     Converts Euler 3-2-1 angles in radians  to a g.
#     Args: euler321_angles = [roll, pitch, yaw], radians
#
#     Returns:
#         list: A vector accelerometer represented as [gx, gy, gz]
#     """
#     q = euler321_to_quaternion(euler321)
#     g = quaternion_to_g(q)
#     return g

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
    norm_g = np.linalg.norm(g_vector)
    if norm_g > 1e-6:
        g_vector /= norm_g
    else:
        print("norm error in g_to_quaternion")
        exit(1)
    gx, gy, gz = g_vector
    roll = np.arctan2(gy, np.sqrt(gx*gx + gz*gz))
    pitch = np.arctan2(-gx, gz)
    yaw = 0.
    angles_euler321 = np.array([ roll, pitch, yaw ])
    return angles_euler321

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

def ppv3(vec=None, label=''):
    print(f"\t{label} [ {vec[0]:6.3f}, {vec[1]:6.3f}, {vec[2]:6.3f} ]; ", end='')

def ppv4(vec=None, label=''):
    print(f"\t{label} [ {vec[0]:6.3f}, {vec[1]:6.3f}, {vec[2]:6.3f}, {vec[3]:6.3f} ]; ", end='')

def quaternion_to_euler321(quaternion):
    """
    Converts quaternion to Euler 3-2-1 angles in downward facing frame
        arg: A quaternion represented as [w, x, y, z].
    Returns:
        list: euler321_angles = [roll, pitch, yaw]
    """
    q0, q1, q2, q3 = quaternion
    roll = np.arctan2( (q0*q1 + q2*q3), 0.5 - 1.*(q1*q1 + q2*q2))
    sp = -2.0 * (q0*q2 - q1*q3)
    if sp >= 1.0:
        pitch = np.pi / 2.
    elif sp <= -1.0:
        pitch = -np.pi / 2.
    else:
        pitch = np.arcsin(sp)
    pitch *= -1

    yaw = np.arctan2((q1*q2 + q0*q3), 0.5 - 1.*(q2*q2 + q3*q3))
    # yaw = 0.
    return np.array([roll, pitch, yaw])

def quaternion_to_g(q):
    """
    Converts quaternion to measurement direction
        arg: A quaternion represented as [x, y, z, w].
    Returns:
        g_vector: A numpy array representing the g-force vector (g_x, g_y, g_z)
    """
    gx = 2. * (q[1]*q[3] - q[0]*q[2])
    gy = 2. * (q[0]*q[1] - q[2]*q[3])
    gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]
    # gx = q[1]*q[3] - q[0]*q[2]
    # gy = q[0]*q[1] - q[2]*q[3]
    # gz = q[0]*q[0]   -0.5                    + q[3]*q[3]
    g_vec = np.array([ gx, gy, gz])
    norm_g = np.linalg.norm(g_vec)
    if norm_g > 1e-6:
        g_vec /= norm_g
    else:
        print("ERROR in quaternion_to_g, 0 norm")
        return None  # handle NaN

    return g_vec
