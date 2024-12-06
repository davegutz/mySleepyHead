import math
import numpy as np
from pyquaternion import Quaternion as Qu

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

def pp7(accelerometer, quat, sample_period=None, label=""):
    angles_vec_deg = quaternion_to_angles(quat) * np.array(180.) / np.pi
    # print(f"pp7 Mahony AHRS {g_vec=} {angles_vec=} {quat=} {angles_vec_deg=}")
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
    print(f"\troll_filt: {angles_vec_deg[0]:.3f}", end='')
    print(f"\tpitch_filt: {angles_vec_deg[1]:.3f}", end='')
    print(f"\tyaw_filt: {angles_vec_deg[2]:.3f}", end='')
    # print(f"\tex:"); print(e[0], 3)
    # print(f"\tey:"); print(e[1], 3)
    # print(f"\tez:"); print(e[2], 3)
    print(f"\tq0: {quat[0]:.3f}", end='')
    print(f"\tq1: {quat[1]:.3f}", end='')
    print(f"\tq2: {quat[2]:.3f}", end='')
    print(f"\tq3: {quat[3]:.3f}")

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
