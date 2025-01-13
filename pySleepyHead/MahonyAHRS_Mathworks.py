import numpy as np
from pyquaternion import Quaternion as Qu
from MahonyAHRS_Utils import euler321_to_quaternion, quaternion_to_euler321, g_to_euler321


class MahonyAHRS_MW:
    """#MAYHONYAHRS Madgwick's implementation of Mayhony's AHRS algorithm
    #
    #   For more information see:
    #   http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    #
    #   Date          Author          Notes
    #   28/09/2011    SOH Madgwick    Initial release
    """
    ## Public properties
    def __init__(self, sample_period=None, quaternion=None, kp=None, ki=None):
        if sample_period is not None:
            self.sample_period = 1./256.
        else:
            self.sample_period = sample_period
        if quaternion is None:
            self.quat = Qu([1., 0., 0., 0.])  # output quaternion describing the Earth relative to the sensor
        else:
            self.quat = quaternion
        if kp is None:
            self.Kp = 1.  # algorithm proportional gain
        else:
            self.Kp = kp
        if ki is None:
            self.Ki = 0.  # algorithm integral gain
        else:
            self.Ki = ki
        self.integralFB_ = np.zeros(3)  # integral error
        self.e = np.zeros(3) # error
        self.roll_ = 0.
        self.pitch_ = 0.
        self.yaw_ = 0.
        self.accel_vec = np.zeros(3)  # saved g-load inputs
        self.gyroscope = np.zeros(3)  # saved gyro rate inputs
        self.euler321_vec_check_deg = np.zeros(3)
        self.euler321_vec_deg = np.zeros(3)
        self.accel_vec = np.zeros(3)  # integral error
        self.label = "pp7 Mathwo AHRS"

    def update_imu(self, accelerometer, gyroscope, sample_time, reset):
        # Check inputs
        accelerometer_norm = np.linalg.norm(accelerometer)
        if accelerometer_norm < 1.e-4:
            print("norm 0 in MW update_imu")
            return # handle NaN
        self.accel_vec = accelerometer / accelerometer_norm
        gyro_norm = np.linalg.norm(gyroscope)
        if gyro_norm > 1.e-4:
            self.gyroscope = gyroscope / gyro_norm
        self.sample_period = sample_time

        if reset:
            euler321_vec = g_to_euler321(self.accel_vec)
            self.quat = euler321_to_quaternion(euler321_vec)
            self.euler321_vec_check_deg = np.array(quaternion_to_euler321(self.quat)) * 180. / np.pi
            self.euler321_vec_deg = euler321_vec * 180. / np.pi

        q = self.quat # short name local variable for readability

        # Estimated direction of gravity and magnetic flux
        v = np.array([  2*(q[1]*q[3] - q[0]*q[2]),
                        2*(q[0]*q[1] + q[2]*q[3]),
                        q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]  ])

        # Error is sum of cross product between estimated direction and measured direction of field
        self.e = np.linalg.cross(self.accel_vec, v)  # error
        if self.Ki > 0:
            self.integralFB_ += self.e * self.sample_period
        else:
            self.integralFB_ = np.zeros(3)

        # Apply feedback terms
        if not reset:
            self.gyroscope += self.Kp * self.e + self.Ki * self.integralFB_

        # Compute rate of change of quaternion
        qDot = Qu(0.5) * q * Qu([ 0., self.gyroscope[0], self.gyroscope[1], self.gyroscope[2] ])

        # Integrate to yield quaternion
        if not reset:
            q += qDot * self.sample_period
        q_norm = q.norm
        if q_norm > 1.e-4:
            self.quat = q / q_norm # normalise quaternion
        self.euler321_vec_deg = quaternion_to_euler321(self.quat) * 180. / np.pi
