import numpy as np
from pyquaternion import Quaternion as Qu
from MahonyAHRS_Utils import angles_to_quaternion, quaternion_to_angles, g_to_angles


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
        self.integralFB_ = np.array([0., 0., 0.])  # integral error
        self.e = np.array([0., 0., 0.]) # error
        self.roll_ = 0.
        self.pitch_ = 0.
        self.yaw_ = 0.
        self.accelerometer = np.array([0., 0., 0.])  # saved g-load inputs
        self.gyroscope = np.array([0., 0., 0.])  # saved gyro rate inputs
        self.angles_vec_check_deg = np.array([0., 0., 0.])
        self.angles_vec_deg = np.array([0., 0., 0.])


    def update(self, gyroscope, accelerometer, magnetometer):
        q = self.quat # short name local variable for readability

        # Normalise accelerometer measurement
        if np.linalg.norm(accelerometer) == 0:
            return  # handle NaN
        accelerometer = accelerometer / np.linalg.norm(accelerometer)    # normalise magnitude

        # Normalise magnetometer measurement
        if np.linalg.norm(magnetometer) == 0:
            return  # handle NaN
        magnetometer = magnetometer / np.linalg.norm(magnetometer)   # normalise magnitude

        # Reference direction of Earth's magnetic field
        h = q * Qu([0., magnetometer]) * q.conjugate
        norm_short_h = np.linalg.norm(h[1], h[2])
        b = Qu([0., norm_short_h, 0., h[3]])

        # Estimated direction of gravity and magnetic field
        v = np.array([  2*(q[1]*q[3] - q[0]*q[2]),
                        2*(q[0]*q[1] + q[2]*q[3]),
                        q[0]^2 - q[1]^2 - q[2]^2 + q[3]^2  ])
        w = np.array([  2*b[1]*(0.5 - q[2]^2 - q[3]^2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]),
                        2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]),
                        2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]^2 - q[2]^2) ])

        # Error is sum of cross product between estimated direction and measured direction of fields
        self.e = np.linalg.cross(accelerometer, v) + np.linalg.cross(magnetometer, w)
        if self.Ki > 0:
            self.integralFB_ = self.integralFB_ + self.e * self.sample_period
        else:
            self.integralFB_ = np.array([0., 0., 0.])

        # Apply feedback terms
        gyroscope = gyroscope + self.Kp * self.e + self.Ki * self.integralFB_

        # Compute rate of change of quaternion
        qDot = 0.5 * q * Qu([0., gyroscope(1), gyroscope(2), gyroscope(3)])

        # Integrate to yield quaternion
        q = q + qDot * self.sample_period
        self.quat = q / np.linalg.norm(q) # normalise quaternion

    def update_imu(self, gyroscope, accelerometer, sample_time, reset):
        self.accelerometer = accelerometer / np.linalg.norm(accelerometer)
        self.gyroscope = gyroscope / np.linalg.norm(gyroscope)
        self.sample_period = sample_time

        # Check valid input
        if np.linalg.norm(self.accelerometer) == 0:
            return # handle NaN

        if reset:
            angles_vec = g_to_angles(accelerometer)
            self.quat = angles_to_quaternion(angles_vec)
            self.angles_vec_check_deg = np.array(quaternion_to_angles(self.quat)) * 180. / np.pi
            self.angles_vec_deg = angles_vec * 180. / np.pi

        q = self.quat # short name local variable for readability

        # Estimated direction of gravity and magnetic flux
        v = np.array([  2*(q[1]*q[3] - q[0]*q[2]),
                        2*(q[0]*q[1] + q[2]*q[3]),
                        q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]  ])

        # Error is sum of cross product between estimated direction and measured direction of field
        self.e = np.linalg.cross(self.accelerometer, v)  # error
        if self.Ki > 0:
            self.integralFB_ += self.Ki * self.e * self.sample_period
        else:
            self.integralFB_ = np.array([0., 0., 0.])

        # Apply feedback terms
        self.gyroscope += self.Kp * self.e + self.Ki * self.integralFB_

        # Compute rate of change of quaternion
        qDot = Qu(0.5) * q * Qu([ 0., self.gyroscope[0], self.gyroscope[1], self.gyroscope[2] ])

        # Integrate to yield quaternion
        q += qDot * self.sample_period
        self.quat = q / q.norm # normalise quaternion
        self.angles_vec_deg = quaternion_to_angles(self.quat)
        
    def pp7(self):
        self.angles_vec_deg = quaternion_to_angles(self.quat) * np.array(180.) / np.pi
        # print(f"pp7 Mahony AHRS {g_vec=} {angles_vec=} {quat=} {self.angles_vec_deg=}")
        print(f"pp7 mathwo AHRS T_acc*100: {(self.sample_period*100.):.3g}", end='')
        print(f"\tx_raw: {self.accelerometer[0]:.3g}", end='')
        print(f"\ty_raw: {self.accelerometer[1]:.3g}", end='')
        print(f"\tz_raw: {self.accelerometer[2]:.3g}", end='')
        # print(f"\tx_raw*200:"); print(x_raw*200+200, 3);
        # print(f"\ty_raw*200:"); print(y_raw*200+200, 3);
        #print(f"\tz_raw*200:"); print(z_raw*200+200, 3);
        # print(f"\ta_raw*200:"); print(a_raw*200+200, 3);
        # print(f"\tb_raw*200:"); print(b_raw*200+200, 3);
        # print(f"\tc_raw*200:"); print(c_raw*200+200, 3);
        # print(f"\troll_filt:"); print(roll_filt, 3);
        # print(f"\tpitch_filt:"); print(pitch_filt, 3);
        # print(f"\tyaw_filt:"); println(yaw_filt, 3);
        print(f"\troll_filt: {self.angles_vec_deg[0]:.3g}", end='')
        print(f"\tpitch_filt: {self.angles_vec_deg[1]:.3g}", end='')
        print(f"\tyaw_filt: {self.angles_vec_deg[2]:.3g}", end='')
        # print(f"\tex:"); print(self.e[0], 3)
        # print(f"\tey:"); print(self.e[1], 3)
        # print(f"\tez:"); print(self.e[2], 3)
        print(f"\tq0: {self.quat[0]:.3g}", end='')
        print(f"\tq1: {self.quat[1]:.3g}", end='')
        print(f"\tq2: {self.quat[2]:.3g}", end='')
        print(f"\tq3: {self.quat[3]:.3g}")
