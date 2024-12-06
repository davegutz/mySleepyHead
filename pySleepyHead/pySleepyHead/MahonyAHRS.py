
import numpy as np
from pyquaternion import Quaternion as Qu

from MahonyAHRS_Mathworks import MahonyAHRS_MW, angles_to_quaternion, quaternion_to_angles, g_to_angles

def pp7(accelerometer, quat, sample_period=None ):
    angles_vec_deg = quaternion_to_angles(quat) * np.array(180.) / np.pi
    # print(f"pp7 Mahony AHRS {g_vec=} {angles_vec=} {quat=} {angles_vec_deg=}")
    print(f"pp7 local  AHRS T_acc*100: {(sample_period * 100.):.3g}", end='')
    print(f"\tx_raw: {accelerometer[0]:.3g}", end='')
    print(f"\ty_raw: {accelerometer[1]:.3g}", end='')
    print(f"\tz_raw: {accelerometer[2]:.3g}", end='')
    # print(f"\tx_raw*200:"); print(x_raw*200+200, 3);
    # print(f"\ty_raw*200:"); print(y_raw*200+200, 3);
    # print(f"\tz_raw*200:"); print(z_raw*200+200, 3);
    # print(f"\ta_raw*200:"); print(a_raw*200+200, 3);
    # print(f"\tb_raw*200:"); print(b_raw*200+200, 3);
    # print(f"\tc_raw*200:"); print(c_raw*200+200, 3);
    # print(f"\troll_filt:"); print(roll_filt, 3);
    # print(f"\tpitch_filt:"); print(pitch_filt, 3);
    # print(f"\tyaw_filt:"); println(yaw_filt, 3);
    print(f"\troll_filt: {angles_vec_deg[0]:.3g}", end='')
    print(f"\tpitch_filt: {angles_vec_deg[1]:.3g}", end='')
    print(f"\tyaw_filt: {angles_vec_deg[2]:.3g}", end='')
    # print(f"\tex:"); print(e[0], 3)
    # print(f"\tey:"); print(e[1], 3)
    # print(f"\tez:"); print(e[2], 3)
    print(f"\tq0: {quat[0]:.3g}", end='')
    print(f"\tq1: {quat[1]:.3g}", end='')
    print(f"\tq2: {quat[2]:.3g}", end='')
    print(f"\tq3: {quat[3]:.3g}")

class MahonyAHRS:
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
        self.angles_vec_check_deg = None
        self.angles_vec_deg = None
        self.halfv = 0.
        self.halfe = 0.
        self.halfex_ = 0.
        self.halfey_ = 0.
        self.halfez_ = 0.
        self.halfvx_ = 0.
        self.halfvy_ = 0.
        self.halfvz_ = 0.
        self.angles_computed = True
        self.roll_ = 0.
        self.pitch_ = 0.
        self.yaw_ = 0.
        self.gyr_x_ = 0.
        self.gyr_y_ = 0.
        self.gyr_z_ = 0.
        self.acc_x_ = 0.
        self.acc_y_ = 0.
        self.acc_z_= 0.

    def update(self, gyroscope, accelerometer, magnetometer):
        q = self.quat # short name local variable for readability

        # Normalise accelerometer measurement
        if np.linalg.norm(accelerometer) == 0:
            return  # handle NaN
        accelerometer /= np.linalg.norm(accelerometer)    # normalize magnitude

        # Normalise magnetometer measurement
        if np.linalg.norm(magnetometer) == 0:
            return  # handle NaN
        magnetometer /= np.linalg.norm(magnetometer)   # normalise magnitude

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
        e = np.linalg.cross(accelerometer, v) + np.linalg.cross(magnetometer, w)
        if self.Ki > 0:
            self.integralFB_ = self.integralFB_ + e * self.sample_period
        else:
            self.integralFB_ = np.array([0., 0., 0.])

        # Apply feedback terms
        gyroscope = gyroscope + self.Kp * e + self.Ki * self.integralFB_

        # Compute rate of change of quaternion
        qDot = 0.5 * q * Qu([0., gyroscope(1), gyroscope(2), gyroscope(3)])

        # Integrate to yield quaternion
        q = q + qDot * self.sample_period
        self.quat = q / np.linalg.norm(q) # normalise quaternion

    def update_imu(self, gyroscope, accelerometer, sample_time, reset):
        q = self.quat # short name local variable for readability
        gyroscope *= 0.0174533
        self.gyr_x_ = gyroscope[0]
        self.gyr_y_ = gyroscope[1]
        self.gyr_z_ = gyroscope[2]
        self.acc_x_ = accelerometer[0]
        self.acc_y_ = accelerometer[1]
        self.acc_z_ = accelerometer[2]
        if sample_time is not None:
            self.sample_period = sample_time

        if not ( self.acc_x_==0. and self.acc_y_==0. and self.acc_z_==0. ):
            # Normalise accelerometer measurement
            if np.linalg.norm(accelerometer) == 0:
                return # handle NaN
            recipNorm = 1. / np.linalg.norm(accelerometer)
            self.acc_x_ *= recipNorm
            self.acc_y_ *= recipNorm
            self.acc_z_ *= recipNorm

            if reset:
                g_vec = np.array([self.acc_x_, self.acc_y_, self.acc_z_])
                angles_vec = g_to_angles(g_vec)
                self.quat = angles_to_quaternion(angles_vec)
                self.angles_vec_check_deg = np.array(quaternion_to_angles(self.quat)) * 180. / np.pi
                self.angles_computed = True
                self.angles_vec_deg = angles_vec * 180. / np.pi

            q = self.quat # short name local variable for readability

            # Estimated direction of gravity and magnetic flux
            self.halfv = np.array([ 2*(q[1]*q[3] - q[0]*q[2]),
                                    2*(q[0]*q[1] + q[2]*q[3]),
                                    # q[0]^2 - q[1]^2 - q[2]^2 + q[3]^2  ])
                                    q[0]*q[0] + q[3]*q[3] - 0.5  ])
            self.halfvx_ = self.halfv[0]
            self.halfvy_ = self.halfv[1]
            self.halfvz_ = self.halfv[2]

            # Error is sum of cross product between estimated direction and measured direction of field
            self.halfe = np.linalg.cross(accelerometer, self.halfv)
            self.halfex_ = (self.acc_y_*self.halfvz_ - self.acc_z_*self.halfvy_)
            self.halfey_ = (self.acc_z_*self.halfvx_ - self.acc_x_*self.halfvz_)
            self.halfez_ = (self.acc_x_*self.halfvy_ - self.acc_y_*self.halfvx_)

            if self.Ki > 0:
                self.integralFB_ += self.Ki * 2. * self.halfe * self.sample_period
                gyroscope += self.integralFB_
            else:
                self.integralFB_ = np.array([0., 0., 0.])

            # Apply proportional feedback
            self.gyr_x_ += 2. * self.Kp * self.halfex_
            self.gyr_y_ += 2. * self.Kp * self.halfey_
            self.gyr_z_ += 2. * self.Kp * self.halfez_

        # Integrate quaternion
        self.gyr_x_ *= 0.5 * self.sample_period
        self.gyr_y_ *= 0.5 * self.sample_period
        self.gyr_z_ *= 0.5 * self.sample_period
        qa = q[0]
        qb = q[1]
        qc = q[2]
        q[0] += (-qb * self.gyr_x_ - qc * self.gyr_y_ - q[3] * self.gyr_z_)
        q[1] += ( qa * self.gyr_x_ + qc * self.gyr_z_ - q[3] * self.gyr_y_)
        q[2] += ( qa * self.gyr_y_ - qb * self.gyr_z_ + q[3] * self.gyr_x_)
        q[3] += ( qa * self.gyr_z_ + qb * self.gyr_y_ - qc * self.gyr_x_)
        recipNorm = 1. / np.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        q[0] *= recipNorm
        q[1] *= recipNorm
        q[2] *= recipNorm
        q[3] *= recipNorm
        self.angles_computed = False
        
    def pp7(self):
        # g_vec = np.array([self.acc_x_, self.acc_y_, self.acc_z_])
        # angles_vec = np.array([self.roll_, self.pitch_, self.yaw_])
        # quat = np.array([ self.quat[0], self.quat[1], self.quat[2], self.quat[3] ])
        self.angles_vec_deg = quaternion_to_angles(self.quat) * np.array(180.) / np.pi
        # print(f"pp7 Mahony AHRS {g_vec=} {angles_vec=} {quat=} {self.angles_vec_deg=}")
        print(f"pp7 Mahony AHRS T_acc*100: {(self.sample_period*100.):.3g}", end='')
        print(f"\tx_raw: {self.acc_x_:.3g}", end='')
        print(f"\ty_raw: {self.acc_y_:.3g}", end='')
        print(f"\tz_raw: {self.acc_z_:.3g}", end='')
        # print(f"\tx_raw*200:"); print(x_raw*200+200, 3);
        # print(f"\ty_raw*200:"); print(y_raw*200+200, 3);
        #print(f"\tz_raw*200:"); print(z_raw*200+200, 3);
        # print(f"\ta_raw*200:"); print(a_raw*200+200, 3);
        # print(f"\tb_raw*200:"); print(b_raw*200+200, 3);
        # print(f"\tc_raw*200:"); print(c_raw*200+200, 3);
        # print(f"\troll_filt:"); print(roll_filt, 3);
        # print(f"\tpitch_filt:"); print(pitch_filt, 3);
        # print(f"\tyaw_filt:"); println(yaw_filt, 3);
        print(f"\troll_filt: {self.roll_:.3g}", end='')
        print(f"\tpitch_filt: {self.pitch_:.3g}", end='')
        print(f"\tyaw_filt: {self.yaw_:.3g}", end='')
        # print(f"\thalfex:"); print(track_filter->getHalfex(), 3);
        # print(f"\thalfey:"); print(track_filter->getHalfey(), 3);
        # print(f"\thalfez:"); println(track_filter->getHalfez(), 3);
        print(f"\tq0: {self.quat[0]:.3g}", end='')
        print(f"\tq1: {self.quat[1]:.3g}", end='')
        print(f"\tq2: {self.quat[2]:.3g}", end='')
        print(f"\tq3: {self.quat[3]:.3g}")


def main():
    sample_time = 0.1
    accel_vec = np.array([1., 1., 1.])
    gyro_vec = np.array([0., 0., 0.])
    track_filter = MahonyAHRS(sample_period=0.1, kp=10., ki=1.)
    track_filter_mathworks = MahonyAHRS_MW(sample_period=0.1, kp=10., ki=1.)
    init = True
    track_filter.update_imu(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time=0.1, reset=init)
    track_filter.pp7()
    track_filter_mathworks.update_imu(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time=0.1, reset=init)
    track_filter_mathworks.pp7()

    angles_vec = g_to_angles(accel_vec)
    quat = angles_to_quaternion(angles_vec)
    angles_vec_check_deg = np.array(quaternion_to_angles(quat)) * 180. / np.pi
    angles_vec_deg = angles_vec * 180. / np.pi
    pp7(accel_vec, quat, sample_period=sample_time)
    print(f"steady state check {accel_vec=} {angles_vec=} {quat=} \n\t{angles_vec_deg=} \n\t{angles_vec_check_deg=}")

    for i in range(5):
        track_filter.update_imu(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time = 0.1, reset=init)
        track_filter.pp7()
        init = False

# import cProfile
# if __name__ == '__main__':
#     cProfile.run('main()')
#

if __name__ == '__main__':
    main()
