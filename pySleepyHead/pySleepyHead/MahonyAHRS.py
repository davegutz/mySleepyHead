import numpy as np
from pyquaternion import Quaternion as Qu
from MahonyAHRS_Mathworks import MahonyAHRS_MW
from MahonyAHRS_Utils import euler321_to_quaternion, quaternion_to_euler321, g_to_euler321, pp7, euler321_to_g, \
    g_to_quaternion, quaternion_to_g


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
        self.integralFB_ = np.zeros(3)  # integral error
        self.euler321_vec = None
        self.euler321_vec_check_deg = None
        self.euler321_vec_deg = None
        self.accel_vec = np.zeros(3)
        self.halfv = 0.
        self.halfe = 0.
        self.halfex_ = 0.
        self.halfey_ = 0.
        self.halfez_ = 0.
        self.halfvx_ = 0.
        self.halfvy_ = 0.
        self.halfvz_ = 0.
        self.euler321_computed = True
        self.roll_ = 0.
        self.pitch_ = 0.
        self.yaw_ = 0.
        self.gyr_x_ = 0.
        self.gyr_y_ = 0.
        self.gyr_z_ = 0.
        self.acc_x_ = 0.
        self.acc_y_ = 0.
        self.acc_z_= 0.
        self.label = "pp7 Mahony AHRS"

    def update_imu(self, accelerometer, gyroscope, sample_time, reset):
        q = self.quat # short name local variable for readability
        gyroscope *= 0.0174533
        self.gyr_x_ = gyroscope[0]
        self.gyr_y_ = gyroscope[1]
        self.gyr_z_ = gyroscope[2]
        self.acc_x_ = accelerometer[0]
        self.acc_y_ = accelerometer[1]
        self.acc_z_ = accelerometer[2]
        self.accel_vec = [self.acc_x_, self.acc_y_, self.acc_z_]
        self.accel_vec /= np.linalg.norm(self.accel_vec)
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
                self.euler321_vec = g_to_euler321(g_vec)
                self.quat = euler321_to_quaternion(self.euler321_vec)

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

            if self.Ki > 0 and not reset:
                self.integralFB_ += self.Ki * 2. * self.halfe * self.sample_period
                gyroscope += self.integralFB_
            else:
                self.integralFB_ = np.zeros(3)

            # Apply proportional feedback
            self.gyr_x_ += 2. * self.Kp * self.halfex_
            self.gyr_y_ += 2. * self.Kp * self.halfey_
            self.gyr_z_ += 2. * self.Kp * self.halfez_

        # Integrate quaternion
        if not reset:
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

        self.euler321_vec_check_deg = np.array(quaternion_to_euler321(self.quat)) * 180. / np.pi
        self.euler321_vec_deg = self.euler321_vec * 180. / np.pi
        self.euler321_computed = False
        
    # def update(self, gyroscope, accelerometer, magnetometer):
    #     q = self.quat # short name local variable for readability
    #
    #     # Normalise accelerometer measurement
    #     if np.linalg.norm(accelerometer) == 0:
    #         return  # handle NaN
    #     accelerometer /= np.linalg.norm(accelerometer)    # normalize magnitude
    #
    #     # Normalise magnetometer measurement
    #     if np.linalg.norm(magnetometer) == 0:
    #         return  # handle NaN
    #     magnetometer /= np.linalg.norm(magnetometer)   # normalise magnitude
    #
    #     # Reference direction of Earth's magnetic field
    #     h = q * Qu([0., magnetometer]) * q.conjugate
    #     norm_short_h = np.linalg.norm(h[1], h[2])
    #     b = Qu([0., norm_short_h, 0., h[3]])
    #
    #     # Estimated direction of gravity and magnetic field
    #     v = np.array([  2*(q[1]*q[3] - q[0]*q[2]),
    #                     2*(q[0]*q[1] + q[2]*q[3]),
    #                     q[0]^2 - q[1]^2 - q[2]^2 + q[3]^2  ])
    #     w = np.array([  2*b[1]*(0.5 - q[2]^2 - q[3]^2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]),
    #                     2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]),
    #                     2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]^2 - q[2]^2) ])
    #
    #     # Error is sum of cross product between estimated direction and measured direction of fields
    #     e = np.linalg.cross(accelerometer, v) + np.linalg.cross(magnetometer, w)
    #     if self.Ki > 0:
    #         self.integralFB_ = self.integralFB_ + e * self.sample_period
    #     else:
    #         self.integralFB_ = np.zeros(3)
    #
    #     # Apply feedback terms
    #     gyroscope = gyroscope + self.Kp * e + self.Ki * self.integralFB_
    #
    #     # Compute rate of change of quaternion
    #     qDot = 0.5 * q * Qu([0., gyroscope(1), gyroscope(2), gyroscope(3)])
    #
    #     # Integrate to yield quaternion
    #     q = q + qDot * self.sample_period
    #     self.quat = q / np.linalg.norm(q) # normalise quaternion


def main():
    sample_time = 0.1
    # https://www.andre-gaschler.com/rotationconverter/
    q = [0.84971050, 0.37282170, 0.37282170,  0.00000000]  # ZYX angles = (45, 45,  0) accel_vec = (0.536,     0.756,     0.375)
    q = [0.92387950, 0.38268340, 0.00000000,  0.00000000]  # ZYX angles = (45,  0,  0) accel_vec = (0.0000000, 0.7070000, 0.70700 ) check
    q = [0.92387950, 0.00000000, 0.38268340,  0.00000000]  # ZYX angles = ( 0, 45,  0) accel_vec = (0.7070000, 0.0000000, 0.70700 )
    # q = [0.70710680, 0.00000000,-0.70710676,  0.00000000]  # ZYX angles = ( 0, 90, 0) accel = (-1,    0,     0)
    # q = [0.70710680, 0.70710676, 0.00000000,  0.00000000]  # ZYX angles = (90,  0, 0) accel = (0,     1,     0)
    # q = [1.00000000, 0.00000000, 0.00000000,  0.00000000]  # ZYX angles = ( 0,  0, 0) accel = (0,     0    , 1)
    euler321_angles = quaternion_to_euler321(q)
    euler321_angles_deg = euler321_angles * 180. / np.pi
    accel_vec = euler321_to_g(euler321_angles)


    # accel_vec = np.array([ 1.0000000, 0.0000000, 0.00000 ])   # ZYX angles = (180., -90, 180.)
    # accel_vec = np.array([-1.0000000, 0.0000000, 0.00000 ])   # ZYX angles = (180.,  90, 180.)
    # accel_vec = np.array([ 0.5360000, 0.7560000, 0.37500 ])   # ZYX angles = ( 45.,  45.,  0.)
    # accel_vec = np.array([ 0.0000000, 0.7070000, 0.70700 ])   # ZYX angles = ( 45.,  0.0,  0.)  check
    # accel_vec = np.array([ 0.7070000, 0.0000000, 0.70700 ])   # ZYX angles = (  0.,  45.,  0.)  check
    euler321_angles = g_to_euler321(accel_vec)

    euler321_angles_deg = np.array([ 45., -45., 0.])
    # euler321_angles_deg = np.array([ 45.,  0., 0.])
    # euler321_angles_deg = np.array([ 0.,  0., 0.])  # check
    # euler321_angles_deg = np.array([ 0.,  -89.99999, 0.])  #
    euler321_angles = euler321_angles_deg * np.pi / 180.
    q = euler321_to_quaternion(euler321_angles)
    accel_vec = quaternion_to_g(q)

    print("Initial values")
    pp7(accel_vec, euler321_angles_deg, q, sample_period=.1, label="prep           ")


    gyro_vec = np.zeros(3)
    track_filter = MahonyAHRS(sample_period=0.1, kp=10., ki=1.)
    track_filter_mathworks = MahonyAHRS_MW(sample_period=0.1, kp=10., ki=1.)
    init = True
    track_filter.update_imu(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time=0.1, reset=init)
    pp7(track_filter.accel_vec, track_filter.euler321_vec_deg, track_filter.quat, sample_period=track_filter.sample_period, label=track_filter.label)
    track_filter_mathworks.update_imu(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time=0.1, reset=init)
    pp7(track_filter_mathworks.accel_vec, track_filter_mathworks.euler321_vec_deg, track_filter_mathworks.quat, sample_period=track_filter_mathworks.sample_period, label=track_filter_mathworks.label)

    # Local steady state check
    euler321_vec_ss = g_to_euler321(accel_vec)
    quat_ss = euler321_to_quaternion(euler321_vec_ss)
    pp7(accel_vec / np.linalg.norm(accel_vec), euler321_vec_ss, quat_ss, sample_period=sample_time, label="pp7 ss    AHRS ")
    print("")

    # euler321_vec_check_deg = np.array(quaternion_to_euler321(quat)) * 180. / np.pi
    # euler321_vec_deg = euler321_vec * 180. / np.pi

    for i in range(3):
        track_filter.update_imu(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time = 0.1, reset=init)
        pp7(track_filter.accel_vec, track_filter.euler321_vec_deg, track_filter.quat, sample_period=track_filter.sample_period, label=track_filter.label)
        track_filter_mathworks.update_imu(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time = 0.1, reset=init)
        pp7(track_filter_mathworks.accel_vec, track_filter_mathworks.euler321_vec_deg, track_filter_mathworks.quat, sample_period=track_filter_mathworks.sample_period, label=track_filter_mathworks.label)
        print("")
        init = False

    print("initial value for reference")
    pp7(accel_vec, euler321_angles_deg, q, sample_period=.1, label="prep           ")
    # pp7(accel_vec, euler321_vec_ss*180./np.pi, quat_ss, sample_period=sample_time, label="pp7 local AHRS ")


# import cProfile
# if __name__ == '__main__':
#     cProfile.run('main()')
#

if __name__ == '__main__':
    main()
