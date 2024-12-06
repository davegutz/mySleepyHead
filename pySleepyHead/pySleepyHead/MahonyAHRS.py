import numpy as np
from pyquaternion import Quaternion as Qu
from MahonyAHRS_Mathworks import MahonyAHRS_MW
from MahonyAHRS_Utils import angles_to_quaternion, quaternion_to_angles, g_to_angles, pp7


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
        self.angles_vec = None
        self.angles_vec_check_deg = None
        self.angles_vec_deg = None
        self.accel_vec = np.array([0., 0., 0.])
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
        self.label = "pp7 Mahony AHRS"

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
                self.angles_vec = g_to_angles(g_vec)
                self.quat = angles_to_quaternion(self.angles_vec)

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
        self.angles_vec_check_deg = np.array(quaternion_to_angles(self.quat)) * 180. / np.pi
        self.angles_vec_deg = self.angles_vec * 180. / np.pi
        self.angles_computed = False
        

def main():
    sample_time = 0.1
    accel_vec = np.array([1., 1., 1.])
    gyro_vec = np.array([0., 0., 0.])
    track_filter = MahonyAHRS(sample_period=0.1, kp=10., ki=1.)
    track_filter_mathworks = MahonyAHRS_MW(sample_period=0.1, kp=10., ki=1.)
    init = True
    track_filter.update_imu(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time=0.1, reset=init)
    pp7(track_filter.accel_vec, track_filter.quat, sample_period=track_filter.sample_period, label=track_filter.label)
    track_filter_mathworks.update_imu(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time=0.1, reset=init)
    pp7(track_filter_mathworks.accel_vec, track_filter_mathworks.quat, sample_period=track_filter_mathworks.sample_period, label=track_filter_mathworks.label)

    # Local steady state check
    angles_vec = g_to_angles(accel_vec)
    quat = angles_to_quaternion(angles_vec)
    angles_vec_check_deg = np.array(quaternion_to_angles(quat)) * 180. / np.pi
    angles_vec_deg = angles_vec * 180. / np.pi
    pp7(accel_vec / np.linalg.norm(accel_vec), quat, sample_period=sample_time, label="pp7 local AHRS ")
    print("")

    for i in range(20):
        track_filter.update_imu(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time = 0.1, reset=init)
        pp7(track_filter.accel_vec, track_filter.quat, sample_period=track_filter.sample_period, label=track_filter.label)
        track_filter_mathworks.update_imu(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time = 0.1, reset=init)
        pp7(track_filter_mathworks.accel_vec, track_filter_mathworks.quat, sample_period=track_filter_mathworks.sample_period, label=track_filter_mathworks.label)
        print("")
        init = False

    pp7(accel_vec, quat, sample_period=sample_time, label="pp7 local AHRS ")


# import cProfile
# if __name__ == '__main__':
#     cProfile.run('main()')
#

if __name__ == '__main__':
    main()
