import numpy as np
from numpy.ma.core import angle
from pyquaternion import Quaternion as Qu
from MahonyAHRS_Mathworks import MahonyAHRS_MW
from MahonyAHRS_Utils import euler321_to_quaternion, quaternion_to_euler321, g_to_euler321, pp7, \
    quaternion_to_g, ppv3, ppv4


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

    def getPitch(self):
        return self.pitch_

    def getRoll(self):
        return self.roll_

    def getYaw(self):
        return self.yaw_

    def pp8(self):
            # euler321_vec_deg = quaternion_to_euler321(quat) * np.array(180.) / np.pi
            # print(f"pp7 Mahony AHRS {g_vec=} {euler321_vec=} {quat=} {euler321_vec_deg=}")
            print(f"  pp8: {(self.sample_period * 100.):.3f};  ", end='')
            print(f"\taccel_raw: [ {self.accel_vec[0]:6.3f}, {self.accel_vec[1]:6.3f}, {self.accel_vec[2]:6.3f} ], g's; ", end='')
            print(f"\thalfe: [ {self.halfe[0]:6.3f}, {self.halfe[1]:6.3f}, {self.halfe[2]:6.3f} ],", end='')
            print(f"\tquat: [ {self.quat[0]:6.3f}, {self.quat[1]:6.3f}, {self.quat[2]:6.3f}, {self.quat[3]:6.3f} ]")

    def updateIMU(self, accelerometer, gyroscope, sample_time, reset):
        q = self.quat # short name local variable for readability
        gyroscope *= 0.0174533
        # Normalise accelerometer measurement
        norm_acc = np.linalg.norm(accelerometer)
        if norm_acc == 0:
            return  # handle NaN
        self.accel_vec = accelerometer / norm_acc
        # print(f"enter:  {self.accel_vec=} exit: ", end='')
        self.acc_x_ = self.accel_vec[0]
        self.acc_y_ = self.accel_vec[1]
        self.acc_z_ = self.accel_vec[2]

        if sample_time is not None:
            self.sample_period = sample_time

        if not ( self.acc_x_==0. and self.acc_y_==0. and self.acc_z_==0. ):
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
            self.halfe = np.linalg.cross(self.accel_vec, self.halfv)
            self.halfex_ = (self.acc_y_*self.halfvz_ - self.acc_z_*self.halfvy_)
            self.halfey_ = (self.acc_z_*self.halfvx_ - self.acc_x_*self.halfvz_)
            self.halfez_ = (self.acc_x_*self.halfvy_ - self.acc_y_*self.halfvx_)

            if self.Ki > 0 and not reset:
                self.integralFB_ += self.Ki * 2. * self.halfe * self.sample_period
                gyroscope += self.integralFB_
                self.gyr_x_ = gyroscope[0]
                self.gyr_y_ = gyroscope[1]
                self.gyr_z_ = gyroscope[2]
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

        self.euler321_vec = quaternion_to_euler321(self.quat)
        self.euler321_vec_deg = self.euler321_vec * 180. / np.pi
        # print(f"{self.accel_vec=}")


def main():
    quat = None
    accel_vec = None
    euler321_angles = None
    sample_time = 0.1
    input_type = 2

    if input_type == 0:
        # https://www.andre-gaschler.com/rotationconverter/
        # quat = [0.84971050, 0.37282170, 0.37282170,  0.00000000]  # ZYX angles = (45, 45,  0) accel_vec = (0.536,     0.756,     0.375)
        quat = [ 0.854,  0.354, -0.354,  0.146 ]  # ZYX angles = (45, -45,  0) accel_vec = (0.536,     0.756,     0.375)
        # quat = [0.92387950, 0.38268340, 0.00000000,  0.00000000]  # ZYX angles = (45,  0,  0) accel_vec = (0.0000000, 0.7070000, 0.70700 ) check
        # quat = [0.92387950, 0.00000000, 0.38268340,  0.00000000]  # ZYX angles = ( 0, 45,  0) accel_vec = (0.7070000, 0.0000000, 0.70700 )
        # quat = [0.70710680, 0.00000000,-0.70710676,  0.00000000]  # ZYX angles = ( 0, 90, 0) accel = (-1,    0,     0)
        # quat = [0.70710680, 0.70710676, 0.00000000,  0.00000000]  # ZYX angles = (90,  0, 0) accel = (0,     1,     0)
        # quat = [1.00000000, 0.00000000, 0.00000000,  0.00000000]  # ZYX angles = ( 0,  0, 0) accel = (0,     0    , 1)
        euler321_angles = quaternion_to_euler321(quat)
        accel_vec = quaternion_to_g(quat)


    elif input_type == 1:
        # accel_vec = np.array([ 1.0000000, 0.0000000, 0.00000 ])   # ZYX angles = (180., -90, 180.)
        # accel_vec = np.array([-1.0000000, 0.0000000, 0.00000 ])   # ZYX angles = (180.,  90, 180.)
        # accel_vec = np.array([ 0.5360000, 0.7560000, 0.37500 ])   # ZYX angles = ( 45.,  45.,  0.)
        accel_vec = np.array([ 0.632,  0.632,  0.447 ])     # ZYX angles = ( 45.,  -45.,  0.)
        # accel_vec = np.array([ 0.5360000, 0.7560000, 0.37500 ])   # ZYX angles = ( 45.,  45.,  0.)
        # accel_vec = np.array([ 0.0000000, 0.7070000, 0.70700 ])   # ZYX angles = ( 45.,  0.0,  0.)  check
        # accel_vec = np.array([ 0.7070000, 0.0000000, 0.70700 ])   # ZYX angles = (  0.,  45.,  0.)  check
        euler321_angles = g_to_euler321(accel_vec)
        quat = euler321_to_quaternion(euler321_angles)
        acc_check = quaternion_to_g(quat)
        print("input_type = 1, accel_vec is input")
        ppv3(label='accel_vec:', vec=accel_vec)
        ppv3(label='euler_angles deg:', vec=euler321_angles*180./np.pi)
        ppv4(label='quat:      ', vec=quat)
        print("")
        ang_check = quaternion_to_euler321(quat)
        acc_check = quaternion_to_g(quat)
        ppv3(label='quat->acc:', vec=acc_check)
        ppv3(label='quat->ang deg:   ', vec=ang_check*180./np.pi)
        print('\t\t\t\t\t""""""\t\t\t\t\t\t\t')
        print("")

    elif input_type == 2:
        # euler321_angles_deg = np.array([ 0., -45., 0.])  # check
        # euler321_angles_deg = np.array([ 45.,  0., 0.])  # check
        # euler321_angles_deg = np.array([ 0.,  45., 0.])  # check
        euler321_angles_deg = np.array([ 45., -45., 0.])  # check  angles check1 bad
        # euler321_angles_deg = np.array([ 45., 45., 0.])  # check
        # euler321_angles_deg = np.array([ 0.,  0., 0.])  # check
        # euler321_angles_deg = np.array([ 0.,  -89.99999, 0.])  # check
        # euler321_angles_deg = np.array([ 5., -45., 0.])  # check  angles check1 bad
        # euler321_angles_deg = np.array([ 45., -5., 0.])  # check  angles check1 bad
        euler321_angles = euler321_angles_deg * np.pi / 180.
        quat = euler321_to_quaternion(euler321_angles)   # check
        accel_vec = quaternion_to_g(quat)  # totally fucked for multi rotations
        print("input_type = 2, euler321_angles is input")
        ppv3(label='euler_angles deg:', vec=euler321_angles*180./np.pi)
        ppv4(label='quat:      ', vec=quat)
        ppv3(label='accel_vec:', vec=accel_vec)
        print("")
        ang_check = g_to_euler321(accel_vec)
        quat_check = euler321_to_quaternion(g_to_euler321(accel_vec))
        ppv3(label='acc->ang deg:    ', vec=ang_check*180./np.pi)
        ppv4(label='acc->quat: ', vec=quat_check)
        print('\t\t\t\t\t""""""\t\t\t\t\t\t\t')
        ang_check = quaternion_to_euler321(quat)
        acc_check = quaternion_to_g(quat)
        ppv3(label='quat->ang deg:   ', vec=ang_check*180./np.pi)
        print('\t\t\t\t\t""""""\t\t\t\t\t\t\t', end='')
        ppv3(label='quat->acc:', vec=acc_check)
        print("")
        print("")

    track_filter = MahonyAHRS(sample_period=0.1, kp=5., ki=8.)  # Kp=5, Ki=8
    track_filter_mathworks = MahonyAHRS_MW(sample_period=0.1, kp=10., ki=1.)

    # Local steady state check
    print('Local steady state check:')
    euler321_vec_ss = g_to_euler321(accel_vec)
    quat_ss = euler321_to_quaternion(euler321_vec_ss)
    pp7(accel_vec / np.linalg.norm(accel_vec), euler321_vec_ss*180./np.pi, quat_ss, sample_period=sample_time, label="pp7 ss    AHRS ")
    print("")

    # Iterative init
    print('iterative initial values:')
    err_tf = 100.
    count = 0
    init = True
    gyro_vec = np.zeros(3)
    track_filter.updateIMU(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time=0.1, reset=init)
    init = False
    while err_tf > 1e-3 and count < 100:
        gyro_vec = np.zeros(3)
        track_filter.updateIMU(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time=0.1, reset=init)
        # pp7(track_filter.accel_vec, track_filter.euler321_vec_deg, track_filter.quat, sample_period=track_filter.sample_period, label=track_filter.label)
        err_tf = np.linalg.norm(abs(track_filter.halfe))
        err_tfmw = np.linalg.norm(abs(track_filter_mathworks.e))
        err = err_tf + err_tfmw
        count += 1
    if count >= 100:
        print(f"init iteration timed out, err = {err}, {err_tf=}, {err_tfmw=}")
    pp7(track_filter.accel_vec, track_filter.euler321_vec_deg, track_filter.quat, sample_period=track_filter.sample_period, label=track_filter.label)
    print("")

    print('Transient')
    err_tf = 100.
    count = 0
    init = True
    accel_vec = np.array([0., 0., 1.])
    gyro_vec = np.zeros(3)
    track_filter.updateIMU(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time=0.1, reset=init)
    init = False
    while err_tf > 1e-3 and count < 100:
        gyro_vec = np.zeros(3)
        track_filter.updateIMU(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time=0.1, reset=init)
        # pp7(track_filter.accel_vec, track_filter.euler321_vec_deg, track_filter.quat, sample_period=track_filter.sample_period, label=track_filter.label)
        err_tf = np.linalg.norm(abs(track_filter.halfe))
        err_tfmw = np.linalg.norm(abs(track_filter_mathworks.e))
        err = err_tf + err_tfmw
        count += 1
    if count >= 100:
        print(f"init iteration timed out, err = {err}, {err_tf=}, {err_tfmw=}")
    pp7(track_filter.accel_vec, track_filter.euler321_vec_deg, track_filter.quat, sample_period=track_filter.sample_period, label=track_filter.label)
    init = False
    # Loop
    num_its = 100
    for i in range(num_its):
        if i == 4:
            accel_vec = np.zeros(3) + np.array([0., 0., 1.])
            accel_vec += np.array([-.707, .707, -.707])
            accel_vec /= np.linalg.norm(accel_vec)
        if i == 24:
            accel_vec = np.zeros(3) + np.array([0., 0., 1.])
        gyro_vec = np.zeros(3)
        track_filter.updateIMU(accelerometer=accel_vec, gyroscope=gyro_vec, sample_time = 0.1, reset=init)
        print(f"{i:3d}\t", end='')
        # track_filter.pp8()
        pp7(track_filter.accel_vec, track_filter.euler321_vec_deg, track_filter.quat, sample_period=track_filter.sample_period, label=track_filter.label)

    print("initial value for reference")
    pp7(accel_vec, euler321_angles_deg, quat, sample_period=.1, label="prep           ")
    # pp7(accel_vec, euler321_vec_ss*180./np.pi, quat_ss, sample_period=sample_time, label="pp7 local AHRS ")


# import cProfile
# if __name__ == '__main__':
#     cProfile.run('main()')
#

if __name__ == '__main__':
    main()
