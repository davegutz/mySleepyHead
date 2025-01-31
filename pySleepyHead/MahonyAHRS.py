import numpy as np
from numpy.ma.core import angle
from pyquaternion import Quaternion as Qu
from MahonyAHRS_Utils import euler321_to_quaternion, quaternion_to_euler321, g_to_euler321, pp7, \
    quaternion_to_g, ppv3, ppv4


class Device:
    NOMINAL_DT = 0.1
    deg_to_rps = 0.0174533

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
    def __init__(self, data=None, sample_period=None, quaternion=None, Kp=None, Ki=None):
        self.Data = data
        self.reset = None
        if sample_period is None:
            self.sample_period = 1./256.
        else:
            self.sample_period = sample_period
        if quaternion is None:
            self.quat = Qu([1., 0., 0., 0.])  # output quaternion describing the Earth relative to the sensor
        else:
            self.quat = quaternion
        if Kp is None:
            self.Kp = 1.  # algorithm proportional gain
        else:
            self.Kp = Kp
        if Ki is None:
            self.Ki = 0.  # algorithm integral gain
        else:
            self.Ki = Ki
        self.integralFB_ = np.zeros(3)  # integral error
        self.euler321_vec = None
        self.euler321_vec_check_deg = None
        self.euler321_vec_deg = None
        self.accel_vec = np.zeros(3)
        self.gyr_vec = np.zeros(3)
        self.halfv = 0.
        self.halfe = 0.
        self.halfex_ = 0.
        self.halfey_ = 0.
        self.halfez_ = 0.
        self.halfvx_ = 0.
        self.halfvy_ = 0.
        self.halfvz_ = 0.
        self.gyr_x_ = 0.
        self.gyr_y_ = 0.
        self.gyr_z_ = 0.
        self.acc_x_ = 0.
        self.acc_y_ = 0.
        self.acc_z_= 0.
        self.ifb_x = 0.
        self.ifb_y = 0.
        self.ifb_z = 0.
        self.q0 = 0.
        self.q1 = 0.
        self.q2 = 0.
        self.q3 = 0.
        self.roll_rad = 0.
        self.pitch_rad = 0.
        self.yaw_rad = 0.
        self.roll_deg = 0.
        self.pitch_deg = 0.
        self.yaw_deg = 0.
        self.label = "pp7 Mahony AHRS"
        self.time = 0.
        self.a_raw = 0.
        self.b_raw = 0.
        self.c_raw = 0.
        self.x_raw = 0.
        self.y_raw = 0.
        self.z_raw = 0.
        self.saved = Saved()  # for plots and prints

    def __repr__(self):
        s = "MahonyAHRS:\n"
        s += "  reset =  {:2d}".format(self.reset)
        s += "  T   =  {:5.1f}".format(self.T)
        s += "  Ki = {:5.1f} Kp = {:5.1f}".format(self.Ki, self.Kp)
        s += "  [x_raw, y_, z_] =  [ {:5.3f}, {:5.3f}, {:5.3f} ]".format(self.acc_x_, self.acc_y_, self.acc_z_)
        s += "  [a_raw, b_, c_] =  [ {:5.3f}, {:5.3f}, {:5.3f} ]".format(self.gyr_x_, self.gyr_y_, self.gyr_z_)
        s += "  [halfex, y, z]  =  [ {:5.3f}, {:5.3f}, {:5.3f} ]".format(self.halfex_, self.halfey_, self.halfez_)
        s += "  [halfvx, y, z]  =  [ {:5.3f}, {:5.3f}, {:5.3f} ]".format(self.halfvx_, self.halfvy_, self.halfvz_)
        s += "  [ifb_x, _y, _z] =  [ {:5.3f}, {:5.3f}, {:5.3f} ]".format(self.ifb_x, self.ifb_y, self.ifb_z)
        s += "  [q0, q1, q2, q3]=  [ {:5.3f}, {:5.3f}, {:5.3f}, {:5.3f} ]".format(self.q0, self.q1, self.q2, self.q3)
        s += "  [roll, pitch, yaw]=[ {:7.5f}, {:7.5f}, {:7.5f} ]".format(self.roll_rad, self.pitch_rad, self.yaw_rad)
        s += "  [roll, pitch, yaw]=[ {:5.3f}, {:5.3f}, {:5.3f} ]".format(self.roll_deg, self.pitch_deg, self.yaw_deg)
        return s

    def __str__(self, prefix=''):
        s = prefix + "MahonyAHRS:\n"
        s += "  reset =  {:2d}  // s\n".format(self.reset)
        s += "  T   =  {:5.1f}  // s\n".format(self.T)
        s += "  Ki = {:5.1f} Kp = {:5.1f}  // s\n".format(self.Ki, self.Kp)
        s += "  [x_raw, y_, z_] =  [ {:5.3f}, {:5.3f}, {:5.3f} ] // g's\n".format(self.acc_x_, self.acc_y_, self.acc_z_)
        s += "  [a_raw, b_, c_] =  [ {:5.3f}, {:5.3f}, {:5.3f} ] // dps\n".format(self.gyr_x_, self.gyr_y_, self.gyr_z_)
        s += "  [halfex, y, z]  =  [ {:5.3f}, {:5.3f}, {:5.3f} ] // ?\n".format(self.halfex_, self.halfey_, self.halfez_)
        s += "  [halfvx, y, z]  =  [ {:5.3f}, {:5.3f}, {:5.3f} ] // ?\n".format(self.halfvx_, self.halfvy_, self.halfvz_)
        s += "  [ifb_x, _y, _z] =  [ {:5.3f}, {:5.3f}, {:5.3f} ] // ?\n".format(self.ifb_x, self.ifb_y, self.ifb_z)
        s += "  [q0, q1, q2, q3]=  [ {:5.3f}, {:5.3f}, {:5.3f}, {:5.3f} ] // ?\n".format(self.q0, self.q1, self.q2, self.q3)
        s += "  [roll, pitch, yaw]=[ {:7.5f}, {:7.5f}, {:7.5f} ] // rad\n".format(self.roll_rad, self.pitch_rad, self.yaw_rad)
        s += "  [roll, pitch, yaw]=[ {:5.3f}, {:5.3f}, {:5.3f} ] // deg\n".format(self.roll_deg, self.pitch_deg, self.yaw_deg)
        return s

    def calculate(self, init_time=-4., verbose=True, t_max=None):
        """Filter data set and calculate candidate filter"""
        t = self.Data.time
        if t_max is not None:
            t_delt = t - t[0]
            t = t[np.where(t_delt <= t_max)]
        t_len = len(t)

        # time loop
        now = t[0]
        for i in range(t_len):
            now = t[i]
            self.reset = (t[i] <= init_time) or (t[i] < 0. and t[0] > init_time)
            self.Data.i = i
            self.time = now

            # Inputs
            self.a_raw = self.Data.a_raw[i]
            self.b_raw = self.Data.b_raw[i]
            self.c_raw = self.Data.c_raw[i]
            self.x_raw = self.Data.x_raw[i]
            self.y_raw = self.Data.y_raw[i]
            self.z_raw = self.Data.z_raw[i]
            gyroscope = np.array([ self.a_raw, self.b_raw, self.c_raw ])
            accelerometer = np.array([ self.x_raw, self.y_raw, self.z_raw ])
            self.Ki = self.Data.twoKi[i] / 2.
            self.Kp = self.Data.twoKp[i] / 2.
            if self.reset:
                self.quat = Qu(self.Data.q0[i], self.Data.q1[i], self.Data.q2[i], self.Data.q3[i])

            # Update time
            self.T = None
            if i == 0:
                self.T = t[1] - t[0]
            else:
                candidate_dt = t[i] - t[i - 1]
                if candidate_dt > 1e-6:
                    self.T = candidate_dt
                else:
                    self.T = Device.NOMINAL_DT

            # Run filters
            self.updateIMU(gyroscope_=gyroscope, accelerometer=accelerometer, sample_time=self.T, reset=self.reset)

            # Log
            self.save(t[i], self.T)

            # Print initial
            if i == 0 and verbose:
                print('time=', t[i])
            if verbose:
                self.__repr__()

            # print(f"{i=} {t[i]}")
            # print(f"GYR: old : [ {self.Data.a_raw[i]}, {self.Data.b_raw[i]}, {self.Data.c_raw[i]} ]  ver: [ {self.a_raw}, {self.b_raw}, {self.c_raw} ] "
            #       f"ACC: old : [ {self.Data.x_raw[i]}, {self.Data.y_raw[i]}, {self.Data.z_raw[i]} ]  ver: [ {self.x_raw}, {self.y_raw}, {self.z_raw} ]  ")
            print(f"HALFV: old : [ {self.Data.halfvx[i]}, {self.Data.halfvy[i]}, {self.Data.halfvz[i]} ]  ver: [ {self.halfvx_}, {self.halfvy_}, {self.halfvz_} ]  end=''")
            # print(f"HALFE: old : [ {self.Data.halfex[i]}, {self.Data.halfey[i]}, {self.Data.halfez[i]} ]  ver: [ {self.halfex_}, {self.halfey_}, {self.halfez_} ]")
            # print(f"quat: old : [ {self.Data.q0[i]}, {self.Data.q1[i]}, {self.Data.q2[i]}, {self.Data.q3[i]} ]  ver: [ {self.quat[0]}, {self.quat[1]}, {self.quat[2]}, {self.quat[3]} ] ")

        # Data
        if verbose:
            print('Sensors:  ', str(self.__str__()))

        return self.saved

    def getPitchDeg(self):
        return self.pitch_deg

    def getRollDeg(self):
        return self.roll_deg

    def getYawDeg(self):
        return self.yaw_deg

    def pp8(self):
            # euler321_vec_deg = quaternion_to_euler321(quat) * np.array(180.) / np.pi
            # print(f"pp7 Mahony AHRS {g_vec=} {euler321_vec=} {quat=} {euler321_vec_deg=}")
            print(f"  pp8: {(self.sample_period * 100.):.3f};  ", end='')
            print(f"\taccel_raw: [ {self.accel_vec[0]:6.3f}, {self.accel_vec[1]:6.3f}, {self.accel_vec[2]:6.3f} ], g's; ", end='')
            print(f"\trot_raw: [ {self.gyr_vec[0]:6.4f}, {self.gyr_vec[1]:6.4f}, {self.gyr_vec[2]:6.4f} ], rps; ", end='')
            print(f"\thalfe: [ {self.halfe[0]:6.3f}, {self.halfe[1]:6.3f}, {self.halfe[2]:6.3f} ],", end='')
            print(f"\tquat: [ {self.quat[0]:6.3f}, {self.quat[1]:6.3f}, {self.quat[2]:6.3f}, {self.quat[3]:6.3f} ]")

    def save(self, time, dt):  # Filter
        """Log Sensors"""
        self.saved.reset.append(self.reset)
        self.saved.time.append(time)
        self.saved.T.append(dt)
        self.saved.a_raw.append(self.a_raw)
        self.saved.b_raw.append(self.b_raw)
        self.saved.c_raw.append(self.c_raw)
        self.saved.x_raw.append(self.x_raw)
        self.saved.y_raw.append(self.y_raw)
        self.saved.z_raw.append(self.z_raw)
        self.saved.halfex.append(self.halfex_)
        self.saved.halfey.append(self.halfey_)
        self.saved.halfez.append(self.halfez_)
        self.saved.halfvx.append(self.halfvx_)
        self.saved.halfvy.append(self.halfvy_)
        self.saved.halfvz.append(self.halfvz_)
        self.saved.ifb_x.append(self.ifb_x)
        self.saved.ifb_y.append(self.ifb_y)
        self.saved.ifb_z.append(self.ifb_z)
        self.saved.q0.append(self.quat[0])
        self.saved.q1.append(self.quat[1])
        self.saved.q2.append(self.quat[2])
        self.saved.q3.append(self.quat[3])
        self.saved.roll_rad.append(self.roll_rad)
        self.saved.pitch_rad.append(self.pitch_rad)
        self.saved.yaw_rad.append(self.yaw_rad)
        self.saved.roll_deg.append(self.roll_deg)
        self.saved.pitch_deg.append(self.pitch_deg)
        self.saved.yaw_deg.append(self.yaw_deg)

    def updateIMU(self, gyroscope_, accelerometer, sample_time, reset):
        # print(f"{gyroscope_=} {accelerometer=} {sample_time=} {reset=}")

        q = self.quat # short name local variable for readability

        gyroscope = gyroscope_ * Device.deg_to_rps

        # Normalize
        norm_acc = np.linalg.norm(accelerometer)
        if norm_acc == 0:
            return  # handle NaN
        self.accel_vec = accelerometer / norm_acc
        norm_gyr = np.linalg.norm(gyroscope)
        if norm_gyr == 0:
            return  # handle NaN
        self.gyr_vec = gyroscope / norm_gyr
        # print(f"{self.gyr_vec=} {self.accel_vec=} {sample_time=} {reset=}")

        self.acc_x_, self.acc_y_, self.acc_z_ = self.accel_vec

        if sample_time is not None:
            self.sample_period = sample_time

        if not ( self.acc_x_==0. and self.acc_y_==0. and self.acc_z_==0. ):
            if reset:
                g_vec = np.array([self.acc_x_, self.acc_y_, self.acc_z_])
                self.euler321_vec = g_to_euler321(g_vec)
                self.quat = euler321_to_quaternion(self.euler321_vec)
                # short name local variable for readability
                q = np.float64( [self.quat[0], self.quat[1], self.quat[2], self.quat[3] ]).copy()

            # Estimated direction of gravity
            self.halfv = np.array([ q[1]*q[3] - q[0]*q[2],
                                    q[0]*q[1] + q[2]*q[3],
                                    # q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]  ])
                                    q[0]*q[0] + q[3]*q[3] - 0.5  ])
            self.halfvx_, self.halfvy_, self.halfvz_ = self.halfv

            # Error is sum of cross product between estimated direction and measured direction of field
            # self.halfe = np.linalg.cross(self.accel_vec, self.halfv)
            self.halfex_ = (self.acc_y_*self.halfvz_ - self.acc_z_*self.halfvy_)
            self.halfey_ = (self.acc_z_*self.halfvx_ - self.acc_x_*self.halfvz_)
            self.halfez_ = (self.acc_x_*self.halfvy_ - self.acc_y_*self.halfvx_)

            if self.Ki > 0 and not reset:
                self.integralFB_ += self.Ki * 2. * self.halfe * self.sample_period
                self.ifb_x, self.ifb_y, self.ifb_z = self.integralFB_
                gyroscope += self.integralFB_
                self.gyr_x_, self.gyr_y_, self.gyr_z_ = self.gyr_vec
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
            qa, qb, qc, _ = q
            q[0] += (-qb * self.gyr_x_ - qc * self.gyr_y_ - q[3] * self.gyr_z_)
            q[1] += ( qa * self.gyr_x_ + qc * self.gyr_z_ - q[3] * self.gyr_y_)
            q[2] += ( qa * self.gyr_y_ - qb * self.gyr_z_ + q[3] * self.gyr_x_)
            q[3] += ( qa * self.gyr_z_ + qb * self.gyr_y_ - qc * self.gyr_x_)
            recipNorm = 1. / np.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
            q *= recipNorm
            self.quat = q

        # Finish up
        self.euler321_vec = quaternion_to_euler321(self.quat)
        self.roll_rad, self.pitch_rad, self.yaw_rad = self.euler321_vec
        self.yaw_rad += np.pi
        self.euler321_vec_deg = self.euler321_vec * 180. / np.pi
        self.roll_deg, self.pitch_deg, self.yaw_deg = self.euler321_vec_deg
        self.yaw_deg += 180.


class Saved:
    # For plot savings.   A better way is 'Saver' class in pyfilter helpers and requires making a __dict__
    def __init__(self):
        self.reset = []
        self.time = []
        self.T = []
        self.a_raw = []
        self.b_raw = []
        self.c_raw = []
        self.o_raw = []
        self.x_raw = []
        self.y_raw = []
        self.z_raw = []
        self.g_raw = []
        self.halfex = []
        self.halfey = []
        self.halfez = []
        self.halfvx = []
        self.halfvy = []
        self.halfvz = []
        self.ifb_x = []
        self.ifb_y = []
        self.ifb_z = []
        self.q0 = []
        self.q1 = []
        self.q2 = []
        self.q3 = []
        self.roll_rad = []
        self.pitch_rad = []
        self.yaw_rad = []
        self.roll_deg = []
        self.pitch_deg = []
        self.yaw_deg = []
        self.twoKi = []
        self.twoKp = []


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

    track_filter = MahonyAHRS(sample_period=0.1, Kp=5., Ki=8.)  # Kp=5, Ki=8
    track_filter_mathworks = MahonyAHRS_MW(sample_period=0.1, Kp=10., Ki=1.)

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
    track_filter.updateIMU(gyroscope=gyro_vec, accelerometer=accel_vec, sample_time=0.1, reset=init)
    init = False
    while err_tf > 1e-3 and count < 100:
        gyro_vec = np.zeros(3)
        track_filter.updateIMU(gyroscope=gyro_vec, accelerometer=accel_vec, sample_time=0.1, reset=init)
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
    track_filter.updateIMU(gyroscope=gyro_vec, accelerometer=accel_vec, sample_time=0.1, reset=init)
    init = False
    while err_tf > 1e-3 and count < 100:
        gyro_vec = np.zeros(3)
        track_filter.updateIMU(gyroscope=gyro_vec, accelerometer=accel_vec, sample_time=0.1, reset=init)
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
        track_filter.updateIMU(gyroscope=gyro_vec, accelerometer=accel_vec, sample_time = 0.1, reset=init)
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
