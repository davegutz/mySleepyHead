
import numpy as np
from pyquaternion import Quaternion as qu
import  math

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
    def __init__(self, samplePeriod=None, quaternion=None, kp=None, ki=None):
        if samplePeriod is not None:
            self.samplePeriod = 1./256.
        else:
            self.samplePeriod = samplePeriod
        if quaternion is None:
            self.quat = qu([1., 0., 0., 0.])  # output quaternion describing the Earth relative to the sensor
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
        self.eInt = np.array([0., 0., 0.])  # integral error
        self.angles_vec_check_deg = None
        self.angles_vec_deg = None
        self.halfv = None
        self.halfe = None

    def update(self, Gyroscope, Accelerometer, Magnetometer):
        q = self.quat # short name local variable for readability

        # Normalise accelerometer measurement
        if np.linalg.norm(Accelerometer) == 0:
            return  # handle NaN
        Accelerometer = Accelerometer / np.linalg.norm(Accelerometer)    # normalise magnitude

        # Normalise magnetometer measurement
        if np.linalg.norm(Magnetometer) == 0:
            return  # handle NaN
        Magnetometer = Magnetometer / np.linalg.norm(Magnetometer)   # normalise magnitude

        # Reference direction of Earth's magnetic feield
        h = q * qu([0., Magnetometer]) * q.conjugate
        norm_short_h = np.linalg.norm(h[1], h[2])
        b = qu([0., norm_short_h, 0., h[3]])

        # Estimated direction of gravity and magnetic field
        v = np.array([  2*(q[1]*q[3] - q[0]*q[2]),
                        2*(q[0]*q[1] + q[2]*q[3]),
                        q[0]^2 - q[1]^2 - q[2]^2 + q[3]^2  ])
        w = np.array([  2*b[1]*(0.5 - q[2]^2 - q[3]^2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]),
                        2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]),
                        2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]^2 - q[2]^2) ])

        # Error is sum of cross product between estimated direction and measured direction of fields
        e = np.linalg.cross(Accelerometer, v) + np.linalg.cross(Magnetometer, w)
        if self.Ki > 0:
            self.eInt = self.eInt + e * self.SamplePeriod
        else:
            self.eInt = np.array([0., 0., 0.])

        # Apply feedback terms
        Gyroscope = Gyroscope + self.Kp * e + self.Ki * self.eInt

        # Compute rate of change of quaternion
        qDot = 0.5 * q * qu([0., Gyroscope(1), Gyroscope(2), Gyroscope(3)])

        # Integrate to yield quaternion
        q = q + qDot * self.SamplePeriod
        self.quat = q / np.linalg.norm(q) # normalise quaternion

    def update_imu(self, Gyroscope, Accelerometer, reset=False):
        Gyroscope *= 0.0174533
        gyr_x_ = Gyroscope[0]
        gyr_y_ = Gyroscope[1]
        gyr_z_ = Gyroscope[2]
        acc_x_ = Accelerometer[0]
        acc_y_ = Accelerometer[1]
        acc_z_ = Accelerometer[2]

        if not ( acc_x_==0. and acc_y_==0. and acc_z_==0. ):
            # Normalise accelerometer measurement
            if np.linalg.norm(Accelerometer) == 0:
                return # handle NaN
            recipNorm = 1. / np.linalg.norm(Accelerometer)
            acc_x_ *= recipNorm
            acc_y_ *= recipNorm
            acc_z_ *= recipNorm

            if reset:
                g_vec = np.array([acc_x_, acc_y_, acc_z_])
                angles_vec = self.g_to_angles(g_vec)
                quat = self.angles_to_quaternion(angles_vec)
                self.angles_vec_check_deg = self.quaternion_to_angles(quat) * 180. / np.pi
                self.angles_vec_deg = angles_vec * 180. / np.pi

            q = self.quat # short name local variable for readability

            # Estimated direction of gravity and magnetic flux
            self.halfv = np.array([ 2*(q[1]*q[3] - q[0]*q[2]),
                                    2*(q[0]*q[1] + q[2]*q[3]),
                                    # q[0]^2 - q[1]^2 - q[2]^2 + q[3]^2  ])
                                    q[0]*q[0] + q[3]*q[3] - 0.5  ])
            halfvx_ = self.halfv[0]
            halfvy_ = self.halfv[1]
            halfvz_ = self.halfv[2]

            # Error is sum of cross product between estimated direction and measured direction of field
            self.halfe = np.linalg.cross(Accelerometer, self.halfv)
            self.halfex_ = (acc_y_*halfvz_ - acc_z_*halfvy_)
            self.halfey_ = (acc_z_*halfvx_ - acc_x_*halfvz_)
            self.halfez_ = (acc_x_*halfvy_ - acc_y_*halfvx_)

            if self.Ki > 0:
                self.eInt += self.Ki * 2. * self.halfe * self.SamplePeriod
            else:
                self.eInt = np.array([0., 0., 0.])

        # Apply feedback terms
        Gyroscope = Gyroscope + self.Kp * e + self.Ki * self.eInt

        # Compute rate of change of quaternion
        qDot = 0.5 * q * qu([ 0., Gyroscope[0], Gyroscope[1], Gyroscope[2] ])

        # Integrate to yield quaternion
        q = q + qDot * self.SamplePeriod
        self.quat = q / np.linalg.norm(q) # normalise quaternion

    def angles_to_quaternion(self, euler_angles):
        """
        Converts Euler angles  to a quaternion.
        Args: euler_angles = [roll, pitch, yaw]

        Returns:
            list: A quaternion represented as [w, x, y, z].
        """

        roll, pitch, yaw = euler_angles

        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [w, x, y, z]

    def g_to_angles(self, g_vector):
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

    def quaternion_to_angles(self, quaternion):
        """
        Converts quaternion to Euler angles
            arg: A quaternion represented as [x, y, z, w].

        Returns:
            list: euler_angles = [roll, pitch, yaw]
        """
        w, x, y, z = quaternion

        roll = np.arctan2(w*x + y*z, 0.5 - x*x - y*y);
        pitch = np.arcsin(-2.0 * (x*z - w*y))
        # yaw = np.arctan2(x*y + w*z, 0.5 - y*y - z*z);
        yaw = 0.;

        return np.array([roll, pitch, yaw])
