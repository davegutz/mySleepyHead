
import numpy as np
from pyquaternion import Quaternion as qu
class MahonyAHRS_mathworks:
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

    def update_imu(self, Gyroscope, Accelerometer):
        # Normalise accelerometer measurement
        if np.linalg.norm(Accelerometer) == 0:
            return # handle NaN
        Accelerometer = Accelerometer / np.linalg.norm(Accelerometer)	# normalise magnitude

        q = self.quat # short name local variable for readability

        # Estimated direction of gravity and magnetic flux
        v = np.array([  2*(q[1]*q[3] - q[0]*q[2]),
                        2*(q[0]*q[1] + q[2]*q[3]),
                        q[0]^2 - q[1]^2 - q[2]^2 + q[3]^2  ])

        # Error is sum of cross product between estimated direction and measured direction of field
        e = np.linalg.cross(Accelerometer, v)
        if self.Ki > 0:
            self.eInt = self.eInt + e * self.SamplePeriod
        else:
            self.eInt = np.array([0., 0., 0.])

        # Apply feedback terms
        Gyroscope = Gyroscope + self.Kp * e + self.Ki * self.eInt

        # Compute rate of change of quaternion
        qDot = 0.5 * q * qu([ 0., Gyroscope[0], Gyroscope[1], Gyroscope[2] ])

        # Integrate to yield quaternion
        q = q + qDot * self.SamplePeriod
        self.quat = q / np.linalg.norm(q) # normalise quaternion
