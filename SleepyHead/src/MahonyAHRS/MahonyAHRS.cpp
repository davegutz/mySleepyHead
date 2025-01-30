//=============================================================================================
// MahonyAHRS.c
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
// Algorithm paper:
// http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4608934&url=http%3A%2F%2Fieeexplore.ieee.org%2Fstamp%2Fstamp.jsp%3Ftp%3D%26arnumber%3D4608934
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>

//-------------------------------------------------------------------------------------------
// Definitions


//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

Mahony::Mahony(const float t_kp, const float t_ki):  anglesComputed_(0),
	acc_x_(0.0f), acc_y_(0.0f), acc_z_(0.0f), gyr_x_(0.0f), gyr_y_(0.0f), gyr_z_(0.0f),
	halfex_(0.0f), halfey_(0.0f), halfez_(0.0f), halfvx_(0.0f), halfvy_(0.0f), halfvz_(0.0f),
	integralFBx_(0.0f), integralFBy_(0.0f), integralFBz_(0.0f),
	mx_(0.0f), my_(0.0f), mz_(0.0f), q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f),
	roll_(0.0f), pitch_(0.0f), yaw_(0.0f), twoKp_(t_kp), twoKi_(t_ki) {}

void Mahony::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,
	float invSampleFreq, const boolean reset)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfwx, halfwy, halfwz;
	float qa, qb, qc;

	gyr_x_ = gx;
	gyr_y_ = gy;
	gyr_z_ = gz;
	acc_x_ = ax;
	acc_y_ = ay;
	acc_z_ = az;
	mx_ = mx;
	my_ = my;
	mz_ = mz;

	// Use IMU algorithm if magnetometer measurement invalid
	// (avoids NaN in magnetometer normalisation)
	if((mx_ == 0.0f) && (my_ == 0.0f) && (mz_ == 0.0f)) {
		updateIMU(gyr_x_, gyr_y_, gyr_z_, acc_x_, acc_y_, acc_z_, invSampleFreq, reset);
		return;
	}

	// Convert gyroscope degrees/sec to radians/sec
	gyr_x_ *= 0.0174533f;
	gyr_y_ *= 0.0174533f;
	gyr_z_ *= 0.0174533f;

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((acc_x_ == 0.0f) && (acc_y_ == 0.0f) && (acc_z_ == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(acc_x_*acc_x_ + acc_y_*acc_y_ + acc_z_*acc_z_);
		acc_x_ *= recipNorm;
		acc_y_ *= recipNorm;
		acc_z_ *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx_*mx_ + my_*my_ + mz_*mz_);
		mx_ *= recipNorm;
		my_ *= recipNorm;
		mz_ *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0_ * q0_;
		q0q1 = q0_ * q1_;
		q0q2 = q0_ * q2_;
		q0q3 = q0_ * q3_;
		q1q1 = q1_ * q1_;
		q1q2 = q1_ * q2_;
		q1q3 = q1_ * q3_;
		q2q2 = q2_ * q2_;
		q2q3 = q2_ * q3_;
		q3q3 = q3_ * q3_;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx_*(0.5f - q2q2 - q3q3) + my_*(q1q2 - q0q3)        + mz_*(q1q3 + q0q2));
		hy = 2.0f * (mx_*(q1q2 + q0q3)        + my_*(0.5f - q1q1 - q3q3) + mz_*(q2q3 - q0q1));
		bx = sqrtf(hx*hx + hy*hy);
		bz = 2.0f * (mx_*(q1q3 - q0q2)        + my_*(q2q3 + q0q1)        + mz_*(0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx_ = q1q3 - q0q2;
		halfvy_ = q0q1 + q2q3;
		halfvz_ = q0q0 - 0.5f + q3q3;
		halfwx = bx*(0.5f - q2q2 - q3q3) + bz*(q1q3 - q0q2);
		halfwy = bx*(q1q2 - q0q3)        + bz*(q0q1 + q2q3);
		halfwz = bx*(q0q2 + q1q3)        + bz*(0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction
		// and measured direction of field vectors
		halfex_ = (acc_y_*halfvz_ - acc_z_*halfvy_) + (my_*halfwz - mz_*halfwy);
		halfey_ = (acc_z_*halfvx_ - acc_x_*halfvz_) + (mz_*halfwx - mx_*halfwz);
		halfez_ = (acc_x_*halfvy_ - acc_y_*halfvx_) + (mx_*halfwy - my_*halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi_ > 0.0f) {
			// integral error scaled by Ki
			integralFBx_ += twoKi_ * halfex_ * invSampleFreq;
			integralFBy_ += twoKi_ * halfey_ * invSampleFreq;
			integralFBz_ += twoKi_ * halfez_ * invSampleFreq;
			gyr_x_ += integralFBx_;	// apply integral feedback
			gyr_y_ += integralFBy_;
			gyr_z_ += integralFBz_;
		} else {
			integralFBx_ = 0.0f;	// prevent integral windup
			integralFBy_ = 0.0f;
			integralFBz_ = 0.0f;
		}

		// Apply proportional feedback
		gyr_x_ += twoKp_ * halfex_;
		gyr_y_ += twoKp_ * halfey_;
		gyr_z_ += twoKp_ * halfez_;
	}

	// Integrate rate of change of quaternion
	gyr_x_ *= (0.5f * invSampleFreq);		// pre-multiply common factors
	gyr_y_ *= (0.5f * invSampleFreq);
	gyr_z_ *= (0.5f * invSampleFreq);
	qa = q0_;
	qb = q1_;
	qc = q2_;
	q0_ += (-qb*gyr_x_ - qc*gyr_y_ - q3_*gyr_z_);
	q1_ += ( qa*gyr_x_ + qc*gyr_z_ - q3_*gyr_y_);
	q2_ += ( qa*gyr_y_ - qb*gyr_z_ + q3_*gyr_x_);
	q3_ += ( qa*gyr_z_ + qb*gyr_y_ - qc*gyr_x_);

	// Normalise quaternion
	recipNorm = invSqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
	q0_ *= recipNorm;
	q1_ *= recipNorm;
	q2_ *= recipNorm;
	q3_ *= recipNorm;
	anglesComputed_ = 0;
}

//-------------------------------------------------------------------------------------------
// IMU algorithm update

void Mahony::updateIMU(const float gx, const float gy, const float gz, const float ax, const float ay, const float az,
					   const float invSampleFreq, const boolean reset)
{
	float recipNorm;

	gyr_x_ = gx;
	gyr_y_ = gy;
	gyr_z_ = gz;
	acc_x_ = ax;
	acc_y_ = ay;
	acc_z_ = az;
	float qa, qb, qc;

	// Convert gyroscope degrees/sec to radians/sec
	gyr_x_ *= 0.0174533f;
	gyr_y_ *= 0.0174533f;
	gyr_z_ *= 0.0174533f;

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((acc_x_ == 0.0f) && (acc_y_ == 0.0f) && (acc_z_ == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(acc_x_*acc_x_ + acc_y_*acc_y_ + acc_z_*acc_z_);
		acc_x_ *= recipNorm;
		acc_y_ *= recipNorm;
		acc_z_ *= recipNorm;

		// Initialization logic
		if(reset)
		{
			g_to_euler321();
			euler321_to_quaternion();
			Serial.print("acc_x_ = "); Serial.print(acc_x_); Serial.print("\tay_ = "); Serial.print(acc_y_); Serial.print("\taz_ = "); Serial.print(acc_z_);
			Serial.print("\troll_init = "); Serial.print(getRoll()); Serial.print("\tpitch_init = "); Serial.print(getPitch()); Serial.print("\tyaw_init = "); Serial.print(getYaw());
			Serial.print("\tq0_init = "); Serial.print(q0_, 5); Serial.print("\tq1_init = "); Serial.print(q1_, 5); Serial.print("\tq2_init = "); Serial.print(q2_, 5); Serial.print("\tq3_init = "); Serial.println(q3_, 5);
		}

		// Estimated direction of gravity
		halfvx_ = q1_*q3_ - q0_*q2_;
		halfvy_ = q0_*q1_ + q2_*q3_;
		halfvz_ = q0_*q0_ + q3_*q3_ - 0.5f ;
		// halfvz_ = (q0_*q0_ - q1_*q1_ - q2_*q2_ + q3_*q3_) * 0.5f;  // alternative computation, same result

		// Error is sum of cross product between estimated
		// and measured direction of gravity
		halfex_ = (acc_y_*halfvz_ - acc_z_*halfvy_);
		halfey_ = (acc_z_*halfvx_ - acc_x_*halfvz_);
		halfez_ = (acc_x_*halfvy_ - acc_y_*halfvx_);

		// Compute and apply integral feedback if enabled
		if(twoKi_ > 0.0f) {
			// integral error scaled by Ki
			integralFBx_ += twoKi_ * halfex_ * invSampleFreq;
			integralFBy_ += twoKi_ * halfey_ * invSampleFreq;
			integralFBz_ += twoKi_ * halfez_ * invSampleFreq;
			gyr_x_ += integralFBx_;	// apply integral feedback
			gyr_y_ += integralFBy_;
			gyr_z_ += integralFBz_;
		} else {
			integralFBx_ = 0.0f;	// prevent integral windup
			integralFBy_ = 0.0f;
			integralFBz_ = 0.0f;
		}

		// Apply proportional feedback
		gyr_x_ += twoKp_ * halfex_;
		gyr_y_ += twoKp_ * halfey_;
		gyr_z_ += twoKp_ * halfez_;
	}

	// Integrate rate of change of quaternion
	gyr_x_ *= (0.5f * invSampleFreq);		// pre-multiply common factors
	gyr_y_ *= (0.5f * invSampleFreq);
	gyr_z_ *= (0.5f * invSampleFreq);
	qa = q0_;
	qb = q1_;
	qc = q2_;
	q0_ += (-qb*gyr_x_ - qc*gyr_y_ - q3_*gyr_z_);
	q1_ += ( qa*gyr_x_ + qc*gyr_z_ - q3_*gyr_y_);
	q2_ += ( qa*gyr_y_ - qb*gyr_z_ + q3_*gyr_x_);
	q3_ += ( qa*gyr_z_ + qb*gyr_y_ - qc*gyr_x_);

	// Normalise quaternion
	recipNorm = invSqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
	q0_ *= recipNorm;
	q1_ *= recipNorm;
	q2_ *= recipNorm;
	q3_ *= recipNorm;
	anglesComputed_ = 0;
}

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float Mahony::invSqrt(float x)
{
	float halfx = 0.5f * x;
	union { float f; long l; } i;
	i.f = x;
	i.l = 0x5f3759df - (i.l >> 1);
	float y = i.f;
	y = y * (1.5f - (halfx*y*y));
	y = y*(1.5f - (halfx*y*y));
	return y;
}

//-------------------------------------------------------------------------------------------


void Mahony::quaternion_to_euler321()
{
	roll_ = atan2f(q0_*q1_ + q2_*q3_, 0.5f - q1_*q1_ - q2_*q2_);
	pitch_ = asinf(-2.0f * (q1_*q3_ - q0_*q2_));
	yaw_ = atan2f(q1_*q2_ + q0_*q3_, 0.5f - q2_*q2_ - q3_*q3_);
	// yaw_ = 0;
	anglesComputed_ = 1;
}

void Mahony::g_to_euler321()
{
	roll_ =  atan2f(acc_y_, sqrt(acc_x_*acc_x_ + acc_z_*acc_z_));
	pitch_ = -atan2f(acc_x_, acc_z_);
}

void Mahony::euler321_to_quaternion()
{
	float cr, sr, cp, sp, cy, sy;
	cr = cos(roll_ / 2.);
	sr = sin(roll_ / 2.);
	cp = cos(pitch_ / 2.);
	sp = sin(pitch_ / 2.);
	cy = cos(yaw_ / 2.);
	sy = sin(yaw_ / 2.);
	q0_ = cr*cp*cy + sr*sp*sy;
	q1_ = sr*cp*cy - cr*sp*sy;
	q2_ = cr*sp*cy + sr*cp*sy;
	q3_ = cr*cp*sy - sr*sp*cy;
}
//============================================================================================
// END OF CODE
//============================================================================================
