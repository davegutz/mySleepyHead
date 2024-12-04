//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#pragma once

#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration

class Mahony {
private:
	float twoKp_;		// 2 * proportional gain (Kp)
	float twoKi_;		// 2 * integral gain (Ki)
	float q0_, q1_, q2_, q3_;	// quaternion of sensor frame relative to auxiliary frame
	float gx_, gy_, gz_, ax_, ay_, az_, mx_, my_, mz_;  // IMU data
	float integralFBx_, integralFBy_, integralFBz_;  // integral error terms scaled by Ki
	float roll_, pitch_, yaw_;
	char anglesComputed_;
	static float invSqrt(float x);
	void computeAccelToAngles();
	void computeAnglesToQuaternion();
	void computeQuaternionToAngles();

//-------------------------------------------------------------------------------------------
// Function declarations

public:
	Mahony(const float t_kp, const float t_ki);
	float getKi() { return twoKi_; }
	float getKp() { return twoKp_; }
	void setKi(const float inp) { twoKi_ = inp; }
	void setKp(const float inp) { twoKp_ = inp; }
	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);
	void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
	float getRoll() {
		if (!anglesComputed_) computeQuaternionToAngles();
		return roll_ * 57.29578f;
	}
	float getPitch() {
		if (!anglesComputed_) computeQuaternionToAngles();
		return pitch_ * 57.29578f;
	}
	float getYaw() {
		if (!anglesComputed_) computeQuaternionToAngles();
		return yaw_ * 57.29578f + 180.0f;
	}
	float getRollRadians() {
		if (!anglesComputed_) computeQuaternionToAngles();
		return roll_;
	}
	float getPitchRadians() {
		if (!anglesComputed_) computeQuaternionToAngles();
		return pitch_;
	}
	float getYawRadians() {
		if (!anglesComputed_) computeQuaternionToAngles();
		return yaw_;
	}
};
