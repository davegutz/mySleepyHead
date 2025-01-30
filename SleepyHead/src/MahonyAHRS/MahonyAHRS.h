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
#include "../../constants.h"
#include <math.h>
#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
  #include <Arduino_LSM6DS3.h>
#else
  #error "Only Arduino nano 33 iot has built in IMU"
  #include "application.h"  // Particle
#endif


//--------------------------------------------------------------------------------------------
// Variable declaration

class Mahony {
private:
	char anglesComputed_;
	float acc_x_, acc_y_, acc_z_;		// accelerometer data, g
	float gyr_x_, gyr_y_, gyr_z_;  		// gyroscope data, deg/s
	float halfex_, halfey_, halfez_;  	// track filter errors
	float halfvx_, halfvy_, halfvz_; 	// estimated gravity directions
	float integralFBx_, integralFBy_, integralFBz_;  // integral error terms scaled by Ki
	float mx_, my_, mz_;  				// magnetometer data (not for 6dof, 9dof only)
	float q0_, q1_, q2_, q3_;			// quaternion of sensor frame relative to auxiliary frame
	float roll_, pitch_, yaw_;  		// euler angle. radians
	float twoKp_;						// 2 * track filter proportional gain (Kp)
	float twoKi_;						// 2 * track filter integral gain (Ki)
	static float invSqrt(float x);
	void g_to_euler321();
	void euler321_to_quaternion();
	void quaternion_to_euler321();

//-------------------------------------------------------------------------------------------
// Function declarations

public:
	Mahony(const float t_kp, const float t_ki);
	float getKi() { return twoKi_; }
	float getKp() { return twoKp_; }
	void setKi(const float inp) { twoKi_ = inp; }
	void setKp(const float inp) { twoKp_ = inp; }
	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq,
	 	const boolean reset);
	void updateIMU(const float gx, const float gy, const float gz, const float ax, const float ay, const float az,
		const float invSampleFreq, const boolean reset);
	float getHalfex() { return halfex_; }
	float getHalfey() { return halfey_; }
	float getHalfez() { return halfez_; }
	float getHalfvx() { return halfvx_; }
	float getHalfvy() { return halfvy_; }
	float getHalfvz() { return halfvz_; }
	float getIntegralFBx() { return integralFBx_; }
	float getIntegralFBy() { return integralFBy_; }
	float getIntegralFBz() { return integralFBz_; }
	float getQ0() { return q0_; }
	float getQ1() { return q1_; }
	float getQ2() { return q2_; }
	float getQ3() { return q3_; }
	float getRoll() {
		if (!anglesComputed_) quaternion_to_euler321();
		return roll_ * 57.29578f;
	}
	float getPitch() {
		if (!anglesComputed_) quaternion_to_euler321();
		return pitch_ * 57.29578f;
	}
	float getYaw() {
		if (!anglesComputed_) quaternion_to_euler321();
		return yaw_ * 57.29578f + 180.0f;
	}
	float getRollRadians() {
		if (!anglesComputed_) quaternion_to_euler321();
		return roll_;
	}
	float getPitchRadians() {
		if (!anglesComputed_) quaternion_to_euler321();
		return pitch_;
	}
	float getYawRadians() {
		if (!anglesComputed_) quaternion_to_euler321();
		return yaw_;
	}
	float ax() { return acc_x_; }
	float ay() { return acc_y_; }
	float az() { return acc_z_; }
	float gx() { return gyr_x_; }
	float gy() { return gyr_y_; }
	float gz() { return gyr_z_; }
	float q0() { return q0_; }
	float q1() { return q1_; }
	float q2() { return q2_; }
	float q3() { return q3_; }
};
