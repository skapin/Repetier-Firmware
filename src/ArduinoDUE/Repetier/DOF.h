#ifndef DOF_H_INCLUDED
#define DOF_H_INCLUDED

#include "SFE_LSM9DS0.h"

#include <inttypes.h>

/**********************************************************************
 *  All credit to : https://github.com/sparkfun/LSM9DS0_Breakout 
 * ********************************************************************/
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define DOF_Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define DOF_Ki 0.0f


typedef struct Point
{
	float x;
	float y;
	float z;
} Point;

class DOF
{
  public:
	
	DOF();
	~DOF(){};
	
	
	Point a;
	Point g;
	Point m;
	
	/** Tait–Bryan angles **/
	float current_pitch; // pitch = x
	float current_roll; // roll = y
	float current_yawl; // yawl = z
	
	float deltat = 0.0f;        // integration interval for both filter schemes

	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
	float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
	float temperature;
	
	void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
	void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
	
	static bool angleEquals( float checked_value, float base, float marge )
	{
		return ( checked_value <= (base + marge ) && checked_value >= (base - marge ));
	};
	
	Point getCurrentAngle();
};











#endif // DOF_H_INCLUDED

