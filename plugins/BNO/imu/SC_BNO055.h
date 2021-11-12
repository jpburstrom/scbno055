/*
  Inertial Measurement Unit (IMU) Sensing with the BNO055
  ----------------------------------------------------------
  When running sketch, hold IMU in a neutral position and press the push button
  once. Tilt IMU down (if wearing as for head-tracking, look down) and press the
  push button a second time. The system is now calibrated. Calibration can be
  run again at any time.

  Based on IMU-Sine-Synth-Pd Example from Bela On Ur Head by Becky Stewart
  https://github.com/theleadingzero/belaonurhead

  Johannes Burström 2021
*/

#include <atomic>
#include "Bela_BNO055.h"

typedef struct  {
    std::atomic<float> ax, ay, az, gx, gy, gz, mx, my, mz, pitch, roll, yaw;
} bnoState_t;

typedef struct {
	imu::Quaternion idleConj;
	imu::Vector<3> gravIdle;
	imu::Vector<3> gravCal;
} bnoCalibration_t;

class SC_BNO055 {
public:
	SC_BNO055() {};
	bool setup();
	void setCalibration(bnoCalibration_t calData);
	void getCalibration(bnoCalibration_t &calData);
	// function declarations
	void readIMU(bnoState_t &state);
	void getNeutralGravity();
	void getDownGravity();
	void recalcCalibration();

private:
	// Change this to change how often the BNO055 IMU is read (in Hz)
	int readInterval = 100;
	I2C_BNO055 bno; // IMU sensor object

	// Quaternions and Vectors
	imu::Quaternion mCalLeft, mCalRight, mCal, mIdleConj = {1, 0, 0, 0};
	imu::Quaternion quat, steering, qRaw;

	imu::Vector<3> mGravIdle, mGravCal;

	int readCount = 0;			// How long until we read again...
	int readIntervalSamples = 0; // How many samples between reads

	//int printThrottle = 0; // used to limit printing frequency
	void resetOrientation();

};
