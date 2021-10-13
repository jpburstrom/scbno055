/*
Inertial Measurement Unit (IMU) Sensing with the BNO055
----------------------------------------------------------

This sketch allows you to hook up BNO055 IMU movement sensing device
to Bela, for example the Adafruit BNO055 breakout board.

To get this working with Bela you need to connect the breakout board to the I2C
terminal on the Bela board. See the Pin guide for details of which pin is which.

Connect a push button with a pull-down resistor to pin P8_08.

When running sketch, hold IMU in a neutral position and press the push button 
once. Tilt IMU down (if wearing as for head-tracking, look down) and press the 
push button a second time. The system is now calibrated. Calibration can be
run again at any time.

Based on IMU-Sine-Synth-Pd Example from Bela On Ur Head by Becky Stewart
https://github.com/theleadingzero/belaonurhead

Johannes Burström 2021
*/

#include "Bela.h"
#include "SC_BNO055.h"

bool SC_BNO055::setup() {
	if(!bno.begin()) {
		rt_printf("Error initialising BNO055\n");
		return false;
	}

	rt_printf("Initialised BNO055\n");
	
	// use external crystal for better accuracy
  	bno.setExtCrystalUse(true);
  	
	// get the system status of the sensor to make sure everything is ok
	uint8_t sysStatus, selfTest, sysError;
  	bno.getSystemStatus(&sysStatus, &selfTest, &sysError);
	rt_printf("System Status: %d (0 is Idle)   Self Test: %d (15 is all good)   System Error: %d (0 is no error)\n", sysStatus, selfTest, sysError);
	// get calibration status
	uint8_t sys, gyro, accel, mag;

	return true;
}

void SC_BNO055::setCalibration(bnoCalibration_t calData)
{
	mIdleConj = calData.idleConj;
	mGravCal = calData.cal;
}

void SC_BNO055::getCalibration(bnoCalibration_t &calData)
{
    calData.idleConj = mIdleConj;
    calData.cal = mGravCal;
}



// Auxiliary task to read from the I2C board
void SC_BNO055::readIMU(bnoState_t &state)
{
	// get calibration status
	uint8_t sys, gyro, accel, mag;

	bno.getCalibration(&sys, &gyro, &accel, &mag);

    imu::Vector<3> vec;


    vec = bno.getVector(I2C_BNO055::VECTOR_ACCELEROMETER);
    state.ax = vec.x();
    state.ay = vec.y();
    state.az = vec.z();

    vec = bno.getVector(I2C_BNO055::VECTOR_GYROSCOPE);
    state.gx = vec.x();
    state.gy = vec.y();
    state.gz = vec.z();

    vec = bno.getVector(I2C_BNO055::VECTOR_MAGNETOMETER);
    state.mx = vec.x();
    state.my = vec.y();
    state.mz = vec.z();

	
	// quaternion data routine from MrHeadTracker
  	imu::Quaternion qRaw = bno.getQuat(); //get sensor raw quaternion data
  	
  	steering = mIdleConj * qRaw; // calculate relative rotation data
  	quat = mCalLeft * steering; // transform it to calibrated coordinate system
  	quat = quat * mCalRight;

    //Yaw, Pitch, Roll, Yaw
    vec = quat.toEuler(); // transform from quaternion to Euler
    state.yaw = vec[0];
    state.pitch = vec[1];
    state.roll = vec[2];

}

// Auxiliary task to read from the I2C board
void SC_BNO055::getNeutralGravity() {
	// read in gravity value
  	imu::Vector<3> gravity = bno.getVector(I2C_BNO055::VECTOR_GRAVITY);
    mIdleConj = bno.getQuat().conjugate(); // sets what is looking forward
  	gravity = gravity.scale(-1);
  	gravity.normalize();
  	mGravIdle = gravity;
}

// Auxiliary task to read from the I2C board
void SC_BNO055::getDownGravity() {
	// read in gravity value
  	imu::Vector<3> gravity = bno.getVector(I2C_BNO055::VECTOR_GRAVITY);
  	gravity = gravity.scale(-1);
  	gravity.normalize();
  	mGravCal = gravity;
}

// calibration of coordinate system from MrHeadTracker
// see http://www.aes.org/e-lib/browse.cfm?elib=18567 for full paper
// describing algorithm
void SC_BNO055::recalcCalibration() {
  	imu::Vector<3> g, gravCalTemp, x, y, z;
  	g = mGravIdle; // looking forward in neutral position
  
  	z = g.scale(-1); 
  	z.normalize();

  	gravCalTemp = mGravCal; // looking down
  	y = gravCalTemp.cross(g);
  	y.normalize();

  	x = y.cross(z);
  	x.normalize();

  	imu::Matrix<3> rot;
  	rot.cell(0, 0) = x.x();
  	rot.cell(1, 0) = x.y();
  	rot.cell(2, 0) = x.z();
  	rot.cell(0, 1) = y.x();
  	rot.cell(1, 1) = y.y();
  	rot.cell(2, 1) = y.z();
  	rot.cell(0, 2) = z.x();
  	rot.cell(1, 2) = z.y();
  	rot.cell(2, 2) = z.z();

  	mCal.fromMatrix(rot);

  	resetOrientation();
}

// from MrHeadTracker
// resets values used for looking forward
void SC_BNO055::resetOrientation() {
  	mCalLeft = mCal.conjugate();
  	mCalRight = mCal;
}
