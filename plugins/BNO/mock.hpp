#include "imu/quaternion.h"

typedef struct  {
    float ax, ay, az,
        gx, gy, gz,
        mx, my, mz,
        pitch, roll, yaw;
} bnoState_t;

typedef struct {
	imu::Quaternion idleConj;
	imu::Quaternion cal;
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
    private:
        bnoCalibration_t data;
};

bool SC_BNO055::setup() {
  return true;
}

void SC_BNO055::setCalibration(bnoCalibration_t calData) {
  data.idleConj = calData.idleConj;
  data.cal = calData.cal;
}
void SC_BNO055::getCalibration(bnoCalibration_t &calData) {
    calData.idleConj = data.idleConj;
    calData.cal = data.cal;
}

void SC_BNO055::readIMU(bnoState_t &state) {
    /*
   state.x = 0.f;
   state.y = 0.f;
   state.z = 0.f;
   */
}

//Calibration methods
void SC_BNO055::getNeutralGravity() {}
void SC_BNO055::getDownGravity() {}
