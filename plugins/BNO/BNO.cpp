#include <atomic>
#include <Bela.h>
#include "SC_PlugIn.h"
#include <thread>

//#include "mock.hpp"
#include "imu/SC_BNO055.h"

// written with reference to the chapter "Writing Unit Generator Plug-ins" in The SuperCollider Book
// and also http://doc.sccode.org/Guides/WritingUGens.html accessed March 2, 2015
//

static InterfaceTable *ft;

enum bnoTask {
    TASK_STOP = 0,
    TASK_RUN=2,
    TASK_CALIBRATE_IDLE=4,
    TASK_CMD=8,
    TASK_CALIBRATE_1=9,
    TASK_CALIBRATE_2=10,
    TASK_SAVE=11,
    TASK_LOAD=12
};

struct BNO : public Unit {
    //Outputs 3 values
    static const int ACCEL = 0;
    static const int GYRO = 1;
    static const int MAG = 2;
    static const int ORIENTATION = 3;

    SC_BNO055* bno;

    int currentTask = TASK_STOP;
    int channel = ORIENTATION;
    int outputs = 3;
    float m_caltrig;
    float m_loadtrig;
    float m_savetrig;

    unsigned int readInterval; // read interval in ms
    std::thread* thread;
    volatile int threadShouldStop;
};

bnoState_t gData = {
	{0.0}, {0.0}, {0.0},
	{0.0}, {0.0}, {0.0},
	{0.0}, {0.0}, {0.0},
	{0.0}, {0.0}, {0.0} };

enum bnoChannel {
    CH_ACC,
    CH_GYR,
    CH_MAG,
    CH_ORI
};

void BNO_Ctor(BNO *unit);
void BNO_Dtor(BNO *unit);
void BNO_next_k(BNO *unit, int numSamples);

static void updateBNO(BNO* unit) {

    while(!unit->threadShouldStop && !Bela_stopRequested()) {
        if (unit->currentTask == TASK_RUN) {
            unit->bno->readIMU(gData);
        } else {

            char calibrationPath[128];
            snprintf(calibrationPath, 127, "%s/.bnoCalibration", getenv("HOME"));
            bnoCalibration_t calData;
            FILE *fp;
            switch ( unit->currentTask ) {
            case TASK_CALIBRATE_IDLE:
                break;

            case TASK_CALIBRATE_1:
                printf("BNO: Calibrating, neutral position\n");
                unit->bno->getNeutralGravity();
                unit->currentTask = TASK_CALIBRATE_IDLE;
                break;

            case TASK_CALIBRATE_2:
                printf("BNO: Calibrating, tilted down\n");
                unit->bno->getDownGravity();
                unit->bno->recalcCalibration();
                printf("Done, running\n");
                unit->currentTask = TASK_RUN;
                break;
            case TASK_SAVE:
                Print("Saving calibration\n");
                fp = fopen(calibrationPath, "wb+");
                if (fp != NULL) {
                    unit->bno->getCalibration(calData);
                    fwrite(&calData, sizeof(bnoCalibration_t), 1, fp);
                    fclose(fp);
                } else {
                    printf("BNO: Couldn't open file for writing\n");
                }
                unit->currentTask = TASK_RUN;
                break;

            case TASK_LOAD:
                Print("Loading calibration\n");
                fp = fopen(calibrationPath, "rb");
                if (fp != NULL) {
                    fread(&(calData), sizeof(bnoCalibration_t), 1, fp);
                    fclose(fp);
                    unit->bno->setCalibration(calData);
                    unit->bno->recalcCalibration();
                } else {
                    printf("BNO: Couldn't open file for reading\n");
                }
                unit->currentTask = TASK_RUN;
                break;
            }
        }
        usleep(unit->readInterval);
    }
}

void BNO_Ctor(BNO *unit) {
    unit->channel = static_cast<int>(IN0(0));
    unit->outputs = 3;
    unit->m_caltrig = 0.f;
    unit->m_savetrig = 0.f;
    unit->m_loadtrig = 0.f;

    unit->bno = new SC_BNO055();

    if ( unit->bno->setup() ) {
        unit->currentTask = TASK_LOAD;
    } else {
		printf("Error initialising BNO055\n");
        unit->currentTask = TASK_STOP;
    }

    SETCALC(BNO_next_k);
    BNO_next_k(unit, 1);

    unit->readInterval = 50; //microseconds
    unit->threadShouldStop = 0;
    unit->thread = new std::thread(updateBNO, unit);
}

void BNO_Dtor(BNO* unit) {
  if(unit->thread && unit->thread->joinable())
  {
    unit->threadShouldStop = 1;
    unit->thread->join();
  }
  delete unit->thread;
  delete unit->bno;
}

void BNO_next_k(BNO *unit, int numSamples) {

    float trig;
    float* calInput = ZIN(1);
    float* loadInput = ZIN(2);
    float* saveInput = ZIN(3);
    float cal_prevtrig = unit->m_caltrig;
    float load_prevtrig = unit->m_loadtrig;
    float save_prevtrig = unit->m_savetrig;

    //If we're currently not in a command, check for command triggers
    if ((unit->currentTask & TASK_CMD) != TASK_CMD) {
        //Calibration
        for (int i = 0; i < numSamples; ++i) {
            trig = ZXP(calInput);
            if (trig > 0.f && cal_prevtrig <= 0.f) {
                if (unit->currentTask == TASK_CALIBRATE_IDLE) {
                    unit->currentTask = TASK_CALIBRATE_2;
                    rt_printf("Calibrating, step 2\n");
                } else {
                    unit->currentTask = TASK_CALIBRATE_1;
                    rt_printf("Calibrating, step 1\n");
                }
            }
            cal_prevtrig = trig;

            trig = ZXP(loadInput);
            if (trig > 0.f && load_prevtrig <= 0.f) {
                unit->currentTask = TASK_LOAD;
                //call aux task here
            }
            load_prevtrig = trig;

            trig = ZXP(saveInput);
            if (trig > 0.f && save_prevtrig <= 0.f) {
                unit->currentTask = TASK_SAVE;
                //call aux task here
            }
            save_prevtrig = trig;
        }

        unit->m_caltrig = cal_prevtrig;
        unit->m_loadtrig = load_prevtrig;
        unit->m_savetrig = save_prevtrig;
    }

    if (unit->currentTask == TASK_RUN) {

        switch (unit->channel) {
        case CH_ACC:
            OUT0(0) = gData.ax;
            OUT0(1) = gData.ay;
            OUT0(2) = gData.az;
            break;
        case CH_GYR:
            OUT0(0) = gData.gx;
            OUT0(1) = gData.gy;
            OUT0(2) = gData.gz;
            break;
        case CH_MAG:
            OUT0(0) = gData.mx;
            OUT0(1) = gData.my;
            OUT0(2) = gData.mz;
            break;
        case CH_ORI:
            OUT0(0) = gData.pitch;
            OUT0(1) = gData.roll;
            OUT0(2) = gData.yaw;
            break;
        }


    } else {
        //Zero outputs while doing calibration
        for (int o = 0; o < 3; o++) {
            OUT0(o) = 0.f;
        }

    }
}


PluginLoad(BNO)
{

    ft = inTable;


    DefineDtorCantAliasUnit(BNO);
}
