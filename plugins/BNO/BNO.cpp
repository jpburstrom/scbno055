#include <pthread.h>
#include <atomic>
#include "SC_Lock.h"
#include "SC_PlugIn.h"

#include "mock.hpp"

// written with reference to the chapter "Writing Unit Generator Plug-ins" in The SuperCollider Book
// and also http://doc.sccode.org/Guides/WritingUGens.html accessed March 2, 2015
//

static InterfaceTable *ft;

//Read interval
#define INTERVAL 2.5
#define IDLE_INTERVAL 100

struct BNO : public Unit {
    //Outputs 3 values
    static const int ACCEL = 0;
    static const int GYRO = 1;
    static const int MAG = 2;
    static const int ORIENTATION = 3;

    int channel = ORIENTATION;
    int outputs = 3;
    float m_caltrig;
    float m_loadtrig;
    float m_savetrig;

};

bnoState_t gData = {0.1, 0.2, 0.3,
    0.2, 0.0, 0.4,
    0.3, 0.0, 0.5,
    0.4, 0.5, 0.6};


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

enum bnoChannel {
    CH_ACC,
    CH_GYR,
    CH_MAG,
    CH_ORI
};

//Current command
std::atomic_int currentTask = { TASK_STOP };


// PLUGIN INTERFACE
void BNO_Ctor(BNO *unit);
void BNO_next_k(BNO *unit, int numSamples);

// PLUGIN IMPLEMENTATION
SC_BNO055 bno;

void BNO_Ctor(BNO *unit) {
    unit->channel = static_cast<int>(IN0(0));
    unit->outputs = 3;
    unit->m_caltrig = 0.f;
    unit->m_savetrig = 0.f;
    unit->m_loadtrig = 0.f;

    SETCALC(BNO_next_k);
    BNO_next_k(unit, 1);
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
    if ((currentTask & TASK_CMD) != TASK_CMD) {
        //Calibration
        for (int i = 0; i < numSamples; ++i) {
            trig = ZXP(calInput);
            if (trig > 0.f && cal_prevtrig <= 0.f) {
                if (currentTask == TASK_CALIBRATE_IDLE) {
                    currentTask = TASK_CALIBRATE_2;
                    printf("Calibrating, step 2\n");
                } else {
                    currentTask = TASK_CALIBRATE_1;
                    printf("Calibrating, step 1\n");
                }
            }
            cal_prevtrig = trig;

            trig = ZXP(loadInput);
            if (trig > 0.f && load_prevtrig <= 0.f) {
                currentTask = TASK_LOAD;
            }
            load_prevtrig = trig;

            trig = ZXP(saveInput);
            if (trig > 0.f && save_prevtrig <= 0.f) {
                currentTask = TASK_SAVE;
            }
            save_prevtrig = trig;
        }

        unit->m_caltrig = cal_prevtrig;
        unit->m_loadtrig = load_prevtrig;
        unit->m_savetrig = save_prevtrig;
    }

    if (currentTask == TASK_RUN) {

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
            OUT0(0) = gData.p;
            OUT0(1) = gData.r;
            OUT0(2) = gData.y;
            break;
        }


    } else {
        //Zero outputs while doing calibration
        for (int o = 0; o < 3; o++) {
            OUT0(o) = 0.f;
        }

    }
}

//Threading stuff
void *gstate_update_func(void *param) {
    if ( bno.setup() ) {
        int newTask = currentTask;
        char calibrationPath[128];
        snprintf(calibrationPath, 127, "%s/.bnoCalibration", getenv("HOME"));
        bnoCalibration_t calData;
        FILE *fp;
        while ( currentTask.load( std::memory_order_relaxed ) != TASK_STOP ) {
            currentTask = newTask;
            while ( currentTask.load( std::memory_order_relaxed ) == TASK_RUN ) {
                //TODO: Fetch all data from IMU (4x3 floats)
                //bno.readIMU(gData);
                std::this_thread::sleep_for( std::chrono::duration<float, std::milli>(INTERVAL)  );
            }
            while ( currentTask.load( std::memory_order_relaxed ) == TASK_CALIBRATE_IDLE ) {
                std::this_thread::sleep_for( std::chrono::duration<float, std::milli>(IDLE_INTERVAL)  );
            }
            switch ( currentTask.load( std::memory_order_relaxed ) ) {
            case TASK_CALIBRATE_1:
                printf("BNO: Calibrating, neutral position\n");
                bno.getNeutralGravity();
                newTask = TASK_CALIBRATE_IDLE;
                break;

            case TASK_CALIBRATE_2:
                printf("BNO: Calibrating, tilted down\n");
                bno.getDownGravity();
                newTask = TASK_RUN;
                break;
            case TASK_SAVE:
                Print("Saving calibration\n");
                  fp = fopen(calibrationPath, "wb+");
                  if (fp != NULL) {
                    bno.getCalibration(calData);
                    fwrite(&calData, sizeof(bnoCalibration_t), 1, fp);
                    fclose(fp);
                  } else {
                    printf("BNO: Couldn't open file for writing\n");
                  }
                newTask = TASK_RUN;
                break;

            case TASK_LOAD:
                Print("Loading calibration\n");
                  fp = fopen(calibrationPath, "rb");
                  if (fp != NULL) {
                    fread(&(calData), sizeof(bnoCalibration_t), 1, fp);
                    fclose(fp);
                    bno.setCalibration(calData);
                  } else {
                    printf("BNO: Couldn't open file for reading\n");
                  }
                newTask = TASK_RUN;
                break;
            }

        }
    } else {
		printf("Error initialising BNO055\n");
    }
    //We need to return something
    return NULL;
}

pthread_t bnoThread;

PluginLoad(BNO)
{

    ft = inTable;
    currentTask = TASK_LOAD;
    //TODO: do we need to start thread here?
    pthread_create(&bnoThread, NULL, gstate_update_func, &gData);


    DefineSimpleUnit(BNO);
}

C_LINKAGE SC_API_EXPORT void unload(InterfaceTable *inTable)
{
    currentTask = TASK_STOP;
    pthread_join(bnoThread, NULL);
}
