#include <pthread.h>
#include <atomic>
#include "SC_Lock.h"
#include "SC_PlugIn.h"

#include "imu/SC_BNO055.h"

// written with reference to the chapter "Writing Unit Generator Plug-ins" in The SuperCollider Book
// and also http://doc.sccode.org/Guides/WritingUGens.html accessed March 2, 2015
//

//Read interval
#define INTERVAL 2.5
#define IDLE_INTERVAL 100

struct BNO : public Unit {
    //Outputs 3 values
    static const int ACCEL = 0;
    static const int GYRO = 1;
    static const int MAG = 2;
    static const int ORIENTATION = 3;
    //Outputs single values
    static const int ROLL = 4;
    static const int PITCH  = 5;
    static const int YAW = 6;

    int channel = ORIENTATION;
    int outputs = 3;

};

struct BNOCmdData {
    //Current data state
    //Filename for save/load operations
    FILE *fp;
    bnoCalibration_t calData;
    char *filename;
};  

bnoState_t gData;

enum bnoTask {
    TASK_STOP = 0,
    TASK_RUN=2,
    TASK_CMD=8,
    TASK_CALIBRATE_NEUTRAL=9,
    TASK_CALIBRATE_DOWN=10,
    TASK_SAVE=11,
    TASK_LOAD=12
};

//These should correspond with class methods
enum bnoCmd {
    CMD_CALIBRATE_NEUTRAL = 1,
    CMD_CALIBRATE_DOWN,
    CMD_SAVE,
    CMD_LOAD
};

//Current command
std::atomic_int currentTask = { TASK_STOP };


// PLUGIN INTERFACE

extern "C" {
    void BNO_Ctor(BNO *unit);
    //void BNO_next_k1(BNO *unit, int numSamples);
    void BNO_next_k(BNO *unit, int numSamples);
}


static InterfaceTable *ft;

// PLUGIN IMPLEMENTATION
SC_BNO055 bno;

void BNO_Ctor(BNO *unit) {
    unit->channel = static_cast<int>(IN0(0));
    unit->outputs = (unit->channel > BNO::ORIENTATION) ? 1 : 3;

    SETCALC(BNO_next_k);
    BNO_next_k(unit, 1);
}

void BNO_next_k(BNO *unit, int numSamples) {

    float value[3];
    int outputs = unit->outputs;

    if (currentTask == TASK_RUN) {
        switch (unit->channel) {
            case BNO::GYRO:
                value[0] = gData.gyro.x();
                value[1] = gData.gyro.y();
                value[2] = gData.gyro.z();
                break;
            case BNO::MAG:
                value[0] = gData.mag.x();
                value[1] = gData.mag.y();
                value[2] = gData.mag.z();
                break;
            //Roll, Pitch, Yaw
            case BNO::ORIENTATION:
                value[0] = gData.orientation.x();
                value[1] = gData.orientation.y();
                value[2] = gData.orientation.z();
                break;
            case BNO::ROLL:
                value[0] = gData.orientation.x();
                break;
            case BNO::PITCH:
                value[0] = gData.orientation.y();
                break;
            case BNO::YAW:
                value[0] = gData.orientation.z();
                break;
            case BNO::ACCEL:
            default:
                value[0] = gData.accel.x();
                value[1] = gData.accel.y();
                value[2] = gData.accel.z();
                break;
        }
    } else {
        value[0] = value[1] = value[2] = 0;
    }

    for (int i = 0; i < numSamples ; i++) {
        for (int o = 0; o < outputs; o++) {
            OUT(o)[i] = value[o];
            //OUT(0) = 0;
            //OUT(1) = 0;
            //OUT(2) = 0;
        }
    }
        //    printf("roll:%f", gData.roll);
}


//Second stage, non-RT
bool cmdStage2(World* world, void* inUserData) {
    BNOCmdData* data = (BNOCmdData*)inUserData;
    switch ( currentTask.load( std::memory_order_relaxed ) ) {
        case TASK_CALIBRATE_NEUTRAL:
            printf("BNO: Calibrating, neutral position\n");
            bno.getNeutralGravity();
            currentTask = TASK_RUN;
            break;
        
        case TASK_CALIBRATE_DOWN:
            printf("BNO: Calibrating, tilted down\n");
            bno.getDownGravity();
            currentTask = TASK_RUN;
            break;

        case TASK_SAVE:
            data->fp = fopen(data->filename, "wb");
            if (data->fp != NULL) {
                bno.getCalibration(data->calData);
                fwrite(&(data->calData), sizeof(bno055Calibration_t), 1, data->fp);
                fclose(data->fp);
            } else {
                printf("BNO: Couldn't open file for writing\n");
            }
            currentTask = TASK_RUN;
            break;

        case TASK_LOAD:
            data->fp = fopen(data->filename, "rb");
            if (data->fp != NULL) {
                fread(&(data->calData), sizeof(bno055Calibration_t), 1, data->fp);
                fclose(data->fp);
                bno.setCalibration(data->calData);
            } else {
                printf("BNO: Couldn't open file for reading\n");
            }
            currentTask = TASK_RUN;
            break;
    }
    return true;
}

//Synchronous, sends completion message
bool cmdStage3(World* world, void* inUserData) {
    Print("Command completed\n");
    return true;
}


void cmdCleanup(World* world, void* inUserData) {
    BNOCmdData* cmdData = (BNOCmdData*)inUserData;
    RTFree(world, cmdData->filename);
    RTFree(world, cmdData);
}

void bnoCmdFunc(World *inWorld, void* inUserData, struct sc_msg_iter *args, void *replyAddr) {
    
    //Allocate a new instance of BNOCmdData
    BNOCmdData* data = (BNOCmdData*)RTAlloc(inWorld, sizeof(BNOCmdData));
    //Empty
    data->filename = 0;

    //Get arguments: First arg is command type
    int cmd = args->geti();
    //Second arg is a string, which might be a filename.
    const char *filename = args->gets();

    if (filename) {
        data->filename = (char*)RTAlloc(inWorld, strlen(filename)+1); // allocate space, free it in cmdCleanup.
        strcpy(data->filename, filename); // copy the string
    }

    int msgSize = args->getbsize();
    char* msgData = 0;
    if (msgSize) {
        // allocate space for completion message
        // scsynth will delete the completion message for you.
        msgData = (char*)RTAlloc(inWorld, msgSize);
        args->getb(msgData, msgSize); // copy completion message.
    }

    switch(cmd) {
        case CMD_CALIBRATE_NEUTRAL:
            currentTask = TASK_CALIBRATE_NEUTRAL;
            break;
        case CMD_CALIBRATE_DOWN:
            currentTask = TASK_CALIBRATE_DOWN;
            break;
        case CMD_SAVE:
            currentTask = TASK_SAVE;
            break;
        case CMD_LOAD:
            currentTask = TASK_LOAD;
            break;
        default:
            printf("BNO: No such command\n");
    }


    if ((currentTask & TASK_CMD) == TASK_CMD) {
        DoAsynchronousCommand(inWorld, replyAddr, "bnoCmd", (void*)data, (AsyncStageFn)cmdStage2, (AsyncStageFn)cmdStage3, NULL, cmdCleanup, msgSize, msgData);
    }

}

//Threading stuff

void *gstate_update_func(void *param) {
    if ( bno.setup() ) {

        while ( currentTask.load( std::memory_order_relaxed ) != TASK_STOP ) {
            while ( currentTask.load( std::memory_order_relaxed ) == TASK_RUN ) {
                bno.readIMU(gData);
                std::this_thread::sleep_for( std::chrono::duration<float, std::milli>(INTERVAL)  );
            }
            while ( (currentTask.load( std::memory_order_relaxed ) & TASK_CMD) == TASK_CMD  ) {
                std::this_thread::sleep_for( std::chrono::milliseconds( IDLE_INTERVAL ) );
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
    currentTask = TASK_RUN;
    //TODO: do we need to start thread here?
    pthread_create(&bnoThread, NULL, gstate_update_func, &gData);

    //bnoThread = std::thread(gstate_update_func);

    DefineSimpleUnit(BNO);

    DefinePlugInCmd("bnoCmd", bnoCmdFunc, 0);
}

C_LINKAGE SC_API_EXPORT void unload(InterfaceTable *inTable)
{
    currentTask = TASK_STOP;
    pthread_join(bnoThread, NULL);
}
