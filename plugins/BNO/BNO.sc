BNO : MultiOutUGen {
    /*
    Channels:
    0: Accel
    1: Gyro
    2: Mag
    3: Orientation (Roll, Pitch, Yaw)
    4: Roll
    5: Pitch
    6: Yaw
    */
    *kr {
        arg channel = 0, mul = 1.0, add = 0;

        ^this.multiNew('control', channel).madd(mul, add)
    }

    *accelKr {
        arg mul = 1.0, add = 0;
        ^this.kr(0, mul, add);
    }

    *gyroKr {
        arg mul = 1.0, add = 0;
        ^this.kr(1, mul, add);
    }

    *magKr {
        arg mul = 1.0, add = 0;
        ^this.kr(2, mul, add);
    }

    *orientationKr {
        arg mul = 1.0, add = 0;
        ^this.kr(3, mul, add);
    }

    *rollKr {
        arg mul = 1.0, add = 0;
        ^this.kr(4, mul, add);
    }

    *pitchKr{
        arg mul = 1.0, add = 0;
        ^this.kr(5, mul, add);
    }

    *yawKr{
        arg mul = 1.0, add = 0;
        ^this.kr(6, mul, add);
    }

    *calibrateNeutral { |server|
        (server ? Server.default).sendMsg(\cmd, \bnoCmd, 1);
    }

    *calibrateDown { |server|
        (server ? Server.default).sendMsg(\cmd, \bnoCmd, 2);
    }

    *saveCalibration { |path, server|
        path = path ?? { Platform.userConfigDir +/+ "bnoCalibration.bin" };
        //No error checking here -- server might be remote
        (server ? Server.default).sendMsg(\cmd, \bnoCmd, 3, path);
    }

    *loadCalibration { |path, server|
        path = path ?? { Platform.userConfigDir +/+ "bnoCalibration.bin" };
        //No error checking here -- server might be remote
        (server ? Server.default).sendMsg(\cmd, \bnoCmd, 4, path);
    }

}
