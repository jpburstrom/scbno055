BNO : MultiOutUGen {
    /*
    Channels:
	0: Accel (xyz)
    1: Gyro (xyz)
    2: Mag (xyz)
    3: Orientation (Roll, Pitch, Yaw)
    */
    *kr {
		arg channel = 0, calibrate = 0, load = 0, save = 0;

        ^this.multiNew('control', channel, calibrate, load, save)
    }

	init {|...theInputs|
		inputs = theInputs;
		^this.initOutputs(3, rate);
	}

    *accelKr {
        arg calibrate = 0, load = 0, save = 0;
        ^this.kr(0, calibrate, load, save);
    }

    *gyroKr {
        arg calibrate = 0, load = 0, save = 0;
        ^this.kr(1, calibrate, load, save);
    }

    *magKr {
        arg calibrate = 0, load = 0, save = 0;
        ^this.kr(2, calibrate, load, save);
    }

    *orientationKr {
        arg calibrate = 0, load = 0, save = 0;
        ^this.kr(3, calibrate, load, save);
    }

}
