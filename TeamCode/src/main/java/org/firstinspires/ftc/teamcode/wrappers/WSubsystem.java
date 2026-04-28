package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class WSubsystem {
    public abstract void init(HardwareMap hardwareMap);
    public void init(HardwareMap hardwareMap, boolean a) {
    };

    public abstract void periodic();
    public abstract void read();
    public abstract void write();
    public abstract void reset();
}
