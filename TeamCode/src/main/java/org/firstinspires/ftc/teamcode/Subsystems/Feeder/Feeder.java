package org.firstinspires.ftc.teamcode.Subsystems.Feeder;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

public class Feeder extends WSubsystem {

    CRServo lFeederMotor;
    CRServo rFeederMotor;
    CRServo blFeederMotor;
    CRServo brFeederMotor;

    @Override
    public void init(HardwareMap hardwareMap) {
        lFeederMotor = hardwareMap.get(CRServo.class,"lFeederMotor");
        lFeederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rFeederMotor = hardwareMap.get(CRServo.class,"rFeederMotor");
        rFeederMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blFeederMotor = hardwareMap.get(CRServo.class, "blFeederMotor");
        blFeederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brFeederMotor = hardwareMap.get(CRServo.class, "brFeederMotor");
        brFeederMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setFeedersMotor(double speed){
        lFeederMotor.setPower(speed);
        rFeederMotor.setPower(speed);
        blFeederMotor.setPower(speed);
        brFeederMotor.setPower(speed);
    }
    @Override
    public void periodic() {}

    @Override
    public void read() {}

    @Override
    public void write() {}

    @Override
    public void reset() {
        if (lFeederMotor != null) {
            setFeedersMotor(0);
        }
    }
}
