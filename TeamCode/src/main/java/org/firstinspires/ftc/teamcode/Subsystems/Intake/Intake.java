package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

public class Intake extends WSubsystem {
    CRServo masterIntakeMotor;
    CRServo slaveIntakeMotor;
    CRServo downIntakeMotor;
    CRServo slaveSlaveIntakeMotor;

    @Override
    public void init(HardwareMap hardwareMap) {
        masterIntakeMotor = hardwareMap.get(CRServo.class,"masterIntakeMotor");
        masterIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slaveIntakeMotor = hardwareMap.get(CRServo.class,"slaveIntakeMotor");
        slaveIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slaveSlaveIntakeMotor = hardwareMap.get(CRServo.class,"slaveslaveIntakeMotor");
        slaveSlaveIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        downIntakeMotor = hardwareMap.get(CRServo.class,"downIntakeMotor");
        downIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setIntakeMotor(double speed){
        slaveSlaveIntakeMotor.setPower(speed);
        masterIntakeMotor.setPower(speed);
        slaveIntakeMotor.setPower(speed);
        downIntakeMotor.setPower(speed);

    }
    public double getPower(){
        return masterIntakeMotor.getPower();
    }

    @Override
    public void periodic() {}

    @Override
    public void read() {}

    @Override
    public void write() {}

    @Override
    public void reset() {}
}
