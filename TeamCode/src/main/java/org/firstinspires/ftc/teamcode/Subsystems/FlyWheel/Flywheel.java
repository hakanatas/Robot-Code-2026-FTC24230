package org.firstinspires.ftc.teamcode.Subsystems.FlyWheel;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.wrappers.WVelocityGroup;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.Util;
@Config
public class Flywheel extends WSubsystem {
    public static double kp = 0.08;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double ks = 4.02951246;
    public static double kv = 0.00200608;
    public static double ka = 0.00002313;
    DcMotorEx masterShooterMotor;
    DcMotorEx slaveShooterMotor;
     WVelocityGroup VelocityController;
    WEncoder encoder;
    @Override
    public void init(HardwareMap hardwareMap) {
        masterShooterMotor = hardwareMap.get(DcMotorEx.class,"masterShooterMotor");
        slaveShooterMotor = hardwareMap.get(DcMotorEx.class,"slaveShooterMotor");
        masterShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slaveShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        masterShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slaveShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        encoder = new WEncoder(new MotorEx(hardwareMap, "slaveShooterMotor").encoder);
        VelocityController = new WVelocityGroup(encoder,masterShooterMotor,slaveShooterMotor)
                .setPIDController(new PIDController(kp,ki,kd))
                .setFeedforwardSimple(ks,kv,ka);
    }
    public boolean IsAtSetpoint(){
        return Util.epsilonEquals(getShooterRPM(),VelocityController.getTargetVelocity(),100);
    }
    public void setSetpointRPM(double rpm){
        VelocityController.setTargetVelocity(rpm);
    }
    public double getShooterRPM(){
        return VelocityController.getVelocity();
    }
    public double getTargetRPM(){
        return VelocityController.getTargetVelocity();
    }
    @Override
    public void periodic() {
        VelocityController.setFeedforwardSimple(ks,kv,ka);
        VelocityController.setPID(kp,ki,kd);
        VelocityController.periodic();
    }
    public void setVoltage(DoubleSupplier voltage){
        VelocityController.setVoltageSupplier(voltage);
    }

    @Override
    public void read() {
        VelocityController.read();
    }

    @Override
    public void write() {
        VelocityController.write();
    }

    @Override
    public void reset() {

    }
}
