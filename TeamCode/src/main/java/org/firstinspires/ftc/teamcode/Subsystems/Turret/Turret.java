package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.Util;
@Config
public class Turret extends WSubsystem {
//    public static double kp = 0.011;
//    public static double ki = 0.01;
//
//    public static double kd = 0.00017;
//    public static double ff = 0.05;
//    public static double kp = 0.017;
//    public static double ki = 0;
//
//    public static double kd = 0.0002479;
//    public static double ff = 0.06;


    DcMotorEx turretMotor;
    WEncoder TurretEncoder;
    public WActuatorGroup TurretController;
    PIDController visionPID;

    @Override
    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class,"turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurretEncoder = new WEncoder(new MotorEx(hardwareMap, "turretMotor").encoder);

        TurretController = new WActuatorGroup(this::getTurretAngle,turretMotor)
                .setPIDController(new PIDController(0.017,0.0,0.0002479))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0.06);
        TurretEncoder.encoder.reset();
        visionPID = new PIDController(0,0,0);
    }
    public double getTurretAngle(){
        double encoderRots= TurretEncoder.getPosition()/28;
        return encoderRots*(1.0/(((1+(46.0/17.0))) * (1+(46.0/17.0))))*(16.0/112.0)*360;
    }

    double handleTurretSetpoint(double setpoint){
        if(setpoint<-120){
            return -120;
        } else if (setpoint>120) {
            return 120;
        }
        return setpoint;
    }
    public void setTurretAngle(double Angle){
        TurretController.setTargetPosition(handleTurretSetpoint(Angle));
    }

    @Override
    public void periodic() {
//        TurretController.setPID(kp,ki,kd);
//        TurretController.setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,ff);
        TurretController.periodic();
    }
    public void setVoltage(DoubleSupplier voltage){
        TurretController.setVoltageSupplier(voltage);
    }

    @Override
    public void read() {
        TurretController.read();
    }

    @Override
    public void write() {
        TurretController.write();
    }

    @Override
    public void reset() {

    }
}
