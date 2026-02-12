package org.firstinspires.ftc.teamcode.Subsystems.Revolver;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
@Config
public class Revolver extends WSubsystem {
    public static double kp = 0.0;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kff = 0.0;
    DcMotorEx revolverMotor;
    static WEncoder RevolverEncoder;
    public static WActuatorGroup RevolverController;
    public RevColorSensorV3 shooterSensor;
    public static DigitalChannel feederSensor;
    public static DigitalChannel rightSensor;
    public static DigitalChannel leftSensor;
    public static DigitalChannel intakeSensor;

    @Override
    public void init(HardwareMap hardwareMap,boolean isAuto) {
        revolverMotor = hardwareMap.get(DcMotorEx.class,"revolverMotor");
        revolverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        revolverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RevolverEncoder = new WEncoder(new MotorEx(hardwareMap,"revolverMotor").encoder);
        RevolverController = new WActuatorGroup(()-> getRevolverAngle().getDegrees(),revolverMotor)
                .setPIDController(new PIDController(0.02,0.0,0.0))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0.043)
                .enableContinuousInput(180,-180);
        //RevolverController.setMaxPower(0.375);
        RevolverController.setMaxPower(1);
        if(isAuto){
            RevolverEncoder.encoder.reset();
        }
        shooterSensor=hardwareMap.get(RevColorSensorV3.class,"shooterSensor");
        leftSensor= hardwareMap.get(DigitalChannel.class,"leftSensor");
        rightSensor= hardwareMap.get(DigitalChannel.class,"rightSensor");
        feederSensor= hardwareMap.get(DigitalChannel.class,"feederSensor");
        intakeSensor = hardwareMap.get(DigitalChannel.class, "intakeSensor");
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        revolverMotor = hardwareMap.get(DcMotorEx.class,"revolverMotor");
        revolverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        revolverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RevolverEncoder = new WEncoder(new MotorEx(hardwareMap,"revolverMotor").encoder);
        RevolverController = new WActuatorGroup(()-> getRevolverAngle().getDegrees(),revolverMotor)
                .setPIDController(new PIDController(0.02,0.0,0.0))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0.043)
                .enableContinuousInput(180,-180);
        //RevolverController.setMaxPower(0.375);
        RevolverController.setMaxPower(1);
        shooterSensor=hardwareMap.get(RevColorSensorV3.class,"shooterSensor");
        leftSensor= hardwareMap.get(DigitalChannel.class,"leftSensor");
        rightSensor= hardwareMap.get(DigitalChannel.class,"rightSensor");
        feederSensor= hardwareMap.get(DigitalChannel.class,"feederSensor");
        intakeSensor = hardwareMap.get(DigitalChannel.class, "intakeSensor");
    }

    public static Rotation2d getRevolverAngle(){
        double encoderRots= RevolverEncoder.getPosition();
        return new Rotation2d(encoderRots*(1.0/((((1+(46.0/17))) * (1+(46.0/11)) * 28)))*(16.0/30.0)*2*Math.PI);
    }
    public static boolean get0Status(){
        boolean status;
        status = rightSensor.getState();
        return status;
    }
//    01 digi to ic20 left
//    feeder to digi 23
//    digi 67 to 45
//    ic21 to digi 67
    public static boolean get1Status(){
        boolean status;
        status = leftSensor.getState();
        return status;
    }
    public static boolean get2Status(){
        boolean status;
        status = feederSensor.getState();
        return status;
    }
    public static boolean getIntakeStatus(){
        boolean status;
        status = intakeSensor.getState();
        return status;
    }
    public static void setRevolverAngle(double angle){
        RevolverController.setTargetPosition(angle);
    }
    public void setVoltage(DoubleSupplier voltage){
        RevolverController.setVoltageSupplier(voltage);
    }
    @Override
    public void periodic() {
        if((Superstructure.slot0.IsthereBall()&&Superstructure.slot1.IsthereBall()&&Superstructure.slot2.IsthereBall())
                &&!Superstructure.isIntakwing){
            RevolverController.setPID(0.015,0.01,0.00025);
            RevolverController.setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT, 0.033);

        }else{
            RevolverController.setPID(0.0065,0,0.00009);
            RevolverController.setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT, 0.034);
        }
        RevolverController.periodic();
    }

    @Override
    public void read() {
        RevolverController.read();
    }

    @Override
    public void write() {
        RevolverController.write();
    }

    @Override
    public void reset() {

    }
}
