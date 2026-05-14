package org.firstinspires.ftc.teamcode.Subsystems.Revolver;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public static double normalKp = 0.0065;
    public static double normalKi = 0.0;
    public static double normalKd = 0.00009;
    public static double normalFeedforward = 0.034;
    public static double aggressiveKp = 0.015;
    public static double aggressiveKi = 0.01;
    public static double aggressiveKd = 0.00025;
    public static double aggressiveFeedforward = 0.033;
    DcMotorEx revolverMotor;
    static WEncoder RevolverEncoder;
    public static WActuatorGroup RevolverController;
    public RevColorSensorV3 shooterSensor;
    public static DigitalChannel feederSensor;
    public static DigitalChannel rightSensor;
    public static DigitalChannel leftSensor;
    public static DigitalChannel intakeSensor;
    private static boolean encoderReferenceSet = false;
    private Boolean aggressiveGainsActive = null;

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
        aggressiveGainsActive = null;
        if(isAuto || !encoderReferenceSet){
            RevolverEncoder.encoder.reset();
            encoderReferenceSet = true;
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
        aggressiveGainsActive = null;
        if(!encoderReferenceSet){
            RevolverEncoder.encoder.reset();
            encoderReferenceSet = true;
        }
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
        boolean fullHoldMode = (Superstructure.slot0.IsthereBall()&&Superstructure.slot1.IsthereBall()&&Superstructure.slot2.IsthereBall())
                &&!Superstructure.isIntakwing;
        boolean aggressiveMode = fullHoldMode || Superstructure.isShooting();
        applyGains(aggressiveMode);
        RevolverController.periodic();
    }

    private void applyGains(boolean aggressiveMode) {
        if (aggressiveGainsActive != null && aggressiveGainsActive == aggressiveMode) {
            return;
        }

        if(aggressiveMode){
            RevolverController.setPID(aggressiveKp,aggressiveKi,aggressiveKd);
            RevolverController.setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT, aggressiveFeedforward);
        }else{
            RevolverController.setPID(normalKp,normalKi,normalKd);
            RevolverController.setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT, normalFeedforward);
        }
        aggressiveGainsActive = aggressiveMode;
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
        if (RevolverController != null && RevolverEncoder != null) {
            setRevolverAngle(getRevolverAngle().getDegrees());
        }
        aggressiveGainsActive = null;
    }
}
