package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.FlyWheel.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Hood.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Revolver.Revolver;
import org.firstinspires.ftc.teamcode.Subsystems.Revolver.Slot;
import org.firstinspires.ftc.teamcode.Subsystems.Tilter.Tilter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.lib.Constants;
import org.firstinspires.ftc.teamcode.lib.math.InterpolatingDouble;

import java.util.ArrayDeque;
import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.Util;
import edu.wpi.first.wpilibj.Timer;
@Config
public class Superstructure{
    static HardwareMap hardwaremap;
//    public static double ANGLE = 0;
//    public static double RPM = 0;
//    public static double kp = 0.9;
//    public static double ki = 0.0;
//    public static double kd = 0.0;
    public static double revolangle = 0;
    public static Revolver revolver=new Revolver();

    public static Intake intake = new Intake();

    public static Feeder feeder = new Feeder();

    public static Flywheel flywheel = new Flywheel();

    public static Turret turret = new Turret();
    public static Tilter tilter= new Tilter();

    public static Vision vision = new Vision();

    public static Hood hood=new Hood();
    static RevBlinkinLedDriver LEDS;
    static double hint = 0;
    static double turretHintAdjustment = 27;
    static Timer wiggleTimer = new Timer();
    static Timer tvMissing = new Timer();
    static int rotDir = -1; //-1 Sol, 1 Sag
    static double globalEr = 0;
    public static double revAngle = 0;
    public static boolean wiggle = false;
    public static boolean autoWiggle = false;

    public static boolean manualHoodControl = false;

    public static boolean manualTurretControl = false;
    public static boolean flywheelIdleMode = true;
    public static double flywheelIdleRPM = 2800.0;
    private static boolean isShooting = false;
    private static double shootingHoodOffsetDeg = 0.0;
    public static boolean isIntakwing = false;
    public static boolean isTilted = false;
    public static boolean manualMode = false;
    public static boolean corrected = true;

    public static boolean isauto = false;
    public static double angularvel = 0;
    public static double turretAimDeadbandDeg = 0.0;
    public static PIDController visionpid = new PIDController(0.8,0,0);
    public static Slot slot0=new Slot(0);
    public static Slot slot1=new Slot(120);
    public static Slot slot2=new Slot(-120);
    public static Slot currentSlot=slot0;
    private static final double REVOLVER_SLOT_TOLERANCE_DEG = 5.0;

    public static void init(HardwareMap hardwareMap) {
        revolver.init(hardwareMap, isauto);
        intake.init(hardwareMap);
        feeder.init(hardwareMap);
        flywheel.init(hardwareMap);
        turret.init(hardwareMap);
        vision.init(hardwareMap);
        hood.init(hardwareMap);
        tilter.init(hardwareMap);
        hardwaremap=hardwareMap;
        LEDS = hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");
        manualHoodControl = false;
        manualTurretControl =false;
        flywheelIdleMode = true;
        corrected = true;
    }

    public static void periodic() {
//        visionpid.setPID(kp,ki,kd);
        if(!manualMode&&!manualHoodControl){
            hood.setHoodAngle(Constants.ShootingParams.kHoodMap.getInterpolated(new InterpolatingDouble(Superstructure.vision.ty)).value + shootingHoodOffsetDeg);
//            hood.setHoodAngle(ANGLE);
        }else if(manualMode&&!manualTurretControl){
            hood.setHoodAngle(18);
        }

//        Revolver.setRevolverAngle(revolangle);

        if (flywheelIdleMode && !isShooting) {
            flywheel.setSetpointRPM(flywheelIdleRPM);
        }

        if(!manualMode&&!manualTurretControl){
            double aimTx = Math.abs(vision.tx) < turretAimDeadbandDeg ? 0.0 : vision.tx;
            double compensatetx = visionpid.calculate(aimTx,0);
            double error;
            if (vision.tv) {
                if(angularvel>0){
                    error = turret.getTurretAngle() + compensatetx +(-AngleUnit.DEGREES.fromRadians(angularvel)* vision.tl);
                }else{
                    error = turret.getTurretAngle() + compensatetx;
                }
                error = new Rotation2d(Units.degreesToRadians(error)).getDegrees();
                turret.setTurretAngle(error);
                globalEr = error;
                tvMissing.reset();
                tvMissing.stop();
            }else{
                tvMissing.start();
                if(tvMissing.get() > 0.25) {
                    if (rotDir == -1) {
                        hint = -15;
                    } else {
                        hint = 15;
                    }

                    if (turret.getTurretAngle() > 50) {
                        rotDir = -1;
                    } else if (turret.getTurretAngle() < -50) {
                        rotDir = 1;
                    }

                    turretHintAdjustment = turret.getTurretAngle();
                    turretHintAdjustment += hint;
                    turret.setTurretAngle(turretHintAdjustment);
                }
            }
        }else if(manualMode&&!manualTurretControl){
            turret.setTurretAngle(0);
        }
//        if(manualTurretControl){
//            turret.setTurretAngle(90);
//        }else{
//            turret.setTurretAngle(0);
//        }

        boolean revolverEmpty=!slot0.IsthereBall()&&!slot2.IsthereBall()&&!slot1.IsthereBall();
        boolean revolverFull=slot0.IsthereBall()&&slot2.IsthereBall()&&slot1.IsthereBall();

        if(isShooting){
            if(flywheel.IsAtSetpoint() && hood.IsAtSetpoint() && vision.tv && Util.epsilonEquals(turret.getTurretAngle(),turret.TurretController.getTargetPosition(), 3)&&!revolverEmpty){
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }else if(revolverEmpty){
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            }
            else{
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
            }
        }else if(!isShooting && !isIntakwing&&!isTilted){
            if(!manualMode){
                if(vision.tv){
                    LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                }else {
                    LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                }
            }else{
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            }
        }else if(isIntakwing){
            if(!revolverFull){
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE);
            }else{
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_STROBE);
            }
        }else if(isTilted){
            LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_MEDIUM);
        }

        turret.periodic();
        vision.periodic();
        revolver.periodic();
        intake.periodic();
        feeder.periodic();
        flywheel.periodic();
        hood.periodic();
        tilter.periodic();
    }
    public static void setVoltage(double voltage){
        flywheel.setVoltage(()->voltage);
        revolver.setVoltage(()->voltage);
        turret.setVoltage(()->voltage);
    }

    public static void setIntakeSystem(double speed){
        intake.setIntakeMotor(speed);
    }

    public static void pointTurretTowardAprilTagSearch(double angle) {
        if (vision.tv || manualMode || manualTurretControl) {
            return;
        }

        tvMissing.reset();
        tvMissing.stop();
        turretHintAdjustment = angle;
        turret.setTurretAngle(angle);
    }

//    public static boolean isRevolverReady() {
//        if (!revolverInitialized) {
//            revAngle = Revolver.getRevolverAngle().getDegrees();
//            revolverInitialized = true;
//        }
//
//        boolean currentWiggle = wiggle || autoWiggle;
//        if (intake.getPower()==0) {
//            adjusted = false;
//            if (!corrected) {
//                revAngle += 60;
//                corrected = true;
//            }
//        if (!Revolver.get2Status()) {
//            if (currentWiggle) {
//                wiggleTimer.start();
//                if (wiggleTimer.get() < 0.25) {
//                    Revolver.setRevolverAngle(revAngle + 15);
//                } else if (wiggleTimer.get() > 0.5 && wiggleTimer.get() < 0.75) {
//                    Revolver.setRevolverAngle(revAngle - 15);
//                } else if (wiggleTimer.get() > 0.75) {
//                    wiggleTimer.reset();
//                    Revolver.setRevolverAngle(revAngle);
//                }
//            } else {
//                wiggleTimer.reset();
//                wiggleTimer.stop();
//                Revolver.setRevolverAngle(revAngle);
//            }
//            isRevReady = false;
//                if (Util.epsilonEquals(Revolver.getRevolverAngle().getDegrees(), revAngle, 5)) {
//                    if (Revolver.get0Status() && Revolver.get1Status() && !Revolver.get2Status()) {
//                        revAngle -= 120;
//                    } else if (Revolver.get0Status()) {
//                        revAngle += 120;
//                    } else if (Revolver.get1Status()) {
//                        revAngle -= 120;
//                    }
//                }
//        } else {
//            isRevReady = true;
//            if (currentWiggle) {
//                wiggleTimer.start();
//                if (wiggleTimer.get() < 0.25) {
//                    Revolver.setRevolverAngle(revAngle + 15);
//                } else if (wiggleTimer.get() > 0.5 && wiggleTimer.get() < 0.75) {
//                    Revolver.setRevolverAngle(revAngle - 15);
//                } else if (wiggleTimer.get() > 0.75) {
//                    wiggleTimer.reset();
//                    Revolver.setRevolverAngle(revAngle);
//                }
//            } else {
//                wiggleTimer.reset();
//                wiggleTimer.stop();
//                Revolver.setRevolverAngle(revAngle);
//            }
//        }
//    }else{
//            corrected=false;
//            if(!adjusted){
//                if (Util.epsilonEquals(Revolver.getRevolverAngle().getDegrees(), revAngle, 5)) {
//                    if (Revolver.get0Status()&&!Revolver.get1Status()) {
//                        revAngle += 60;
//                        adjusted=true;
//                    } else if (!Revolver.get0Status()&&Revolver.get1Status()) {
//                        revAngle -= 60;
//                        adjusted=true;
//                    } else if (!Revolver.get0Status()&&!Revolver.get1Status()) {
//                        revAngle += 60;
//                        adjusted=true;
//                    }else{
//                        revAngle += 180;
//                        adjusted=true;
//                    }}
//            }
//            if (Util.epsilonEquals(Revolver.getRevolverAngle().getDegrees(), revAngle, 5)) {
//                if(Revolver.getIntakeStatus()){
//                    revAngle+=120;
//                }}
//            Revolver.setRevolverAngle(revAngle);
//        }
//        return isRevReady;
//    }

    public static boolean isRevolverReady() {
        if(intake.getPower()==0){
            updateShootSlotsFromSensors();
            revAngle = getNextShootRevolverAngle();
        }else{
            if((slot0.IsthereBall()&& slot1.IsthereBall()&&slot2.IsthereBall())){
                revAngle=slot0.getAngle();
            }else {
                if((slot0.IsthereBall()&& slot1.IsthereBall())){
                    revAngle=Rotation2d.fromDegrees(slot2.getAngle()+180).getDegrees();
                } else if (slot0.IsthereBall()&& slot2.IsthereBall()) {
                    revAngle=Rotation2d.fromDegrees(slot1.getAngle()-180).getDegrees();
                } else if (slot1.IsthereBall()&& slot2.IsthereBall()) {
                    revAngle=Rotation2d.fromDegrees(slot0.getAngle()+180).getDegrees();
                }else{
                    if(slot0.IsthereBall()){
                        revAngle=Rotation2d.fromDegrees(slot1.getAngle()-180).getDegrees();
                    } else if (slot1.IsthereBall()) {
                        revAngle=Rotation2d.fromDegrees(slot0.getAngle()+180).getDegrees();
                    }else if (slot2.IsthereBall()) {
                        revAngle=Rotation2d.fromDegrees(slot0.getAngle()+180).getDegrees();
                    }else{
                        revAngle=Rotation2d.fromDegrees(slot0.getAngle()+180).getDegrees();
                    }
                }
            }
            if(Util.epsilonEquals(MathUtil.inputModulus(Revolver.getRevolverAngle().getDegrees(),-180,180) , revAngle, 5)){
                if(revAngle==Rotation2d.fromDegrees(slot0.getAngle()+180).getDegrees()){
                    slot0.setIsthereBall(Revolver.getIntakeStatus());
                } else if (revAngle== Rotation2d.fromDegrees(slot1.getAngle()-180).getDegrees()) {
                    slot1.setIsthereBall(Revolver.getIntakeStatus());
                }else if (revAngle==Rotation2d.fromDegrees(slot2.getAngle()+180).getDegrees()) {
                    slot2.setIsthereBall(Revolver.getIntakeStatus());
                }
            }
        }
        Revolver.setRevolverAngle(revAngle);
        return currentSlot.IsthereBall();
    }

    private static void updateShootSlotsFromSensors() {
        double currentAngle = MathUtil.inputModulus(Revolver.getRevolverAngle().getDegrees(), -180, 180);
        double targetAngle = MathUtil.inputModulus(Revolver.RevolverController.getTargetPosition(), -180, 180);

        if(!Util.epsilonEquals(currentAngle, targetAngle, REVOLVER_SLOT_TOLERANCE_DEG)){
            return;
        }

        if(isSameSlotTarget(targetAngle, slot0.getAngle())){
            slot0.setIsthereBall(Revolver.get2Status());
            slot1.setIsthereBall(Revolver.get0Status());
            slot2.setIsthereBall(Revolver.get1Status());
            currentSlot=slot0;
        } else if (isSameSlotTarget(targetAngle, slot1.getAngle())) {
            slot0.setIsthereBall(Revolver.get1Status());
            slot1.setIsthereBall(Revolver.get2Status());
            slot2.setIsthereBall(Revolver.get0Status());
            currentSlot=slot1;
        }else if (isSameSlotTarget(targetAngle, slot2.getAngle())) {
            slot0.setIsthereBall(Revolver.get0Status());
            slot1.setIsthereBall(Revolver.get1Status());
            slot2.setIsthereBall(Revolver.get2Status());
            currentSlot=slot2;
        }
    }

    private static boolean isSameSlotTarget(double targetAngle, double slotAngle) {
        return Util.epsilonEquals(
                MathUtil.inputModulus(targetAngle, -180, 180),
                MathUtil.inputModulus(slotAngle, -180, 180),
                0.5
        );
    }

    private static double getNextShootRevolverAngle() {
        if(slot0.IsthereBall()&& slot1.IsthereBall()&&slot2.IsthereBall()){
            return slot0.getAngle();
        }else {
            if(slot0.IsthereBall()&& slot1.IsthereBall()){
                return slot0.getAngle();
            } else if (slot0.IsthereBall()&& slot2.IsthereBall()) {
                return slot0.getAngle();
            } else if (slot1.IsthereBall()&& slot2.IsthereBall()) {
                return slot1.getAngle();
            }else{
                if(slot0.IsthereBall()){
                    return slot0.getAngle();
                } else if (slot1.IsthereBall()) {
                    return slot1.getAngle();
                }else if (slot2.IsthereBall()) {
                    return slot2.getAngle();
                }else{
                    return slot0.getAngle();
                }
            }
        }
    }

    public static void resetRevolverWithSensors() {
        revolver.zeroToCurrentPosition();

        boolean shooterHasBall = Revolver.get2Status();
        boolean rightHasBall = Revolver.get0Status();
        boolean leftHasBall = Revolver.get1Status();

        slot0.setIsthereBall(shooterHasBall);
        slot1.setIsthereBall(rightHasBall);
        slot2.setIsthereBall(leftHasBall);
        currentSlot = slot0;

        if (shooterHasBall) {
            revAngle = slot0.getAngle();
        } else if (rightHasBall && leftHasBall) {
            revAngle = slot2.getAngle();
        } else if (rightHasBall) {
            revAngle = slot1.getAngle();
        } else if (leftHasBall) {
            revAngle = slot2.getAngle();
        } else {
            revAngle = slot0.getAngle();
        }

        Revolver.setRevolverAngle(revAngle);
    }

    public static void setShootSystem(){
        setShootSystem(1.0);
    }

    public static void setShootSystem(double rpmMultiplier){
        setShootSystem(rpmMultiplier, 0.0);
    }

    public static void setShootSystem(double rpmMultiplier, double hoodOffsetDeg){
        isShooting = true;
        shootingHoodOffsetDeg = hoodOffsetDeg;
        if(!manualMode){
            flywheel.setSetpointRPM(Constants.ShootingParams.kRPMMap.getInterpolated(new InterpolatingDouble(Superstructure.vision.ty)).value * rpmMultiplier);
        }else{
            flywheel.setSetpointRPM(3500 * rpmMultiplier);
        }
//        flywheel.setSetpointRPM(RPM);
        if(flywheel.IsAtSetpoint() && hood.IsAtSetpoint() && vision.tv && Util.epsilonEquals(turret.getTurretAngle(),turret.TurretController.getTargetPosition(), 3)){
            feeder.setFeedersMotor(1);
        }else{
            feeder.setFeedersMotor(0);
        }
    }

    public static void stopShooting() {
        isShooting = false;
        autoWiggle = false;
        shootingHoodOffsetDeg = 0.0;
        feeder.setFeedersMotor(0);
    }

    public static void setFlywheelIdleMode(boolean enabled) {
        flywheelIdleMode = enabled;
        if (!enabled) {
            isShooting = false;
        }
    }

    public static void read() {
        revolver.read();
        intake.read();
        feeder.read();
        flywheel.read();
        turret.read();
        vision.read();
        hood.read();
        tilter.read();
    }

    public static void write() {
        revolver.write();
        intake.write();
        feeder.write();
        flywheel.write();
        turret.write();
        vision.write();
        hood.write();
        tilter.write();
    }

    public static void reset() {
        slot0.setIsthereBall(false);
        slot1.setIsthereBall(false);
        slot2.setIsthereBall(false);
        currentSlot = slot0;
        revAngle = slot0.getAngle();
        revolver.reset();
        intake.reset();
        feeder.reset();
        flywheel.reset();
        turret.reset();
        vision.reset();
        hood.reset();
        tilter.reset();
        manualHoodControl = false;
        manualTurretControl = false;
        flywheelIdleMode = true;
        isShooting = false;
        autoWiggle = false;
        shootingHoodOffsetDeg = 0.0;
        turretAimDeadbandDeg = 0.0;
    }
}
