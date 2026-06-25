package org.firstinspires.ftc.teamcode.red.teleop;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.pedropathing.math.Vector;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Revolver.Revolver;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.Subsystems.Tilter.Tilter;
import org.firstinspires.ftc.teamcode.lib.math.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.lib.joysticklib.TimeDelayedBoolean;
import org.firstinspires.ftc.teamcode.lib.joysticklib.Toggle;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "redSahaUzakHizTest")
public class redFieldDistanceRPMTest extends OpMode {
    private static final double ROBOT_RADIUS = 9;
    public static double LONG_DISTANCE_THRESHOLD_INCHES = 70.0;
    public static double LONG_DISTANCE_SHOT_RPM_MULTIPLIER = 0.95;
    public static double LONG_DISTANCE_LIMITED_HOOD_RPM_MULTIPLIER = 0.92;
    public static double LONG_DISTANCE_HOOD_OFFSET_DEG = 7.0;
    public static double MAX_HOOD_ANGLE_DEG = 32.0;
    public static double VISION_TURRET_DEADBAND_DEG = 0.0;
    private Follower follower;
    private TelemetryManager telemetryM;
    private FieldManager panelsField;
    private Style robotStyle;
    private Style historyStyle;
    private Pose savedPose;
    private boolean poseLoaded = false;
    private double shotDistanceInches = Double.NaN;
    private double shotRpmMultiplier = 1.0;
    private double shotHoodOffsetDeg = 0.0;
    private double shotHoodTargetDeg = Double.NaN;
    private double shotBaseHoodDeg = Double.NaN;
    private boolean shotHoodLimited = false;
    public TimeDelayedBoolean manualModeTimer = new TimeDelayedBoolean();
    public Toggle manualModeTrigger = new Toggle(false);
    public Toggle tilterTrigger=new Toggle(false);

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        Superstructure.manualHoodControl = false;
        Superstructure.isauto=false;
        Superstructure.init(hardwareMap);
        Superstructure.reset();
        Superstructure.turretAimDeadbandDeg = VISION_TURRET_DEADBAND_DEG;
        Superstructure.vision.LL3.pipelineSwitch(1);
        Superstructure.tilter.setState(Tilter.tilterState.IDLE);

        savedPose = PoseStorage.getSavedPose();
        follower.setStartingPose(savedPose);

        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        robotStyle = new Style("", "#4CAF50", 0.75);      // Yeşil - Robot
        historyStyle = new Style("", "#FF9800", 0.5);    // Turuncu - Geçmiş
    }

    @Override
    public void init_loop() {
        if (!poseLoaded) {
            telemetryM.debug("Starting from Auto position:");
            telemetryM.debug("Position", PoseStorage.getSavedPose());
            telemetryM.debug("X", savedPose.getX());
            telemetryM.debug("Y", savedPose.getY());
            telemetryM.debug("Heading (deg)", Math.toDegrees(savedPose.getHeading()));
            telemetryM.debug("Press PLAY when ready!");
            telemetryM.update();

            follower.update();
            drawOnDashboard();
        }
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        poseLoaded = true;
    }

    @Override
    public void loop() {
        follower.update();

        follower.setTeleOpDrive(
                0.85 * -gamepad1.left_stick_y,
                0.85 * -gamepad1.left_stick_x,
                0.5 * -gamepad1.right_stick_x,
                false
        );

        if(gamepad1.right_trigger>0.5){
            Superstructure.isIntakwing=true;
            Superstructure.setIntakeSystem(1);
        }else{
            if(gamepad1.triangle){
                Superstructure.isIntakwing=true;
                Superstructure.setIntakeSystem(-1);
            } else{
                Superstructure.isIntakwing=false;
                Superstructure.setIntakeSystem(0);
            }
        }

        if(gamepad1.right_bumper){
            updateShotAdjustments();
            Superstructure.setShootSystem(shotRpmMultiplier, shotHoodOffsetDeg);
        } else {
            shotRpmMultiplier = 1.0;
            shotHoodOffsetDeg = 0.0;
            shotHoodTargetDeg = Double.NaN;
            shotBaseHoodDeg = Double.NaN;
            shotHoodLimited = false;
            Superstructure.stopShooting();
        }

        if(gamepad1.touchpad){
            Superstructure.wiggle = true;
        } else {
            Superstructure.wiggle = false;
        }

        manualModeTrigger.update(manualModeTimer.update(gamepad1.dpad_right&&gamepad1.square,2));

        if(manualModeTrigger.getState()){
            Superstructure.manualMode =true;
        }else{
            Superstructure.manualMode =false;
        }

        tilterTrigger.update(gamepad1.ps);

        if(tilterTrigger.getState()){
            Superstructure.tilter.setState(Tilter.tilterState.DEPLOYED);
            Superstructure.isTilted=true;
        }else{
            Superstructure.tilter.setState(Tilter.tilterState.IDLE);
            Superstructure.isTilted=false;
        }

        Superstructure.setVoltage(hardwareMap.voltageSensor.iterator().next().getVoltage());

        Superstructure.isRevolverReady();

        drawOnDashboard();

        if (gamepad1.options) {
            follower.setPose(new Pose(0, 0, 0));
            PoseStorage.reset();
        }

        // Telemetry
        double turretAngleDeg = Superstructure.turret.getTurretAngle();
        double turretTargetDeg = Superstructure.turret.TurretController.getTargetPosition();
        double turretErrorDeg = turretTargetDeg - turretAngleDeg;
        double shooterRpm = Superstructure.flywheel.getShooterRPM();
        double shooterTargetRpm = Superstructure.flywheel.getTargetRPM();
        double hoodAngleDeg = Superstructure.hood.getHoodAngle();
        double hoodTargetDeg = Superstructure.hood.getTargetAngle();

        debugTelemetry("OpMode", "redSahaUzakHizTest");
        debugTelemetry("Vision TV", Superstructure.vision.tv);
        debugTelemetry("AprilTag ID", Superstructure.vision.ApriltagID);
        debugTelemetry("TX", Superstructure.vision.tx);
        debugTelemetry("TY", Superstructure.vision.ty);
        debugTelemetry("Target distance in", shotDistanceInches);
        debugTelemetry("Turret angle", turretAngleDeg);
        debugTelemetry("Turret target", turretTargetDeg);
        debugTelemetry("Turret error", turretErrorDeg);
        debugTelemetry("Turret ready 3deg", Math.abs(turretErrorDeg) <= 3.0);
        debugTelemetry("Turret deadband", Superstructure.turretAimDeadbandDeg);
        debugTelemetry("Shooter RPM", shooterRpm);
        debugTelemetry("Shooter target", shooterTargetRpm);
        debugTelemetry("Shooter error", shooterTargetRpm - shooterRpm);
        debugTelemetry("Shooter ready", Superstructure.flywheel.IsAtSetpoint());
        debugTelemetry("Hood angle", hoodAngleDeg);
        debugTelemetry("Hood target actual", hoodTargetDeg);
        debugTelemetry("Hood error", hoodTargetDeg - hoodAngleDeg);
        debugTelemetry("Hood ready", Superstructure.hood.IsAtSetpoint());
        debugTelemetry("Shot RPM x", shotRpmMultiplier);
        debugTelemetry("Shot base hood", shotBaseHoodDeg);
        debugTelemetry("Shot hood offset", shotHoodOffsetDeg);
        debugTelemetry("Shot hood target calc", shotHoodTargetDeg);
        debugTelemetry("Hood limited", shotHoodLimited);
        debugTelemetry("s0", Revolver.get0Status());
        debugTelemetry("s1", Revolver.get1Status());
        debugTelemetry("s2", Revolver.get2Status());
        debugTelemetry("rv", Revolver.getRevolverAngle().getDegrees());
        telemetryM.update();
        telemetry.update();

        Superstructure.read();
        Superstructure.periodic();
        Superstructure.write();
    }

    private void debugTelemetry(String caption, Object value) {
        telemetryM.debug(caption, value);
        telemetry.addData(caption, value);
    }

    private void drawOnDashboard() {
        try {
            // Pose geçmişini çiz
            drawPoseHistory();

            // Mevcut robot pozisyonunu çiz
            drawRobot(follower.getPose(), robotStyle);

            // Paketi gönder
            panelsField.update();
        } catch (Exception e) {
            telemetry.addData("Drawing Error", e.getMessage());
        }
    }

    private void updateShotAdjustments() {
        shotDistanceInches = Superstructure.vision.getTargetDistanceInches();
        if (Superstructure.vision.tv
                && !Double.isNaN(shotDistanceInches)
                && shotDistanceInches >= LONG_DISTANCE_THRESHOLD_INCHES) {
            shotBaseHoodDeg = org.firstinspires.ftc.teamcode.lib.Constants.ShootingParams.kHoodMap
                    .getInterpolated(new InterpolatingDouble(Superstructure.vision.ty)).value;
            double availableHoodOffsetDeg = Math.max(0.0, MAX_HOOD_ANGLE_DEG - shotBaseHoodDeg);

            shotHoodOffsetDeg = Math.min(LONG_DISTANCE_HOOD_OFFSET_DEG, availableHoodOffsetDeg);
            shotHoodTargetDeg = shotBaseHoodDeg + shotHoodOffsetDeg;
            shotHoodLimited = shotHoodOffsetDeg < LONG_DISTANCE_HOOD_OFFSET_DEG;
            shotRpmMultiplier = shotHoodLimited
                    ? LONG_DISTANCE_LIMITED_HOOD_RPM_MULTIPLIER
                    : LONG_DISTANCE_SHOT_RPM_MULTIPLIER;
            return;
        }

        shotRpmMultiplier = 1.0;
        shotHoodOffsetDeg = 0.0;
        shotHoodTargetDeg = Double.NaN;
        shotBaseHoodDeg = Double.NaN;
        shotHoodLimited = false;
    }

    private void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) ||
                Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        // Robot gövdesi (daire)
        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        // Heading çizgisi (robotun yönü)
        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2;
        double y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent();
        double y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }
    private void drawPoseHistory() {
        panelsField.setStyle(historyStyle);

        double[] xPositions = follower.getPoseHistory().getXPositionsArray();
        double[] yPositions = follower.getPoseHistory().getYPositionsArray();

        for (int i = 0; i < xPositions.length - 1; i++) {
            panelsField.moveCursor(xPositions[i], yPositions[i]);
            panelsField.line(xPositions[i + 1], yPositions[i + 1]);
        }
    }

    @Override
    public void stop() {
        Superstructure.turretAimDeadbandDeg = 0.0;
        PoseStorage.savePose(follower.getPose());
        telemetryM.debug("TeleOp stopped! Final pose saved.");
    }
}
