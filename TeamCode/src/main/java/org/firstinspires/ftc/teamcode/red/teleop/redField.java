package org.firstinspires.ftc.teamcode.red.teleop;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Revolver.Revolver;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.Subsystems.Tilter.Tilter;
import org.firstinspires.ftc.teamcode.lib.joysticklib.TimeDelayedBoolean;
import org.firstinspires.ftc.teamcode.lib.joysticklib.Toggle;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "redSaha")
public class redField extends OpMode {
    private static final double ROBOT_RADIUS = 9;
    private static final double FAST_TRANSLATION_SPEED = 1.0;
    private static final double SLOW_TRANSLATION_SPEED = 0.65;
    private static final double FAST_TURN_SPEED = 0.60;
    private static final double SLOW_TURN_SPEED = 0.35;
    private Follower follower;
    private TelemetryManager telemetryM;
    private FieldManager panelsField;
    private Style robotStyle;
    private Style historyStyle;
    private Pose savedPose;
    private boolean poseLoaded = false;
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
        double translationSpeed = gamepad1.left_bumper ? SLOW_TRANSLATION_SPEED : FAST_TRANSLATION_SPEED;
        double turnSpeed = gamepad1.left_bumper ? SLOW_TURN_SPEED : FAST_TURN_SPEED;
        double forwardCommand = translationSpeed * -gamepad1.left_stick_y;
        double strafeCommand = translationSpeed * -gamepad1.left_stick_x;
        double turnCommand = turnSpeed * -gamepad1.right_stick_x;

        follower.setTeleOpDrive(
                forwardCommand,
                strafeCommand,
                turnCommand,
                false
        );

        follower.update();

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
            Superstructure.setShootSystem();
        } else {
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

        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        Superstructure.setVoltage(batteryVoltage);

        if (gamepad1.options) {
            follower.setPose(new Pose(0, 0, 0));
            PoseStorage.reset();
        }

        Superstructure.read();
        Superstructure.periodic();
        Superstructure.write();
        Superstructure.isRevolverReady();

        drawOnDashboard();

        // Telemetry
        telemetryM.debug("s0", Revolver.get0Status());
        telemetryM.debug("s1", Revolver.get1Status());
        telemetryM.debug("s2", Revolver.get2Status());
        telemetryM.debug("rv", Revolver.getRevolverAngle().getDegrees());
        telemetryM.debug("TY", Superstructure.vision.ty);
        telemetryM.debug("driveMode", gamepad1.left_bumper ? "SLOW" : "FAST");
        telemetryM.update();
        updateDriverStationTelemetry(translationSpeed, turnSpeed, forwardCommand, strafeCommand, turnCommand, batteryVoltage);
    }

    private void updateDriverStationTelemetry(double translationSpeed, double turnSpeed,
                                              double forwardCommand, double strafeCommand,
                                              double turnCommand, double batteryVoltage) {
        Pose pose = follower.getPose();

        telemetry.addData("opMode", "redSaha");
        telemetry.addData("driveMode", gamepad1.left_bumper ? "SLOW" : "FAST");
        telemetry.addData("driveFrame", "FIELD");
        telemetry.addData("translationSpeed", "%.2f", translationSpeed);
        telemetry.addData("turnSpeed", "%.2f", turnSpeed);
        telemetry.addData("leftBumper", gamepad1.left_bumper);
        telemetry.addData("left_y raw", "%.2f", gamepad1.left_stick_y);
        telemetry.addData("left_x raw", "%.2f", gamepad1.left_stick_x);
        telemetry.addData("right_x raw", "%.2f", gamepad1.right_stick_x);
        telemetry.addData("forward cmd", "%.2f", forwardCommand);
        telemetry.addData("strafe cmd", "%.2f", strafeCommand);
        telemetry.addData("turn cmd", "%.2f", turnCommand);
        telemetry.addData("battery", "%.2f V", batteryVoltage);
        if (pose != null) {
            telemetry.addData("pose", "x %.1f y %.1f h %.1f",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        }
        addThroughputTelemetry();
        telemetry.update();
    }

    private void addThroughputTelemetry() {
        double turretAngle = Superstructure.turret.getTurretAngle();
        double turretTarget = Superstructure.turret.TurretController.getTargetPosition();
        boolean turretReady = Math.abs(turretAngle - turretTarget) <= 3.0;
        boolean flywheelReady = Superstructure.flywheel.IsAtSetpoint();
        boolean hoodReady = Superstructure.hood.IsAtSetpoint();
        boolean visionReady = Superstructure.vision.tv;
        boolean revolverEmpty = !Superstructure.slot0.IsthereBall()
                && !Superstructure.slot1.IsthereBall()
                && !Superstructure.slot2.IsthereBall();

        telemetry.addData("shoot req", gamepad1.right_bumper);
        telemetry.addData("flywheel", "%.0f / %.0f rpm",
                Superstructure.flywheel.getShooterRPM(),
                Superstructure.flywheel.getTargetRPM());
        telemetry.addData("ready F/H/T/V", "%s/%s/%s/%s",
                flywheelReady, hoodReady, turretReady, visionReady);
        telemetry.addData("turret", "%.1f -> %.1f", turretAngle, turretTarget);
        telemetry.addData("hood", "%.1f", Superstructure.hood.getHoodAngle());
        telemetry.addData("feeder gate", flywheelReady && hoodReady && turretReady && visionReady);
        telemetry.addData("intake power", "%.1f", Superstructure.intake.getPower());
        telemetry.addData("intake sensor", Revolver.getIntakeStatus());
        telemetry.addData("slots 0/1/2", "%s/%s/%s",
                Superstructure.slot0.IsthereBall(),
                Superstructure.slot1.IsthereBall(),
                Superstructure.slot2.IsthereBall());
        telemetry.addData("revolver empty", revolverEmpty);
        telemetry.addData("revolver", "%.1f -> %.1f",
                Revolver.getRevolverAngle().getDegrees(),
                Revolver.RevolverController.getTargetPosition());
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
        PoseStorage.savePose(follower.getPose());
        telemetryM.debug("TeleOp stopped! Final pose saved.");
    }
}
