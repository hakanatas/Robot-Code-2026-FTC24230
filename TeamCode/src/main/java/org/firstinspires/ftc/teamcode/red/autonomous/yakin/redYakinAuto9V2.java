package org.firstinspires.ftc.teamcode.red.autonomous.yakin;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Revolver.Revolver;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.Subsystems.Tilter.Tilter;
import org.firstinspires.ftc.teamcode.lib.Debouncer;
import org.firstinspires.ftc.teamcode.lib.math.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@Autonomous(name = "7-red_Yakin_9_V2")
public class redYakinAuto9V2 extends OpMode {
    private static final double ROBOT_RADIUS = 9;
    private static final double FLYWHEEL_IDLE_RPM = 3000.0;
    private static final double SHOT_TURRET_TOLERANCE_DEG = 3.0;
    private static final double SHOT_SETTLE_SECONDS = 0.08;
    private static final double PARK_SETTLE_SECONDS = 0.03;
    private static final double VISION_GATE_TIMEOUT_SECONDS = 0.16;
    private static final double REVOLVER_EMPTY_DEBOUNCE_SECONDS = 0.15;
    private static final boolean ENABLE_SHOT_LOG = false;
    private static final String SHOT_LOG_FILE_NAME = "redYakinAuto9V2-shot-log.csv";
    TelemetryManager telemetryM;
    private Follower follower;
    private int pathState;
    private FieldManager panelsField;
    private Style robotStyle;
    private Style pathStyle;
    private Timer pathTimer, opmodeTimer, shotBurstTimer;
    private Debouncer revolverEmptyDebouncer;
    private BufferedWriter shotLog;
    private File shotLogFile;
    private int shotLogLineCount = 0;
    private String shotLogStatus = "not started";
    private boolean lastRevolverEmptyRaw = false;
    private boolean lastRevolverEmpty = false;
    private boolean lastRevolverFull = false;
    private boolean shotBurstActive = false;
    private double lockedFlywheelRPM = FLYWHEEL_IDLE_RPM;
    private double lockedHoodAngle = 0.0;
    private double lockedTurretAngle = 0.0;

    // PATH POZISYONLARI
    private final Pose startPose = new Pose(117.700, 129.350, Math.toRadians(44));
    private final Pose score1Pose = new Pose(84.000, 84.300, Math.toRadians(0));
    private final Pose PPGPoseBehind = new Pose(102.000, 84.300, Math.toRadians(0));
    private final Pose PPGPose = new Pose(125.000, 84.300, Math.toRadians(0));
    private final Pose score2Pose = new Pose(94.000, 84.300, Math.toRadians(0));
    private final Pose PGPPoseBehind = new Pose(103.000, 61.000, Math.toRadians(0));
    private final Pose PGPPose = new Pose(131.000, 61.000, Math.toRadians(0));
    private final Pose score3Pose = new Pose(83.000, 71.000, Math.toRadians(0));
    private final Pose parkPose = new Pose(82.500, 63.000, Math.toRadians(0));

    // PATHLER
    private Path score1;
    private PathChain PPGBehind, PPG, score2, PGPBehind, PGP, score3, park;

    public void buildPaths() {
        score1 = new Path(new BezierLine(startPose, score1Pose));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());

        PPGBehind = follower.pathBuilder()
                .addPath(new BezierLine(score1Pose, PPGPoseBehind))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), PPGPoseBehind.getHeading())
                .build();

        PPG = follower.pathBuilder()
                .addPath(new BezierLine(PPGPoseBehind, PPGPose))
                .setLinearHeadingInterpolation(PPGPoseBehind.getHeading(), PPGPose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(PPGPose, score2Pose))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), score2Pose.getHeading())
                .build();

        PGPBehind = follower.pathBuilder()
                .addPath(new BezierLine(score2Pose, PGPPoseBehind))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), PGPPoseBehind.getHeading())
                .build();

        PGP = follower.pathBuilder()
                .addPath(new BezierLine(PGPPoseBehind, PGPPose))
                .setLinearHeadingInterpolation(PGPPoseBehind.getHeading(), PGPPose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose, score3Pose))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), score3Pose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(score3Pose, parkPose))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        boolean revolverEmptyRaw = !Superstructure.slot0.IsthereBall() && !Superstructure.slot2.IsthereBall() && !Superstructure.slot1.IsthereBall();
        boolean revolverEmpty = revolverEmptyDebouncer.calculate(revolverEmptyRaw);
        boolean revolverFull = Superstructure.slot0.IsthereBall() && Superstructure.slot2.IsthereBall() && Superstructure.slot1.IsthereBall();
        lastRevolverEmptyRaw = revolverEmptyRaw;
        lastRevolverEmpty = revolverEmpty;
        lastRevolverFull = revolverFull;

        switch (pathState) {
            case 0:
                // Başlangıç - score1'e git
                follower.followPath(score1);
                setPathState(1);
                break;

            case 1:
                // score1'e ulaşma bekleniyor
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                // İlk shoot (score1)
                runShotBurst();

                if (revolverEmpty) {
                    finishShotBurst();
                    follower.followPath(PPGBehind);
                    setPathState(3);
                }
                break;

            case 3:
                Superstructure.setIntakeSystem(1); // ← Intake aç

                if (!follower.isBusy()) {
                    follower.followPath(PPG);
                    setPathState(4);
                }
                break;

            case 4:
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy() || revolverFull) {
                    follower.followPath(score2);
                    setPathState(5);
                }
                break;

            case 5:
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    Superstructure.setIntakeSystem(0);
                    setPathState(6);
                }
                break;

            case 6:
                // Robot stabilize olsun
                if (pathTimer.getElapsedTimeSeconds() > SHOT_SETTLE_SECONDS) {
                    setPathState(7);
                }
                break;

            case 7:
                // İkinci shoot (score2)
                runShotBurst();

                if (revolverEmpty) {
                    finishShotBurst();
                    follower.followPath(PGPBehind);
                    setPathState(8);
                }
                break;

            case 8:
                Superstructure.setIntakeSystem(1); // ← Intake aç

                if (!follower.isBusy()) {
                    follower.followPath(PGP);
                    setPathState(9);
                }
                break;

            case 9:
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy() || revolverFull) {
                    follower.followPath(score3);
                    setPathState(10);
                }
                break;

            case 10:
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    Superstructure.setIntakeSystem(0);
                    setPathState(11);
                }
                break;

            case 11:
                // Robot stabilize olsun
                if (pathTimer.getElapsedTimeSeconds() > SHOT_SETTLE_SECONDS) {
                    setPathState(12);
                }
                break;

            case 12:
                // Üçüncü shoot (score3)
                runShotBurst();

                if (revolverEmpty) {
                    // Üçüncü shoot bitti, sistemleri kapat
                    finishShotBurst();
                    Superstructure.setFlywheelIdleMode(false);
                    Superstructure.flywheel.setSetpointRPM(0);
                    Superstructure.manualHoodControl = true;
                    Superstructure.manualTurretControl = true;
                    Superstructure.turret.setTurretAngle(0.0);
                    Superstructure.hood.setHoodAngle(0.0);
                    setPathState(13);
                }
                break;

            case 13:
                // Hood'un kapanmasını bekle
                Superstructure.feeder.setFeedersMotor(0);

                if (pathTimer.getElapsedTimeSeconds() > PARK_SETTLE_SECONDS) {
                    // Park'a HIZLI git
                    follower.followPath(park);
                    setPathState(14);
                }
                break;

            case 14:
                // Park pozisyonuna gidiliyor


                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }

        // FLYWHEEL RÖLANTI SİSTEMİ
        if (pathState != 2 && pathState != 7 && pathState != 12 && pathState != 13 && pathState != 14 && pathState != -1) {
            keepShooterWarm();
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shotBurstTimer = new Timer();
        opmodeTimer.resetTimer();

        // Debouncer oluştur - 0.5 saniye, Rising edge
        revolverEmptyDebouncer = new Debouncer(REVOLVER_EMPTY_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);

        follower = Constants.createFollower(hardwareMap);
        Superstructure.isauto=true;
        Superstructure.init(hardwareMap);
        Superstructure.reset();
        Superstructure.flywheelIdleRPM = FLYWHEEL_IDLE_RPM;
        Superstructure.vision.LL3.pipelineSwitch(1);
        Superstructure.tilter.setState(Tilter.tilterState.IDLE);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        initShotLog();

        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        robotStyle = new Style("", "#4CAF50", 0.75);
        pathStyle = new Style("", "#3F51B5", 0.75);

        follower.setStartingPose(startPose);

        buildPaths();

        PoseStorage.savePose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        revolverEmptyDebouncer.reset(); // ← Debouncer'ı resetle!
        resetShotBurst();
        setPathState(0);
        logShotState("START");
        telemetryM.debug("Autonomous started! 3-Shoot sequence");
    }

    @Override
    public void loop() {
        follower.update();

        Superstructure.read();
        Superstructure.periodic();
        Superstructure.write();
        Superstructure.isRevolverReady();

        autonomousPathUpdate();
        logShotState("LOOP");

        drawOnDashboard();

        PoseStorage.savePose(follower.getPose());

        telemetryM.debug("State", pathState);
        telemetryM.debug("Shot Log", shotLogStatus);
        telemetryM.debug("Saved for TeleOp", PoseStorage.getSavedPose());
        telemetryM.update();
    }
    private void drawOnDashboard() {
        try {
            if (follower.getCurrentPath() != null) {
                drawPath(follower.getCurrentPath());

                Pose closestPoint = follower.getPointFromPath(
                        follower.getCurrentPath().getClosestPointTValue()
                );
                drawRobot(new Pose(
                        closestPoint.getX(),
                        closestPoint.getY(),
                        follower.getCurrentPath().getHeadingGoal(
                                follower.getCurrentPath().getClosestPointTValue()
                        )
                ), pathStyle);
            }

            drawPoseHistory();

            drawRobot(follower.getPose(), robotStyle);

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

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

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
    private void drawPath(Path path) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(pathStyle);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }
    private void drawPoseHistory() {
        Style historyStyle = new Style("", "#FF9800", 0.5);
        panelsField.setStyle(historyStyle);

        double[] xPositions = follower.getPoseHistory().getXPositionsArray();
        double[] yPositions = follower.getPoseHistory().getYPositionsArray();

        for (int i = 0; i < xPositions.length - 1; i++) {
            panelsField.moveCursor(xPositions[i], yPositions[i]);
            panelsField.line(xPositions[i + 1], yPositions[i + 1]);
        }
    }

    private void runShotBurst() {
        if (!shotBurstActive) {
            Superstructure.setShootSystem();
            if (isLiveShotGateReady()) {
                beginShotBurst();
            }
            return;
        }

        holdLockedShotTargets();
        Superstructure.feeder.setFeedersMotor(isLockedShotGateReady() ? 1.0 : 0.0);
    }

    private void beginShotBurst() {
        shotBurstActive = true;
        shotBurstTimer.resetTimer();

        lockedFlywheelRPM = Superstructure.flywheel.getTargetRPM();
        lockedHoodAngle = Superstructure.hood.getTargetAngle();
        lockedTurretAngle = Superstructure.turret.TurretController.getTargetPosition();

        Superstructure.manualHoodControl = true;
        Superstructure.manualTurretControl = true;
        holdLockedShotTargets();
        Superstructure.feeder.setFeedersMotor(1.0);
        logShotState("SHOT_BURST_START");
    }

    private void finishShotBurst() {
        Superstructure.stopShooting();
        resetShotBurst();
        logShotState("SHOT_BURST_FINISH");
    }

    private void resetShotBurst() {
        shotBurstActive = false;
        Superstructure.manualHoodControl = false;
        Superstructure.manualTurretControl = false;
    }

    private void holdLockedShotTargets() {
        Superstructure.flywheel.setSetpointRPM(lockedFlywheelRPM);
        Superstructure.hood.setHoodAngle(lockedHoodAngle);
        Superstructure.turret.setTurretAngle(lockedTurretAngle);
    }

    private void keepShooterWarm() {
        double warmupRPM = FLYWHEEL_IDLE_RPM;
        if (Superstructure.vision.tv) {
            warmupRPM = org.firstinspires.ftc.teamcode.lib.Constants.ShootingParams.kRPMMap
                    .getInterpolated(new InterpolatingDouble(Superstructure.vision.ty)).value;
        }

        Superstructure.flywheelIdleRPM = warmupRPM;
        Superstructure.flywheel.setSetpointRPM(warmupRPM);
        Superstructure.feeder.setFeedersMotor(0);
    }

    private boolean isLiveShotGateReady() {
        return Superstructure.flywheel.IsAtSetpoint()
                && Superstructure.hood.IsAtSetpoint()
                && isVisionReadyForShot()
                && isTurretReady();
    }

    private boolean isLockedShotGateReady() {
        return Superstructure.flywheel.IsAtSetpoint()
                && Superstructure.hood.IsAtSetpoint()
                && isTurretReady();
    }

    private boolean isTurretReady() {
        double turretError = Superstructure.turret.TurretController.getTargetPosition()
                - Superstructure.turret.getTurretAngle();
        return Math.abs(turretError) <= SHOT_TURRET_TOLERANCE_DEG;
    }

    private boolean isVisionReadyForShot() {
        return Superstructure.vision.tv
                || (pathTimer != null && pathTimer.getElapsedTimeSeconds() > VISION_GATE_TIMEOUT_SECONDS);
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        telemetryM.debug("Path state changed to: " + pState);
        logShotState("STATE_" + pState);
    }

    @Override
    public void stop() {
        logShotState("STOP");
        closeShotLog();
        resetShotBurst();
        telemetryM.debug("Autonomous stopped!");
        Superstructure.setFlywheelIdleMode(false);
        Superstructure.manualHoodControl = true;
        Superstructure.setIntakeSystem(0);
        Superstructure.flywheel.setSetpointRPM(0);
        Superstructure.feeder.setFeedersMotor(0);
        Superstructure.hood.setHoodAngle(0);
        PoseStorage.savePose(follower.getPose());
    }

    private void initShotLog() {
        if (!ENABLE_SHOT_LOG) {
            shotLogStatus = "disabled";
            return;
        }

        try {
            File logDir = hardwareMap.appContext.getFilesDir();
            if (!logDir.exists() && !logDir.mkdirs()) {
                shotLogStatus = "log dir failed: " + logDir.getAbsolutePath();
                return;
            }

            shotLogFile = new File(logDir, SHOT_LOG_FILE_NAME);
            shotLog = new BufferedWriter(new FileWriter(shotLogFile, false));
            shotLog.write("time_s,state,event,pathTime_s,followerBusy,visionTv,tagId,tx,ty,flywheelRpm,flywheelTargetRpm,flywheelReady,hoodAngle,hoodTarget,hoodReady,turretAngle,turretTarget,turretError,turretReady,shotGate,shotBurstActive,lockedShotGate,lockedFlywheelRpm,lockedHoodTarget,lockedTurretTarget,feederPower,intakePower,slot0,slot1,slot2,currentSlotHasBall,revolverEmptyRaw,revolverEmptyDebounced,revolverFull,revolverAngle,revolverTarget,sensor0,sensor1,sensor2,intakeSensor,poseX,poseY,poseHeadingDeg\n");
            shotLog.flush();
            shotLogStatus = shotLogFile.getAbsolutePath();
        } catch (IOException e) {
            shotLogStatus = "log init error: " + e.getMessage();
            closeShotLog();
        }
    }

    private void logShotState(String event) {
        if (shotLog == null) {
            return;
        }

        try {
            Pose pose = follower == null ? null : follower.getPose();
            double poseX = pose == null ? Double.NaN : pose.getX();
            double poseY = pose == null ? Double.NaN : pose.getY();
            double poseHeadingDeg = pose == null ? Double.NaN : Math.toDegrees(pose.getHeading());

            double pathTime = pathTimer == null ? Double.NaN : pathTimer.getElapsedTimeSeconds();
            double time = opmodeTimer == null ? Double.NaN : opmodeTimer.getElapsedTimeSeconds();
            boolean followerBusy = follower != null && follower.isBusy();

            boolean flywheelReady = Superstructure.flywheel.IsAtSetpoint();
            boolean hoodReady = Superstructure.hood.IsAtSetpoint();
            double turretAngle = Superstructure.turret.getTurretAngle();
            double turretTarget = Superstructure.turret.TurretController.getTargetPosition();
            double turretError = turretTarget - turretAngle;
            boolean turretReady = Math.abs(turretError) <= 3.0;
            boolean shotGate = flywheelReady && hoodReady && isVisionReadyForShot() && turretReady;
            boolean lockedShotGate = shotBurstActive && isLockedShotGateReady();

            shotLog.write(String.format(Locale.US,
                    "%.3f,%d,%s,%.3f,%b,%b,%d,%.3f,%.3f,%.1f,%.1f,%b,%.2f,%.2f,%b,%.2f,%.2f,%.2f,%b,%b,%b,%b,%.1f,%.2f,%.2f,%.2f,%.2f,%b,%b,%b,%b,%b,%b,%b,%.2f,%.2f,%b,%b,%b,%b,%.2f,%.2f,%.2f%n",
                    time,
                    pathState,
                    event,
                    pathTime,
                    followerBusy,
                    Superstructure.vision.tv,
                    Superstructure.vision.ApriltagID,
                    Superstructure.vision.tx,
                    Superstructure.vision.ty,
                    Superstructure.flywheel.getShooterRPM(),
                    Superstructure.flywheel.getTargetRPM(),
                    flywheelReady,
                    Superstructure.hood.getHoodAngle(),
                    Superstructure.hood.getTargetAngle(),
                    hoodReady,
                    turretAngle,
                    turretTarget,
                    turretError,
                    turretReady,
                    shotGate,
                    shotBurstActive,
                    lockedShotGate,
                    lockedFlywheelRPM,
                    lockedHoodAngle,
                    lockedTurretAngle,
                    Superstructure.feeder.getPower(),
                    Superstructure.intake.getPower(),
                    Superstructure.slot0.IsthereBall(),
                    Superstructure.slot1.IsthereBall(),
                    Superstructure.slot2.IsthereBall(),
                    Superstructure.currentSlot.IsthereBall(),
                    lastRevolverEmptyRaw,
                    lastRevolverEmpty,
                    lastRevolverFull,
                    Revolver.getRevolverAngle().getDegrees(),
                    Revolver.RevolverController.getTargetPosition(),
                    Revolver.get0Status(),
                    Revolver.get1Status(),
                    Revolver.get2Status(),
                    Revolver.getIntakeStatus(),
                    poseX,
                    poseY,
                    poseHeadingDeg
            ));

            shotLogLineCount++;
            if (shotLogLineCount % 10 == 0) {
                shotLog.flush();
            }
        } catch (IOException e) {
            shotLogStatus = "log write error: " + e.getMessage();
            closeShotLog();
        }
    }

    private void closeShotLog() {
        if (shotLog == null) {
            return;
        }

        try {
            shotLog.flush();
            shotLog.close();
        } catch (IOException ignored) {
        } finally {
            shotLog = null;
        }
    }
}
