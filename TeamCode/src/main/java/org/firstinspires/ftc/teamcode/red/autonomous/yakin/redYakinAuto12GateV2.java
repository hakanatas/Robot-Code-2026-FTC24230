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
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.Subsystems.Tilter.Tilter;
import org.firstinspires.ftc.teamcode.lib.Debouncer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "1-red_Yakin_Kapi_12_V2")
public class redYakinAuto12GateV2 extends OpMode {
    private static final double ROBOT_RADIUS = 9;
    private static final double FLYWHEEL_IDLE_RPM = 3000.0;
    private static final double FIRST_SHOT_TAG_SEARCH_TURRET_ANGLE_DEG = 45.0;
    private static final double SHOT_TURRET_OPEN_OFFSET_DEG = 1.5;
    private static final double SECOND_PICKUP_SHOT_TURRET_OFFSET_DEG = SHOT_TURRET_OPEN_OFFSET_DEG - 3.0;
    private static final double FINAL_SHOT_TURRET_OFFSET_DEG = SHOT_TURRET_OPEN_OFFSET_DEG - 3.0;
    private static final double FINAL_SHOT_MIN_ATTEMPT_SECONDS = 1.0;
    private static final double GATE_RETURN_T_VALUE = 0.88;
    private static final double CM_TO_INCHES = 1.0 / 2.54;
    private static final double OTHER_PICKUP_RIGHT_OFFSET_INCHES = 4.0 * CM_TO_INCHES;
    private static final double SCORE_POSE_LEFT_OFFSET_INCHES = 2.5 * CM_TO_INCHES;
    private static final double SCORE_POSE_UP_OFFSET_INCHES = 2.5 * CM_TO_INCHES;
    private static final PathConstraints V2_DRIVE_PATH_CONSTRAINTS = new PathConstraints(
            1.000,
            0.20,
            0.25,
            0.015,
            0,
            1.0,
            10,
            1.0
    );
    private static final PathConstraints V2_GATE_PATH_CONSTRAINTS = new PathConstraints(
            0.990,
            0.20,
            0.30,
            0.015,
            0,
            1.0,
            10,
            1.0
    );
    TelemetryManager telemetryM;
    private Follower follower;
    private int pathState;
    private FieldManager panelsField;
    private Style robotStyle;
    private Style pathStyle;
    private Timer pathTimer, opmodeTimer;
    private Debouncer revolverEmptyDebouncer;
    private boolean firstShotTagSearchActive;

    // PATH POZISYONLARI
    private final Pose startPose = new Pose(117.700, 129.350, Math.toRadians(44));
    private final Pose score1Pose = new Pose(96.000 - SCORE_POSE_LEFT_OFFSET_INCHES, 84.300 + SCORE_POSE_UP_OFFSET_INCHES, Math.toRadians(0));
    private final Pose PPGPose = new Pose(125.000, 84.300, Math.toRadians(0));
    private final Pose backPose = new Pose(117.000, 84.300, Math.toRadians(0));
    private final Pose gatePose = new Pose(126.000, 75.000, Math.toRadians(0));
    private final Pose score2Pose = new Pose(97.000 - SCORE_POSE_LEFT_OFFSET_INCHES, 84.300 + SCORE_POSE_UP_OFFSET_INCHES, Math.toRadians(0));
    private final Pose PGPPoseBehind = new Pose(103.000, 61.000 - OTHER_PICKUP_RIGHT_OFFSET_INCHES, Math.toRadians(0));
    private final Pose PGPPose = new Pose(131.000, 61.000 - OTHER_PICKUP_RIGHT_OFFSET_INCHES, Math.toRadians(0));
    private final Pose score3Pose = new Pose(83.000 - SCORE_POSE_LEFT_OFFSET_INCHES, 71.000 + SCORE_POSE_UP_OFFSET_INCHES, Math.toRadians(0));
    private final Pose GPPPoseBehind = new Pose(102.000, 38.000 - OTHER_PICKUP_RIGHT_OFFSET_INCHES, Math.toRadians(0));
    private final Pose GPPPose = new Pose(131.000, 38.000 - OTHER_PICKUP_RIGHT_OFFSET_INCHES, Math.toRadians(0));
    private final Pose score4Pose = new Pose(83.000 - SCORE_POSE_LEFT_OFFSET_INCHES, 71.000 + SCORE_POSE_UP_OFFSET_INCHES, Math.toRadians(0));
    private final Pose parkPose = new Pose(83.000, 65.000, Math.toRadians(0));

    // PATHLER
    private Path score1;
    private PathChain PPG, back, gate, score2, PGPBehind, PGP, score3, GPPBehind, GPP, score4, park;

    public void buildPaths() {
        score1 = new Path(new BezierLine(startPose, score1Pose));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());

        PPG = follower.pathBuilder()
                .addPath(new BezierLine(score1Pose, PPGPose))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), PPGPose.getHeading())
                .build();

        back = follower.pathBuilder()
                .addPath(new BezierLine(PPGPose, backPose))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), backPose.getHeading())
                .setConstraintsForAll(V2_DRIVE_PATH_CONSTRAINTS)
                .build();

        gate = follower.pathBuilder()
                .addPath(new BezierLine(backPose, gatePose))
                .setLinearHeadingInterpolation(backPose.getHeading(), gatePose.getHeading())
                .setConstraintsForAll(V2_GATE_PATH_CONSTRAINTS)
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, score2Pose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), score2Pose.getHeading())
                .setConstraintsForAll(V2_DRIVE_PATH_CONSTRAINTS)
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
                .setConstraintsForAll(V2_DRIVE_PATH_CONSTRAINTS)
                .build();

        GPPBehind = follower.pathBuilder()
                .addPath(new BezierLine(score3Pose, GPPPoseBehind))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), GPPPoseBehind.getHeading())
                .build();

        GPP = follower.pathBuilder()
                .addPath(new BezierLine(GPPPoseBehind, GPPPose))
                .setLinearHeadingInterpolation(GPPPoseBehind.getHeading(), GPPPose.getHeading())
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose, score4Pose))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), score4Pose.getHeading())
                .setConstraintsForAll(V2_DRIVE_PATH_CONSTRAINTS)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(score4Pose, parkPose))
                .setLinearHeadingInterpolation(score4Pose.getHeading(), parkPose.getHeading())
                .setConstraintsForAll(V2_DRIVE_PATH_CONSTRAINTS)
                .build();
    }

    public void autonomousPathUpdate() {
        boolean revolverEmptyRaw = !Superstructure.slot0.IsthereBall() && !Superstructure.slot2.IsthereBall() && !Superstructure.slot1.IsthereBall();
        boolean revolverEmpty = revolverEmptyDebouncer.calculate(revolverEmptyRaw);
        boolean revolverFull = Superstructure.slot0.IsthereBall() && Superstructure.slot2.IsthereBall() && Superstructure.slot1.IsthereBall();

        // 29.7 saniye timeout kontrolü
        if (opmodeTimer.getElapsedTimeSeconds() >= 29.2 && pathState != -1) {
            // Acil park modu
            Superstructure.stopShooting();
            Superstructure.setFlywheelIdleMode(false);
            Superstructure.flywheel.setSetpointRPM(0);
            Superstructure.manualHoodControl = true;
            Superstructure.manualTurretControl = true;
            Superstructure.turret.setTurretAngle(0.0);
            Superstructure.hood.setHoodAngle(0.0);
            Superstructure.setIntakeSystem(0);
            follower.followPath(park);
            setPathState(-1);
            return;
        }

        switch (pathState) {
            case 0:
                // Başlangıç - score1'e git
                holdFirstShotTagSearchAngle();
                follower.followPath(score1);
                setPathState(1);
                break;

            case 1:
                // score1'e ulaşma bekleniyor
                holdFirstShotTagSearchAngle();
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                // İlk shoot (score1)
                firstShotTagSearchActive = false;
                Superstructure.setShootSystem();

                if (revolverEmpty) {
                    Superstructure.stopShooting();
                    // PPG'ye git + Intake AÇ
                    Superstructure.setIntakeSystem(1);
                    follower.followPath(PPG);
                    setPathState(3);
                }
                break;

            case 3:
                // PPG'ye gidiliyor - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy() || revolverFull) {
                    // PPG bitti, back'e git
                    follower.followPath(back);
                    setPathState(4);
                }
                break;

            case 4:
                // BACK'e gidiliyor - intake AÇIK (YENİ!)
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    // Back bitti, gate'e git
                    follower.followPath(gate, 1.0, false);
                    setPathState(5);
                }
                break;

            case 5:
                // GATE'e gidiliyor - intake AÇIK (YENİ!)
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy() || follower.getCurrentTValue() > GATE_RETURN_T_VALUE) {
                    // Gate bitti, score2'ye git
                    follower.followPath(score2);
                    setPathState(6);
                }
                break;

            case 6:
                // Score2'ye giderken - intake KAPALI
                Superstructure.setIntakeSystem(0);

                if (!follower.isBusy()) {
                    // Score2'ye ulaştık, intake'i kapat
                    Superstructure.setIntakeSystem(0);
                    setPathState(7);
                }
                break;

            case 7:
                // Robot stabilize olsun
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(8);
                }
                break;

            case 8:
                // İkinci shoot (score2)
                Superstructure.setShootSystem();

                if (revolverEmpty) {
                    Superstructure.stopShooting();
                    // PGPBehind'a git + Intake AÇ
                    Superstructure.setIntakeSystem(1);
                    follower.followPath(PGPBehind);
                    setPathState(9);
                }
                break;

            case 9:
                // PGPBehind'a gidiliyor - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    // PGPBehind'a ulaştık, PGP'ye git
                    follower.followPath(PGP);
                    setPathState(10);
                }
                break;

            case 10:
                // PGP'ye giderken - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy() || revolverFull) {
                    // PGP bitti, score3'e git
                    follower.followPath(score3);
                    setPathState(11);
                }
                break;

            case 11:
                // Score3'e giderken - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    // Score3'e ulaştık, intake'i kapat
                    Superstructure.setIntakeSystem(0);
                    setPathState(12);
                }
                break;

            case 12:
                // Robot stabilize olsun
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(13);
                }
                break;

            case 13:
                // Üçüncü shoot (score3)
                Superstructure.setShootSystem();

                if (revolverEmpty) {
                    Superstructure.stopShooting();
                    // GPPBehind'a git + Intake AÇ
                    Superstructure.setIntakeSystem(1);
                    follower.followPath(GPPBehind);
                    setPathState(14);
                }
                break;

            case 14:
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    follower.followPath(GPP);
                    setPathState(15);
                }
                break;

            case 15:
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy() || revolverFull) {
                    follower.followPath(score4);
                    setPathState(16);
                }
                break;

            case 16:
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    Superstructure.setIntakeSystem(0);
                    setPathState(17);
                }
                break;

            case 17:
                // Robot stabilize olsun
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(18);
                }
                break;

            case 18:
                // Dördüncü shoot (score4) - SON SHOOT!
                Superstructure.setShootSystem();

                if (revolverEmpty && pathTimer.getElapsedTimeSeconds() > FINAL_SHOT_MIN_ATTEMPT_SECONDS) {
                    // Dördüncü shoot bitti, sistemleri kapat
                    Superstructure.stopShooting();
                    Superstructure.setFlywheelIdleMode(false);
                    Superstructure.flywheel.setSetpointRPM(0);
                    Superstructure.manualHoodControl = true;
                    Superstructure.manualTurretControl = true;
                    Superstructure.turret.setTurretAngle(0.0);
                    Superstructure.hood.setHoodAngle(0.0);
                    setPathState(19);
                }
                break;

            case 19:
                // Hood'un kapanmasını bekle
                Superstructure.feeder.setFeedersMotor(0);

                if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                    follower.followPath(park);
                    setPathState(20);
                }
                break;

            case 20:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }

        // FLYWHEEL RÖLANTI SİSTEMİ
        if (pathState != 2 && pathState != 8 && pathState != 13 && pathState != 18 && pathState != 19 && pathState != 20 && pathState != -1) {
            Superstructure.flywheel.setSetpointRPM(FLYWHEEL_IDLE_RPM);
            Superstructure.feeder.setFeedersMotor(0);
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Debouncer oluştur - 0.5 saniye, Rising edge
        revolverEmptyDebouncer = new Debouncer(0.2, Debouncer.DebounceType.kRising);

        follower = Constants.createFollower(hardwareMap);
        Superstructure.isauto=true;
        Superstructure.init(hardwareMap);
        Superstructure.reset();
        Superstructure.vision.LL3.pipelineSwitch(1);
        Superstructure.tilter.setState(Tilter.tilterState.IDLE);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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
        revolverEmptyDebouncer.reset();
        firstShotTagSearchActive = true;
        holdFirstShotTagSearchAngle();
        setPathState(0);
        telemetryM.debug("Autonomous started! 4-Shoot sequence with BACK+GATE (12-BALL)");
    }

    @Override
    public void loop() {
        follower.update();

        Superstructure.read();
        Superstructure.periodic();
        applyV2TurretTargetAdjustments();
        Superstructure.write();
        Superstructure.isRevolverReady();

        autonomousPathUpdate();

        drawOnDashboard();

        PoseStorage.savePose(follower.getPose());

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

    private void holdFirstShotTagSearchAngle() {
        if (!firstShotTagSearchActive || Superstructure.vision.tv) {
            return;
        }

        Superstructure.manualTurretControl = false;
        Superstructure.turret.setTurretAngle(FIRST_SHOT_TAG_SEARCH_TURRET_ANGLE_DEG);
    }

    private void applyV2TurretTargetAdjustments() {
        boolean targetAdjusted = false;

        if (firstShotTagSearchActive && !Superstructure.vision.tv && (pathState == 0 || pathState == 1)) {
            Superstructure.manualTurretControl = false;
            Superstructure.turret.setTurretAngle(FIRST_SHOT_TAG_SEARCH_TURRET_ANGLE_DEG);
            targetAdjusted = true;
        } else if (isShotState() && Superstructure.vision.tv) {
            double target = Superstructure.turret.TurretController.getTargetPosition() + getShotTurretOffsetDeg();
            Superstructure.turret.setTurretAngle(target);
            targetAdjusted = true;
        }

        if (targetAdjusted) {
            Superstructure.turret.periodic();
        }
    }

    private boolean isShotState() {
        return isShotState(pathState);
    }

    private boolean isShotState(int state) {
        return state == 2 || state == 8 || state == 13 || state == 18;
    }

    private double getShotTurretOffsetDeg() {
        if (pathState == 13) {
            return SECOND_PICKUP_SHOT_TURRET_OFFSET_DEG;
        }
        if (pathState == 18) {
            return FINAL_SHOT_TURRET_OFFSET_DEG;
        }

        return SHOT_TURRET_OPEN_OFFSET_DEG;
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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        if (revolverEmptyDebouncer != null && isShotState(pState)) {
            revolverEmptyDebouncer.reset();
        }
        telemetryM.debug("Path state changed to: " + pState);
    }

    @Override
    public void stop() {
        telemetryM.debug("Autonomous stopped!");
        Superstructure.setFlywheelIdleMode(false);
        Superstructure.manualHoodControl = true;
        Superstructure.setIntakeSystem(0);
        Superstructure.flywheel.setSetpointRPM(0);
        Superstructure.feeder.setFeedersMotor(0);
        Superstructure.hood.setHoodAngle(0);
        PoseStorage.savePose(follower.getPose());
    }
}
