package org.firstinspires.ftc.teamcode.blue.autonomous.uzak;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
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
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.Subsystems.Tilter.Tilter;
import org.firstinspires.ftc.teamcode.lib.Debouncer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "6-blue_Uzak_6")
public class blueUzakAuto6 extends OpMode {
    private static final double ROBOT_RADIUS = 9;
    private static final double FLYWHEEL_IDLE_RPM = 3700.0;
    private static final double BLUE_UZAK_SHOT_RPM_MULTIPLIER = 0.95;
    private static final double BLUE_UZAK_HOOD_OFFSET_DEG = 2.0;
    private static final double APRILTAG_SEARCH_TURRET_ANGLE_DEG = 55.0;
    private static final double SHOT_EXIT_WAIT_SECONDS = 0.50;
    private static final double FORCE_PARK_TIME_SECONDS = 29.0;
    TelemetryManager telemetryM;
    private Follower follower;
    private int pathState;
    private FieldManager panelsField;
    private Style robotStyle;
    private Style pathStyle;
    private Timer pathTimer, opmodeTimer;
    private Debouncer revolverEmptyDebouncer;

    // PATH POZISYONLARI
    private final Pose startPose = new Pose(54.800, 8.600, Math.toRadians(180));
    private final Pose score1Pose = new Pose(61.500, 27.000, Math.toRadians(180));
    private final Pose GPPPoseBehind = new Pose(42.000, 35.000, Math.toRadians(180));
    private final Pose GPPPose = new Pose(12.000, 35.000, Math.toRadians(180));
    private final Pose score2Pose = new Pose(59.000, 27.000, Math.toRadians(180));
    private final Pose parkPose = new Pose(57.000, 40.000, Math.toRadians(180));
    // PATHLER
    private Path score1;
    private PathChain GPPBehind, GPP, score2, park;

    public void buildPaths() {
        score1 = new Path(new BezierLine(startPose, score1Pose));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());

        GPPBehind = follower.pathBuilder()
                .addPath(new BezierLine(score1Pose, GPPPoseBehind))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), GPPPoseBehind.getHeading())
                .build();

        GPP = follower.pathBuilder()
                .addPath(new BezierLine(GPPPoseBehind, GPPPose))
                .setLinearHeadingInterpolation(GPPPoseBehind.getHeading(), GPPPose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose, score2Pose))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), score2Pose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(score2Pose, parkPose))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        boolean revolverEmptyRaw = !Superstructure.slot0.IsthereBall() && !Superstructure.slot2.IsthereBall() && !Superstructure.slot1.IsthereBall();
        boolean revolverEmpty = revolverEmptyDebouncer.calculate(revolverEmptyRaw);
        boolean revolverFull = Superstructure.slot0.IsthereBall() && Superstructure.slot2.IsthereBall() && Superstructure.slot1.IsthereBall();

        if (opmodeTimer.getElapsedTimeSeconds() >= FORCE_PARK_TIME_SECONDS
                && pathState != 9
                && pathState != -1) {
            forcePark();
            return;
        }

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
                Superstructure.setShootSystem(BLUE_UZAK_SHOT_RPM_MULTIPLIER, BLUE_UZAK_HOOD_OFFSET_DEG);

                if (revolverEmpty) {
                    Superstructure.stopShooting();
                    follower.followPath(GPPBehind);
                    setPathState(3);
                }
                break;

            case 3:
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {

                    follower.followPath(GPP);
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
                    // Score2'ye ulaştık, intake'i kapat
                    Superstructure.setIntakeSystem(0);
                    setPathState(6);
                }
                break;

            case 6:
                // Robot stabilize olsun
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(7);
                }
                break;

            case 7:
                // İkinci shoot (score2'de)
                Superstructure.setShootSystem(BLUE_UZAK_SHOT_RPM_MULTIPLIER, BLUE_UZAK_HOOD_OFFSET_DEG);

                if (revolverEmpty) {
                    // İkinci shoot bitti, sistemleri kapat
                    Superstructure.stopShooting();
                    Superstructure.setFlywheelIdleMode(false);
                    Superstructure.flywheel.setSetpointRPM(0);
                    Superstructure.manualHoodControl = true;
                    Superstructure.manualTurretControl = true;
                    Superstructure.turret.setTurretAngle(0.0);
                    Superstructure.hood.setHoodAngle(0.0);
                    setPathState(8);
                }
                break;

            case 8:
                Superstructure.feeder.setFeedersMotor(0);

                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    follower.followPath(park);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    setPathState(-1); // Otonom bitti
                }
                break;
        }

        // FLYWHEEL RÖLANTI SİSTEMİ
        if (pathState != 2 && pathState != 7 && pathState != 8 && pathState != 9 && pathState != -1) {
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
        revolverEmptyDebouncer = new Debouncer(SHOT_EXIT_WAIT_SECONDS, Debouncer.DebounceType.kRising);

        follower = Constants.createFollower(hardwareMap);
        Superstructure.isauto=true;
        Superstructure.init(hardwareMap);
        Superstructure.reset();
        Superstructure.vision.LL3.pipelineSwitch(0);
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
        revolverEmptyDebouncer.reset(); // ← Debouncer'ı resetle!
        setPathState(0);
        telemetryM.debug("Autonomous started! 2-Shoot Uzak sequence");
    }

    @Override
    public void loop() {
        follower.update();

        Superstructure.read();
        Superstructure.periodic();
        Superstructure.write();
        Superstructure.isRevolverReady();

        autonomousPathUpdate();

        drawOnDashboard();

        PoseStorage.savePose(follower.getPose());

        telemetryM.debug("State", pathState);
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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        telemetryM.debug("Path state changed to: " + pState);
    }

    private void forcePark() {
        Superstructure.stopShooting();
        Superstructure.setFlywheelIdleMode(false);
        Superstructure.flywheel.setSetpointRPM(0);
        Superstructure.manualHoodControl = true;
        Superstructure.manualTurretControl = true;
        Superstructure.turret.setTurretAngle(0.0);
        Superstructure.hood.setHoodAngle(0.0);
        Superstructure.setIntakeSystem(0);
        Superstructure.feeder.setFeedersMotor(0);
        follower.followPath(park);
        setPathState(9);
        telemetryM.debug("Force park", opmodeTimer.getElapsedTimeSeconds());
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
