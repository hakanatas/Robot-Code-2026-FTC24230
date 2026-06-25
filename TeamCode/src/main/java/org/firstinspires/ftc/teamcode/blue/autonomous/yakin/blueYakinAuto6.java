package org.firstinspires.ftc.teamcode.blue.autonomous.yakin;

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

@Autonomous(name = "9-blue_Yakın_6")
public class blueYakinAuto6 extends OpMode {
    private static final double ROBOT_RADIUS = 9;
    private static final double FLYWHEEL_IDLE_RPM = 3000.0;
    private static final double FORCE_STOP_TIME_SECONDS = 29.0;
    TelemetryManager telemetryM;
    private Follower follower;
    private int pathState;
    private FieldManager panelsField;
    private Style robotStyle;
    private Style pathStyle;
    private Timer pathTimer, opmodeTimer;
    private Debouncer revolverEmptyDebouncer;

    // PATH POZISYONLARI
    private final Pose startPose = new Pose(26.300, 129.350, Math.toRadians(136));
    private final Pose score1Pose = new Pose(60.000, 84.300, Math.toRadians(180));
    private final Pose PPGPoseBehind = new Pose(42.000, 84.300, Math.toRadians(180));
    private final Pose PPGPose = new Pose(19.000, 84.300, Math.toRadians(180));
    private final Pose score2Pose = new Pose(53.000, 112.000, Math.toRadians(180));

    // PATHLER
    private Path score1;
    private PathChain PPGBehind, PPG, score2;

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
    }

    public void autonomousPathUpdate() {
        boolean revolverEmptyRaw = !Superstructure.slot0.IsthereBall() && !Superstructure.slot2.IsthereBall() && !Superstructure.slot1.IsthereBall();
        boolean revolverEmpty = revolverEmptyDebouncer.calculate(revolverEmptyRaw);
        boolean revolverFull = Superstructure.slot0.IsthereBall() && Superstructure.slot2.IsthereBall() && Superstructure.slot1.IsthereBall();

        if (opmodeTimer.getElapsedTimeSeconds() >= FORCE_STOP_TIME_SECONDS
                && pathState != -1) {
            forceStop();
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
                Superstructure.setShootSystem();

                if (revolverEmpty) {
                    Superstructure.stopShooting();
                    follower.followPath(PPGBehind);
                    setPathState(3);
                }
                break;

            case 3:
                Superstructure.setIntakeSystem(1); // ← Intake aç

                if (!follower.isBusy()) {
                    // PPGBehind'a ulaştık, şimdi YAVAŞ başla ve intake aç
                    follower.followPath(PPG);
                    setPathState(4);
                }
                break;

            case 4:
                // PPG'ye giderken - YAVAŞ, intake AÇIK (TOPLAR BURADA!)
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy() || revolverFull) {
                    // PPG bitti, şimdi score2'ye HIZLI git
                    follower.followPath(score2);
                    setPathState(5);
                }
                break;

            case 5:
                // Score2'ye giderken - HIZLI, intake AÇIK
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
                Superstructure.setShootSystem();

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
                // Hood'un kapanmasını bekle
                Superstructure.feeder.setFeedersMotor(0);
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(-1); // Otonom bitti
                }
                break;
        }

        // FLYWHEEL RÖLANTI SİSTEMİ
        if (pathState != 2 && pathState != 7 && pathState != 8 && pathState != -1) {
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
        revolverEmptyDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kRising);

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
        revolverEmptyDebouncer.reset();
        setPathState(0);
        telemetryM.debug("Autonomous started! 2-Shoot sequence");
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

        // Telemetry
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

    private void forceStop() {
        Superstructure.stopShooting();
        Superstructure.setFlywheelIdleMode(false);
        Superstructure.flywheel.setSetpointRPM(0);
        Superstructure.manualHoodControl = true;
        Superstructure.manualTurretControl = true;
        Superstructure.turret.setTurretAngle(0.0);
        Superstructure.hood.setHoodAngle(0.0);
        Superstructure.setIntakeSystem(0);
        Superstructure.feeder.setFeedersMotor(0);
        setPathState(-1);
        telemetryM.debug("Force stop", opmodeTimer.getElapsedTimeSeconds());
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
