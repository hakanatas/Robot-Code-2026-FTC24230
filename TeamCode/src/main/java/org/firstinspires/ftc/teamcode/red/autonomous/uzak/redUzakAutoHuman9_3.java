package org.firstinspires.ftc.teamcode.red.autonomous.uzak;

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

@Autonomous(name = "5-red_Uzak_Human_9+3")
public class redUzakAutoHuman9_3 extends OpMode {
    private static final double ROBOT_RADIUS = 9;
    private static final double FLYWHEEL_IDLE_RPM = 3700.0;
    private static final double HUMAN_WAIT_TIME = 1.0; // Human'da bekleme süresi
    TelemetryManager telemetryM;
    private Follower follower;
    private int pathState;
    private FieldManager panelsField;
    private Style robotStyle;
    private Style pathStyle;
    private Timer pathTimer, opmodeTimer;
    private Debouncer revolverEmptyDebouncer;

    // PATH POZISYONLARI
    private final Pose startPose = new Pose(87.700, 8.800, Math.toRadians(0));
    private final Pose score1Pose = new Pose(87.700, 18.000, Math.toRadians(0));
    private final Pose humanPose = new Pose(131.000, 8.800, Math.toRadians(0));
    private final Pose score2Pose = new Pose(84.000, 26.000, Math.toRadians(0));
    private final Pose GPPPoseBehind = new Pose(102.000, 35.000, Math.toRadians(0));
    private final Pose GPPPose = new Pose(132.000, 35.000, Math.toRadians(0));
    private final Pose score3Pose = new Pose(84.000, 27.500, Math.toRadians(0));
    private final Pose human2Pose = new Pose(131.000, 11.000, Math.toRadians(0));
    private final Pose score4Pose = new Pose(83.000, 26.000, Math.toRadians(0));
    private final Pose parkPose = new Pose(89.000, 26.000, Math.toRadians(0));

    // PATHLER
    private Path score1;
    private PathChain human, score2, GPPBehind, GPP, score3, human2, score4, park;

    public void buildPaths() {
        score1 = new Path(new BezierLine(startPose, score1Pose));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());

        human = follower.pathBuilder()
                .addPath(new BezierLine(score1Pose, humanPose))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), humanPose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(humanPose, score2Pose))
                .setLinearHeadingInterpolation(humanPose.getHeading(), score2Pose.getHeading())
                .build();

        GPPBehind = follower.pathBuilder()
                .addPath(new BezierLine(score2Pose, GPPPoseBehind))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), GPPPoseBehind.getHeading())
                .build();

        GPP = follower.pathBuilder()
                .addPath(new BezierLine(GPPPoseBehind, GPPPose))
                .setLinearHeadingInterpolation(GPPPoseBehind.getHeading(), GPPPose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose, score3Pose))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), score3Pose.getHeading())
                .build();

        human2 = follower.pathBuilder()
                .addPath(new BezierLine(score3Pose, human2Pose))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), human2Pose.getHeading())
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierLine(human2Pose, score4Pose))
                .setLinearHeadingInterpolation(human2Pose.getHeading(), score4Pose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(score4Pose, parkPose))
                .setLinearHeadingInterpolation(score4Pose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        boolean revolverEmptyRaw = !Superstructure.slot0.IsthereBall() && !Superstructure.slot2.IsthereBall() && !Superstructure.slot1.IsthereBall();
        boolean revolverEmpty = revolverEmptyDebouncer.calculate(revolverEmptyRaw);
        boolean revolverFull = Superstructure.slot0.IsthereBall() && Superstructure.slot2.IsthereBall() && Superstructure.slot1.IsthereBall();

        // 29.5 saniye timeout kontrolü - PARK'A GİT!
        if (opmodeTimer.getElapsedTimeSeconds() >= 29 && pathState != 18 && pathState != -1) {
            // Acil sistemleri kapat
            Superstructure.stopShooting();
            Superstructure.setFlywheelIdleMode(false);
            Superstructure.flywheel.setSetpointRPM(0);
            Superstructure.manualHoodControl = true;
            Superstructure.manualTurretControl = true;
            Superstructure.turret.setTurretAngle(0.0);
            Superstructure.hood.setHoodAngle(0.0);
            Superstructure.setIntakeSystem(0);
            Superstructure.feeder.setFeedersMotor(0);

            // PARK'A GİT!
            follower.followPath(park);
            setPathState(18);
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
                    // Human'a git + Intake AÇ
                    Superstructure.setIntakeSystem(1);
                    follower.followPath(human);
                    setPathState(3);
                }
                break;

            case 3:
                // Human'a gidiliyor - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    // Human'a ulaştık, BEKLE!
                    setPathState(4);
                }
                break;

            case 4:
                // HUMAN'DA BEKLEME - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (pathTimer.getElapsedTimeSeconds() > HUMAN_WAIT_TIME || revolverFull) {
                    // Bekleme bitti veya revolver dolu, score2'ye git
                    follower.followPath(score2);
                    setPathState(5);
                }
                break;

            case 5:
                // Score2'ye giderken - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    // Score2'ye ulaştık, intake'i kapat
                    Superstructure.setIntakeSystem(0);
                    setPathState(6);
                }
                break;

            case 6:
                // Robot stabilize olsun
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    setPathState(7);
                }
                break;

            case 7:
                // İkinci shoot (score2)
                Superstructure.setShootSystem();

                if (revolverEmpty) {
                    Superstructure.stopShooting();
                    // GPPBehind'a git + Intake AÇ
                    Superstructure.setIntakeSystem(1);
                    follower.followPath(GPPBehind);
                    setPathState(8);
                }
                break;

            case 8:
                // GPPBehind'a gidiliyor - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    // GPPBehind'a ulaştık, GPP'ye git
                    follower.followPath(GPP);
                    setPathState(9);
                }
                break;

            case 9:
                // GPP'ye giderken - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy() || revolverFull) {
                    // GPP bitti veya revolver dolu, score3'e git
                    follower.followPath(score3);
                    setPathState(10);
                }
                break;

            case 10:
                // Score3'e giderken - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    // Score3'e ulaştık, intake'i kapat
                    Superstructure.setIntakeSystem(0);
                    setPathState(11);
                }
                break;

            case 11:
                // Robot stabilize olsun
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    setPathState(12);
                }
                break;

            case 12:
                // Üçüncü shoot (score3)
                Superstructure.setShootSystem();

                if (revolverEmpty) {
                    Superstructure.stopShooting();
                    // Human2'ye git + Intake AÇ
                    Superstructure.setIntakeSystem(1);
                    follower.followPath(human2);
                    setPathState(13);
                }
                break;

            case 13:
                // Human2'ye gidiliyor - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    // Human2'ye ulaştık, BEKLE!
                    setPathState(14);
                }
                break;

            case 14:
                // HUMAN2'DE BEKLEME - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (pathTimer.getElapsedTimeSeconds() > HUMAN_WAIT_TIME || revolverFull) {
                    // Bekleme bitti veya revolver dolu, score4'e git
                    follower.followPath(score4);
                    setPathState(15);
                }
                break;

            case 15:
                // Score4'e giderken - intake AÇIK
                Superstructure.setIntakeSystem(1);

                if (!follower.isBusy()) {
                    // Score4'e ulaştık, intake'i kapat
                    Superstructure.setIntakeSystem(0);
                    setPathState(16);
                }
                break;

            case 16:
                // Robot stabilize olsun
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    setPathState(17);
                }
                break;

            case 17:
                // Dördüncü shoot (score4) - SON SHOOT!
                Superstructure.setShootSystem();

                if (revolverEmpty) {
                    // Dördüncü shoot bitti, sistemleri kapat
                    Superstructure.stopShooting();
                    Superstructure.setFlywheelIdleMode(false);
                    Superstructure.flywheel.setSetpointRPM(0);
                    Superstructure.manualHoodControl = true;
                    Superstructure.manualTurretControl = true;
                    Superstructure.turret.setTurretAngle(0.0);
                    Superstructure.hood.setHoodAngle(0.0);
                    Superstructure.feeder.setFeedersMotor(0);

                    // Hood kapanmasını bekle
                    setPathState(18);
                }
                break;

            case 18:
                // Park pozisyonuna git
                if (!follower.isBusy()) {
                    // Park path henüz başlatılmamışsa başlat
                    follower.followPath(park);
                    setPathState(19);
                }
                break;

            case 19:
                // Park pozisyonuna gidiliyor
                if (!follower.isBusy()) {
                    // Park'a ulaştık
                    setPathState(-1); // Otonom bitti
                }
                break;
        }

        // FLYWHEEL RÖLANTI SİSTEMİ
        if (pathState != 2 && pathState != 7 && pathState != 12 && pathState != 17 && pathState != 18 && pathState != 19 && pathState != -1) {
            Superstructure.flywheel.setSetpointRPM(FLYWHEEL_IDLE_RPM);
            Superstructure.feeder.setFeedersMotor(0);
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Debouncer oluştur - 0.15 saniye, Rising edge
        revolverEmptyDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kRising);

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
        setPathState(0);
        telemetryM.debug("Autonomous started! 4-Shoot HUMAN sequence (15-BALL)");
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