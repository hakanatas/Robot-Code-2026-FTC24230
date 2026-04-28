package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(20.0)
            .forwardZeroPowerAcceleration(-32.913194232941784)
            .lateralZeroPowerAcceleration(-40.49923109206457)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0.0,0.01,0.03))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0.0,0.01,0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(2.0,0.0,0.1,0.0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3.0,0.0,0.1,0.0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005,0.0,0.00015,0.3,0.0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.0,0.0005,0.3,0.0))
            .centripetalScaling(0.0009);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeftMotor")
            .leftRearMotorName("backLeftMotor")
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(70.63213486558809)
            .yVelocity(64.49939349317175);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(DistanceUnit.INCH.fromMm(-22.99))
            .strafePodX(DistanceUnit.INCH.fromMm(-163.51))
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("LocalizerModule")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.950,
            0.1,
            0.1,
            0.007,
            10,
            1.0,
            10,
            1.0
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

