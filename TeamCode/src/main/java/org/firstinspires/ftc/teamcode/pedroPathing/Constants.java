package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-36.5)
            .lateralZeroPowerAcceleration(-61.65)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.00023)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.18, 0.001, 0.01, 0.025))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0.001, 0.01, 0.025))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0.6, 0.025220627)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(87.1222)
            .yVelocity(69.911)
            .useBrakeModeInTeleOp(true);

    public static ThreeWheelIMUConstants localizerConstants =
            new ThreeWheelIMUConstants()
                    .forwardTicksToInches(.006086)
                    .strafeTicksToInches(.001536)
                    .turnTicksToInches(-.002026)
                    .rightPodY(62 / (25.4)) // millimeters to inches
                    .leftPodY(-62 / (25.4))
                    .strafePodX(-150 / (25.4))
                    .rightEncoder_HardwareMapName("rightFront") // set manually
                    .leftEncoder_HardwareMapName("leftIntake")
                    .strafeEncoder_HardwareMapName("leftFront")
                    .leftEncoderDirection(Encoder.FORWARD)
                    .rightEncoderDirection(Encoder.FORWARD)
                    .strafeEncoderDirection(Encoder.REVERSE)
                    .IMU_HardwareMapName("imu")
                    .IMU_Orientation(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                            )
                    );

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            2.67,
            4
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
