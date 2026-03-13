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
            .forwardZeroPowerAcceleration(-40.32)
            .lateralZeroPowerAcceleration(-54.55)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.00023)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.01, 0.03))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.16, 0.0003, 0.015, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0.0, 0.07, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.09, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0065, 0, 0.00072, 0.6, 0)
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
            .xVelocity(157.9222)
            .yVelocity(40.1784)
            .useBrakeModeInTeleOp(true);

    public static ThreeWheelIMUConstants localizerConstants =
            new ThreeWheelIMUConstants()
                    .forwardTicksToInches(.001976)
                    .strafeTicksToInches(-.001986)
                    .turnTicksToInches(.002026)
                    .rightPodY(60 / (25.4)) // millimeters to inches
                    .leftPodY(-60 / (25.4))
                    .strafePodX(-4 / (25.4))
                    .rightEncoder_HardwareMapName("leftIntake") // set manually
                    .leftEncoder_HardwareMapName("leftFront")
                    .strafeEncoder_HardwareMapName("rightFront")
                    .leftEncoderDirection(Encoder.FORWARD)
                    .rightEncoderDirection(Encoder.FORWARD)
                    .strafeEncoderDirection(Encoder.FORWARD)
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
