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
            .mass(10.5)
            .forwardZeroPowerAcceleration(-28.632)
            .lateralZeroPowerAcceleration(-66.93984)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.00023)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0001, 0.01, 0.025))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.08, 0.0004, 0.01, 0.025))

            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, 0.025))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 1, 0.1, 0.025))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.009, 0.001, 0.0005, 0.6, 0.025220627)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(82.49931)
            .yVelocity(65.97648)
            .useBrakeModeInTeleOp(true);

    public static ThreeWheelIMUConstants localizerConstants =
            new ThreeWheelIMUConstants()
                    .forwardTicksToInches(.00199)
                    .strafeTicksToInches(-.00196)
                    .turnTicksToInches(-.00202)
                    .rightPodY(35 / (25.4)) // millimeters to inches
                    .leftPodY(-35 / (25.4))
                    .strafePodX(-5 / (25.4))
                    .rightEncoder_HardwareMapName("rightFront") // set manually
                    .leftEncoder_HardwareMapName("rightRear")
                    .strafeEncoder_HardwareMapName("leftFront")
                    .leftEncoderDirection(Encoder.FORWARD)
                    .rightEncoderDirection(Encoder.FORWARD)
                    .strafeEncoderDirection(Encoder.REVERSE)
                    .IMU_HardwareMapName("imu")
                    .IMU_Orientation(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                    RevHubOrientationOnRobot.UsbFacingDirection.UP
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
