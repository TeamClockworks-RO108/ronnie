package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
            .mass(14.5)
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
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0065, 0, 0.00072, 0.6, 0))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.11, 0.07944204113241114, 0.0024323399908359493));

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

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.36)
            .strafePodX(-7.08)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            ;


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            2.67,
            4
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
