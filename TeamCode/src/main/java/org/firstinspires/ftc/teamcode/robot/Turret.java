package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Flywheel.CORR_OFFSET_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.Flywheel.TURRET_TO_ODOM;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Drawing;

@Configurable
public class Turret {
    private static double TICKS_PER_180 = 14610;

    private final Telemetry telemetry;
    private final Follower follower;
    public static double goalTargetX;
    public static double goalTargetY;

    private final PIDFController pid;

    private final PIDFController pidMovement;

    private final DcMotor encoderMotor;
    private final CRServo headingServo0, headingServo1;

    private boolean isOverride = false;

    private long overrideCorrectionOffset = 0;
    private long beforeCorrectionOffset = 0;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(1.2, 0.1, 0.05, 0.3);

    // This moves the turret anticipatively by joystick input. works only in teleop.
    private static PIDFCoefficients predictiveMovementControl = new PIDFCoefficients(0.13, 0, 0.01, 0);
    private final Supplier<Double> wheelRotationPower;

    public Turret (HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose goalTarget, boolean reset, Supplier<Double> wheelRotationPower) {
        encoderMotor = hardwareMap.get(DcMotor.class, "rightIntake");

        headingServo0 = hardwareMap.get(CRServo.class, "heading0");
        headingServo0.setDirection(CRServo.Direction.REVERSE);

        headingServo1 = hardwareMap.get(CRServo.class, "heading1");
        headingServo1.setDirection(CRServo.Direction.REVERSE);

        this.telemetry = telemetry;
        this.follower = follower;
        goalTargetX = goalTarget.getX();
        goalTargetY = goalTarget.getY();
        this.wheelRotationPower = wheelRotationPower;



        pid = new PIDFController(pidfCoefficients);
        pidMovement = new PIDFController(predictiveMovementControl);

        if (reset) {
            encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // This zeros the turret at each opmode start.
        overrideCorrectionOffset = - getEncoder();
    }

    private long getEncoder() {
        return encoderMotor.getCurrentPosition() + overrideCorrectionOffset;
    }

    public void update() {
        long encPos = getEncoder();
        double turretPosition = (encPos / TICKS_PER_180) * Math.PI;

        double turretAngle = follower.getPose().getHeading() + Math.toRadians(CORR_OFFSET_ANGLE);
        double px = Math.cos(turretAngle) * TURRET_TO_ODOM;
        double py = Math.sin(turretAngle) * TURRET_TO_ODOM;

        double robotx = follower.getPose().getX() + px;
        double roboty = follower.getPose().getY() + py;

        Drawing.drawRobot(new Pose(robotx, roboty), Drawing.turretLook);

        double angleToGoal = - AngleUnit.normalizeRadians(Math.PI - Math.atan2(goalTargetY - roboty, goalTargetX - robotx));
        double robotHeading =  AngleUnit.normalizeRadians(follower.getPose().getHeading() - Math.PI);

        double angleTurret = AngleUnit.normalizeRadians(angleToGoal - robotHeading);

        angleTurret = Math.max(angleTurret, -Math.PI );
        angleTurret = Math.min(angleTurret,  Math.PI/2);

        pid.updateError(angleTurret - turretPosition);
        double power = pid.run();

        power = exponentialPowerAlgo(Math.abs(power)) * Math.signum(power);

        pidMovement.updateError(wheelRotationPower.get());
        power += pidMovement.run();

        if (!isOverride) {
            setTurretPower(power);
        }

        telemetry.addData("Turret Power", power);
        telemetry.addData("Angle Robot to Goal", Math.toDegrees(angleToGoal));
        telemetry.addData("Angle Turret to Goal", Math.toDegrees(angleTurret));
        telemetry.addData("Turret Encoder", encPos);
        telemetry.addData("Turret Override", isOverride);

        //if (pidfCoefficients != pid.getCoefficients()) {
        //    pid.setCoefficients(pidfCoefficients);
        //}
    }

    private double exponentialPowerAlgo(double power) {
        double interior = power * 4 + 0.9;
        return ((Math.log10(interior) / Math.log10(2.71)) / 1.56) * 0.3 + power * 0.7;
    }


    private void setTurretPower(double power){
        headingServo0.setPower(power);
        headingServo1.setPower(power);
    }

    public void manualOverride(double power) {
        if (Math.abs(power) < 0.01) {
            if (isOverride) {
                overrideCorrectionOffset -= (getEncoder() - beforeCorrectionOffset);
            }
            isOverride = false;
        } else {
            if (!isOverride) {
                beforeCorrectionOffset = getEncoder();
            }
            isOverride = true;
            setTurretPower(power);
        }

    }
}
