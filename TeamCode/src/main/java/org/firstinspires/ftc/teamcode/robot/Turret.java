package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class Turret {
    private static double TICKS_PER_180 = 14610;

    private final Telemetry telemetry;
    private final Follower follower;
    private final Pose goalTarget;

    private final PIDFController pid;

    private final DcMotor encoderMotor;
    private final DcMotor headingMotor;

    private boolean isOverride = false;

    private long overrideCorrectionOffset = 0;
    private long beforeCorrectionOffset = 0;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(1.4, 0, 0.1, 1);

    public Turret (HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose goalTarget){
        encoderMotor = hardwareMap.get(DcMotor.class, "rightIntake");

        headingMotor = hardwareMap.get(DcMotor.class, "heading");
        headingMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;
        this.follower = follower;
        this.goalTarget = goalTarget;

        pid = new PIDFController(pidfCoefficients);

        // This zeros the turret at each opmode start.
        overrideCorrectionOffset = - getEncoder();
    }

    private long getEncoder() {
        return encoderMotor.getCurrentPosition() + overrideCorrectionOffset;
    }

    public void update() {
        long encPos = getEncoder();
        double turretPosition = (encPos / TICKS_PER_180) * Math.PI;

        double robotx = follower.getPose().getX();
        double roboty = follower.getPose().getY();
        double angleToGoal = - AngleUnit.normalizeRadians(Math.PI - Math.atan2(goalTarget.getY() - roboty, goalTarget.getX() - robotx));
        double robotHeading =  AngleUnit.normalizeRadians(follower.getPose().getHeading() - Math.PI);

        double angleTurret = AngleUnit.normalizeRadians(angleToGoal - robotHeading);

        angleTurret = Math.max(angleTurret, -Math.PI * 3 / 2);
        angleTurret = Math.min(angleTurret, 3 * Math.PI / 4);

        pid.updateError(angleTurret - turretPosition);
        double power = pid.run();
        power = exponentialPowerAlgo(Math.abs(power)) * Math.signum(power);
        if (!isOverride) {
            setTurretPower(-power);
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
        headingMotor.setPower(power);
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
