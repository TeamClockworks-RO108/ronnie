package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Drawing;

@Configurable
public class Turret {
    private static final double TICKS_PER_180 = 14610;
    private static final double RADIANS_TO_ENCODER_TICKS = TICKS_PER_180 / Math.PI;
//    private static final double AUTO_START_OFFSET = -7;
    private static final double TURRET_LOWER_BOUND = -Math.PI * 5 / 8;
    private static final double TURRET_UPPER_BOUND = Math.PI / 2;

    private final Telemetry telemetry;
    private final Follower follower;

    public static double goalTargetX;
    public static double goalTargetY;

    private final PIDFController pid;

    private final PIDFController pidMovement;

    private final DcMotor encoderMotor;
    private final CRServo headingServo0, headingServo1;

    private boolean isOverride = false;

    private long correctionOffset = 0;
    private long beforeCorrectionOffset = 0;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(1.2, 0.3, 0.05, 0.3);

    // This moves the turret anticipatively by joystick input. works only in teleop.
    private static PIDFCoefficients predictiveMovementControl = new PIDFCoefficients(0.13, 0, 0.01, 0);
    private final Supplier<Double> wheelRotationPower;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose goalTarget,
                  Supplier<Double> wheelRotationPower, boolean reset) {
        encoderMotor = hardwareMap.get(DcMotor.class, "leftFront");

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

        /*  old encoder resetting logic
            THE CALLS ARE ASYNCHRONOUS AND BEHAVE WEIRDLY

            ncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // This zeros the turret at each opmode start.
            overrideCorrectionOffset = - getEncoder();
        */

        if (reset) {
            encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // This zeros the turret at each opmode start.
            correctionOffset = - getEncoder();
        } else {
            correctionOffset = 0;
        }
    }

    public void update() {
        double angleError = computeAngleMotion();

        pid.updateError(angleError);
        double power = pid.run();

        power = exponentialPowerAlgo(Math.abs(power)) * Math.signum(power);

        pidMovement.updateError(wheelRotationPower.get());
        power += pidMovement.run();

        if (!isOverride) {
            setTurretPower(power);
        }

        telemetry.addData("Turret Power", power);
        telemetry.addData("Turret Encoder", getEncoder());
        telemetry.addData("Turret Override", isOverride);

        // if (pidfCoefficients != pid.getCoefficients()) {
        //     pid.setCoefficients(pidfCoefficients);
        // }
    }

    private Pose overridePose = null;

    /**
     *
     * @param pose set to null to not use override
     */
    public void setOverridePose(Pose pose) {
       this.overridePose = pose;
    }

    private Pose getRobotPose() {
        return overridePose != null ? overridePose : follower.getPose();
    }

    private double computeAngleMotion() {
        double turretLocalAngle = (getEncoder() / TICKS_PER_180) * Math.PI;
        turretLocalAngle = AngleUnit.normalizeRadians(turretLocalAngle);

        double robotx = getRobotPose().getX();
        double roboty = getRobotPose().getY();
        

         double robotGlobalAngle = getRobotPose().getHeading();

        double turretGlobalAngle = AngleUnit.normalizeRadians(robotGlobalAngle + turretLocalAngle);

        // field goal location
        double goalGlobalAngle = Math.atan2(goalTargetY - roboty, goalTargetX - robotx);
        double goalLocalAngle = AngleUnit.normalizeRadians(goalGlobalAngle - robotGlobalAngle);

        double errorAngle = computeTurretError(goalLocalAngle, turretLocalAngle);

        // telemetry
        Drawing.drawRobot(new Pose(robotx, roboty), Drawing.turretLook);

        telemetry.addData("turretGlobalAngle", Math.toDegrees(turretGlobalAngle));
        telemetry.addData("turretLocalAngle", Math.toDegrees(turretLocalAngle));
        telemetry.addData("goalGlobalAngle", Math.toDegrees(goalGlobalAngle));
        telemetry.addData("goalLocalAngle", Math.toDegrees(goalLocalAngle));
        telemetry.addData("errorAngle", errorAngle);

        return errorAngle;
    }

    private static double computeTurretError(double target, double current) {
        target = AngleUnit.normalizeRadians(target);
        current = AngleUnit.normalizeRadians(current);

        // clamp target into legal region
        target = clamp(target);

        double error = target - current;

        return error;
    }

    private static double clamp(double angle) {
        return Math.max(TURRET_LOWER_BOUND, Math.min(angle, TURRET_UPPER_BOUND));
    }

    private static boolean isWithinBoundaries(double angle) {
        return angle >= TURRET_LOWER_BOUND && angle <= TURRET_UPPER_BOUND;
    }

    private long getEncoder() {
        return encoderMotor.getCurrentPosition() + correctionOffset;
    }
    private double exponentialPowerAlgo(double power) {
        double interior = power * 4 + 0.9;
        return ((Math.log10(interior) / Math.log10(2.71)) / 1.56) * 0.3 + power * 0.7;
    }

    private void setTurretPower(double power) {
        headingServo0.setPower(power);
        headingServo1.setPower(power);
    }

    public void manualOverride(double power) {
        if (Math.abs(power) < 0.01) {
            if (isOverride) {
                correctionOffset -= (getEncoder() - beforeCorrectionOffset);
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