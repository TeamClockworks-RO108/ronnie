package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.field.poses.Poses;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.VVPidf;

@Configurable
public class Flywheel {
    private final DcMotorEx rightMotor;
    private final DcMotorEx leftMotor;
    private final Telemetry telemetry;
    private final Follower follower;
    private final Servo hoodServo;
//    public static PIDFCoefficients constants = new PIDFCoefficients(0.007, 0, 0.0001, 0.00039);
    public static PIDFCoefficients constants = new PIDFCoefficients(0.00835, 0.000001, 0.00006, 0.000357);
    public static double LINEAR_A = 1.68;
    public static double LINEAR_B = -0.905;
    private static double G = 9.80665;
    public static double V_TICKS_FAR = 1450;
    public static double V_TICKS_CLOSE = 1300;
    public static double V_TICKS_VERY_FAR = 1650;
    private double velocity = .0;
    private static double ROBOT_H = 30.0 / 100;
    public static double GOAL_H = 120.0 / 100;
    public static double RADIUS = 48.0 / 1000;
    private static final double SERVO_LOWER_BOUND = 0.27;
    private static final double SERVO_UPPER_BOUND = 1.0;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    public static final double CLOSE_DIST = 88.582;
    public static final double FAR_DIST = 131;
    private final Poses poses;
    private final InterpLUT interp = new InterpLUT();

//    private final PIDFController pid;

    private final VVPidf pidf;
    private final PIDFController pid;

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Poses poses) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor = hardwareMap.get(DcMotorEx.class, "flywheel1");
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        pid = new PIDFController(constants);

        pidf = new VVPidf();

        this.telemetry = telemetry;
        this.follower = follower;
        this.poses = poses;

//        interp.add(130, 1500);
//        interp.add(133, 1670);
//        interp.add(145, 1700);
//        interp.add(160, 1800);
//        interp.add(500, 1800);
        interp.add(0, 1450);
        interp.add(131, 1450);
        interp.add(140, 1600);
//        interp.add(150, 1700);
        interp.add(500, 1600);

        interp.createLUT();
    }

    public void update() {
        pidf.set_coeffs(constants.P, constants.D, constants.F, 0);

        updateSpeed();

//        pid.updateError(velocity - getVelocity());
//        double pow = pid.run();

        double pow = pidf.update(getVelocity(), velocity);

        leftMotor.setPower(pow);
        rightMotor.setPower(pow);

        double ang = getAngle();

        if (!Double.isNaN(ang)) {
            double servoPos = MathUtil.clamp(toServoPos(ang), SERVO_LOWER_BOUND, SERVO_UPPER_BOUND);

            if(distanceToGoal() > FAR_DIST) {
                //dist further than FAR_DIST cannot be achieved with
                // any angle at a certain speed with calculation
                servoPos = SERVO_LOWER_BOUND;
            }

            hoodServo.setPosition(servoPos);
        }

        panelsTelemetry.getTelemetry().addData("l_v", leftMotor.getVelocity());
        panelsTelemetry.getTelemetry().addData("r_v", rightMotor.getVelocity());
        panelsTelemetry.getTelemetry().addData("Current Velocity", getVelocity());
        panelsTelemetry.getTelemetry().addData("Target velocity", velocity);

        telemetry.addData("Hood angle (degrees)", Math.toDegrees(ang));
        telemetry.addData("Distance to goal", distanceToGoal());

        panelsTelemetry.getTelemetry().update();
    }

    private double getAngle() {
        double distanceShoot = (distanceToGoal()) * 2.54 / 100;
        double V = toMps();

        double delta = Math.sqrt(pow(V, 4) - 2 * G * (GOAL_H - ROBOT_H) * pow(V, 2) - pow(G, 2) * pow(distanceShoot, 2));
        double denominator = (G * distanceShoot);

        double t1 = (pow(V, 2) + delta) / denominator;
        double t2 = (pow(V, 2) - delta) / denominator;

        double a1 = Math.atan(t1);
        double a2 = Math.atan(t2);
        double p1 = toServoPos(a1);
        double p2 = toServoPos(a2);
        double middle = (SERVO_LOWER_BOUND + SERVO_UPPER_BOUND) / 2;
        double angle;

        if(inRange(p1, SERVO_LOWER_BOUND, SERVO_UPPER_BOUND) && !inRange(p2, SERVO_LOWER_BOUND, SERVO_UPPER_BOUND)) {
            angle = a1;
        } else if (inRange(p2, SERVO_LOWER_BOUND, SERVO_UPPER_BOUND) && !inRange(p1, SERVO_LOWER_BOUND, SERVO_UPPER_BOUND)) {
            angle = a2;
        } else if (Math.abs(middle - p1) > Math.abs(middle - p2)) {
            angle = a2;
        } else {
            angle = a1;
        }

        telemetry.addData("Angle 1", Math.toDegrees(a1));
        telemetry.addData("Angle 2", Math.toDegrees(a2));
        telemetry.addData("Target angle", Math.toDegrees(angle));
        telemetry.addData("V (m/s)", V);
        telemetry.addData("Target servo pos", toServoPos(angle));

        return angle;
    }

    private double pow(double v, int b) {
        if (b == 0) return 1;

        if (b % 2 == 0) {
            double res = pow(v, b / 2);
            return res * res;
        }

        return v * pow(v, b - 1);
    }

    private double toServoPos(double angle) {
        return angle * LINEAR_A + LINEAR_B;
    }

    private void updateSpeed() {
        double dist = distanceToGoal();

        if(dist <= CLOSE_DIST)  {
            velocity = V_TICKS_CLOSE;
        } else {
            velocity = interp.get(dist);
        }
    }

    private boolean inRange(double val, double l, double r) {
        return val >= l && val <= r;
    }

    private double distanceToGoal() {
        Pose curr = follower.getPose();

        double deltaX = poses.goal().getX() - curr.getX();
        double deltaY = poses.goal().getY() - curr.getY();
        return Math.sqrt(pow(deltaX, 2) + pow(deltaY, 2));
    }

    private double toMps() {
        return leftMotor.getVelocity(AngleUnit.DEGREES) * RADIUS;
    }

    // later make to switch between encoders
    private double getVelocity() {
        return leftMotor.getVelocity();
    }
}
