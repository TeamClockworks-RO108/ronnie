package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.field.TeleOpPoses;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Configurable
public class Flywheel {
    private final DcMotorEx rightMotor;
    private final DcMotorEx leftMotor;
    private final Telemetry telemetry;
    private final Follower follower;
    private final Servo hoodServo;
    private final Pose goalPose;
    public static PIDFCoefficients constants = new PIDFCoefficients(100, 0.1, 1.1, 15);
    public static double LINEAR_A = -5.357;
    public static double LINEAR_B = 0.294 - LINEAR_A * 0.6; // for accuracy in calculation
    private static double G = 9.80665;
    public static double V_TICKS = 3050;
    private static double ROBOT_H = 30.0 / 100;
    public static double GOAL_H = 120.0 / 100;
    public static double RADIUS = 48.0 / 1000;
    private static final double SERVO_LOWER_BOUND = 0.1889;
    private static final double SERVO_UPPER_BOUND = 0.71;

    public static double OFFSET = 20.0;

    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose goalPose) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, constants);

        leftMotor = hardwareMap.get(DcMotorEx.class, "flywheel1");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, constants);

        hoodServo = hardwareMap.get(Servo.class, "hood");

        this.telemetry = telemetry;
        this.follower = follower;
        this.goalPose = goalPose;
        hoodServo.setPosition(0.44);
    }

    public void update() {
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, constants);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, constants);


        rightMotor.setVelocity(V_TICKS / 2);
        leftMotor.setVelocity(V_TICKS / 2);

        double ang = getAngle();

        if(!Double.isNaN(ang)) {
            double servoPos = MathUtil.clamp(toServoPos(ang), SERVO_LOWER_BOUND, SERVO_UPPER_BOUND);
            hoodServo.setPosition(servoPos);
        }

        panelsTelemetry.getTelemetry().addData("left velocity", leftMotor.getVelocity());
        panelsTelemetry.getTelemetry().addData("right velocity", rightMotor.getVelocity());
        panelsTelemetry.getTelemetry().addData("target velocity", V_TICKS / 2);

        telemetry.addData("Hood angle (degrees)", Math.toDegrees(ang));
        telemetry.addData("Distance to goal", distanceToGoal());
        telemetry.addData("Left velocity", leftMotor.getVelocity());
        telemetry.addData("Right velocity", rightMotor.getVelocity());
        telemetry.addData("Velocity avg", (leftMotor.getVelocity() + rightMotor.getVelocity()) / 2);
        panelsTelemetry.getTelemetry().update();
    }

    private double getAngle() {
        double distanceShoot = (distanceToGoal() + OFFSET) * 2.54 / 100;
        double V = toMps();

        double delta = Math.sqrt(pow(V, 4) - 2 * G * (GOAL_H - ROBOT_H) * pow(V, 2) - pow(G, 2) * pow(distanceShoot, 2));
        double denominator = (G * distanceShoot);

        double t1 = (pow(V, 2) + delta) / denominator;
        double t2 = (pow(V, 2) - delta) / denominator;

        double a1 = Math.atan(t1);
        double a2 = Math.atan(t2);
        double middle = Math.toRadians(50);
        double angle;

        if(Math.abs(middle - a1) > Math.abs(middle - a2)) {
            angle = a2;
        } else {
            angle = a1;
        }

        telemetry.addData("t1", t1);
        telemetry.addData("t2", t2);
        telemetry.addData("atan(t1)", Math.atan(t1));
        telemetry.addData("atan(t2)", Math.atan(t2));
        telemetry.addData("V (m/s)", V);
        telemetry.addData("servo pos", hoodServo.getPosition());
        telemetry.addData("target servo pos", toServoPos(angle));

        return angle;
    }

    private double pow(double v, int b) {
        if(b == 0) return 1;

        if(b % 2 == 0) {
            double res = pow(v, b / 2);
            return res * res;
        }

        return v * pow(v, b - 1);
    }

    private double toServoPos(double angle) {
        return (angle - LINEAR_B) / LINEAR_A;
    }

    private double distanceToGoal() {
        Pose curr = follower.getPose();

        double deltaX = goalPose.getX() - curr.getX();
        double deltaY = goalPose.getY() - curr.getY();
        return Math.sqrt(pow(deltaX, 2) + pow(deltaY, 2));
    }

    private double toMps() {
        return (leftMotor.getVelocity(AngleUnit.DEGREES) + rightMotor.getVelocity(AngleUnit.DEGREES)) / 2 * RADIUS;
    }
}
