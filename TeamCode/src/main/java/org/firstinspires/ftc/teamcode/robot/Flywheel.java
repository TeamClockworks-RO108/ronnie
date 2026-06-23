package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Flywheel {
    private final DcMotorEx rightMotor;
    private final DcMotorEx leftMotor;
    private final Telemetry telemetry;
    private final Follower follower;
    private final Servo hoodServo;
    private final Pose goalPose;
    public static PIDFCoefficients constants = new PIDFCoefficients(.008, .000003, .00005, .003);
    private final PIDFController controller;
    public static double LINEAR_A = -5.357;
    public static double LINEAR_B = 0.294 - LINEAR_A * 0.6; // for accuracy in calculation
    private static double G = 9.80665;
    public static double V_TICKS = 1200;
    private static double ROBOT_H = 30.0 / 100;
    public static double GOAL_H = 106.0 / 100;
    public static double RADIUS = 48.0 / 1000;

    public static double TICKS_PER_REV = 28;

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose goalPose) {
        this.controller = new PIDFController(constants);

        rightMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor = hardwareMap.get(DcMotorEx.class, "flywheel1");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, "hood");


        this.telemetry = telemetry;
        this.follower = follower;
        this.goalPose = goalPose;
    }

    public void update() {
        rightMotor.setVelocity(V_TICKS / 2);
        leftMotor.setVelocity(V_TICKS / 2);

        double ang = getAngle();
        telemetry.addData("Hood required angle", Math.toDegrees(ang));
        telemetry.addData("Distance to goal", distanceToGoal());
    }

    private double getAngle() {
        double distanceShoot = distanceToGoal() / 100;
        double V = toMps(V_TICKS);

        double delta = Math.sqrt(pow(V, 4) - 2 * G * (GOAL_H - ROBOT_H) * pow(V, 2) - pow(G, 2) * pow(distanceShoot, 2));

        double denominator = (G * distanceShoot);

        double t1 = (pow(V, 2) + delta) / denominator;
        double t2 = (pow(V, 2) - delta) / denominator;

        telemetry.addData("t1", t1);
        telemetry.addData("t2", t2);
        telemetry.addData("V (m/s)", V);
        telemetry.addData("V ticks (ticks/s)", V_TICKS);

        return Math.atan(Math.max(t1, t2));
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

    public double distanceToGoal() {
        Pose curr = follower.getPose();

        double deltaX = Math.abs(goalPose.getX() - curr.getX());
        double deltaY = Math.abs(goalPose.getY() - curr.getY());
        return Math.sqrt(pow(deltaX, 2) + pow(deltaY, 2));
    }

    public double toMps(double ticks) {
        return 2 * RADIUS * Math.PI * ticks / TICKS_PER_REV;
    }
}
