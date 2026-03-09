package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Flywheel {
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Telemetry telemetry;

    private final Follower follower;

    private final Servo hoodServo;
    private static final double HOOD_LIFTED = 0.8, HOOD_LOWERED = 0.2;

    // tbd ok ok tbd
    private static double farHood = 0.72, centerHood = 0.43, defaultHood = 0.58, closeHood = 0.20;
    private static double farSpeed = 1500, centerSpeed = 1240, defaultSpeed = 1150, closeSpeed = 1000;

    private static double farDistance = 115, centerDistance = 82, defaultDistance = 47, closeDistance = 32;

    private final PIDFCoefficients constants = new PIDFCoefficients();
    public static double kp = 300;
    public static double ki = 14;
    public static double kd = 5;
    public static double kf = 0;

    public static double aimingTarget;

    public static double idleSpeed = 300;

    public boolean running = false;

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
        constants.d = kd;
        constants.p = kp;
        constants.i = ki;
        constants.f = kf;

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftFlywheel");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                constants);

        rightMotor = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                constants);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, "hood");

        this.telemetry = telemetry;
        this.follower = follower;
    }

    public double distanceToGoal() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        // coordinates of goal tune with limelight at runtime
        double xx = x - 12;
        double yy = y - 132;

        return Math.sqrt(xx * xx + yy * yy);
    }


    public void update() {
        double distanceShoot = distanceToGoal();
        telemetry.addData("leftFlywheel TPS", leftMotor.getVelocity());
        telemetry.addData("rightFlywheel TPS", rightMotor.getVelocity());
        telemetry.addData("Distance to goal", distanceShoot);

        boolean changed = false;

        if (running) {
            double calculatedVelocity = 0, hoodPosition = 0;

            double [] distances = new double[] { closeDistance, defaultDistance, centerDistance, farDistance, farDistance + 999 };
            double [] speeds = new double[] { closeSpeed, defaultSpeed, centerSpeed, farSpeed, farSpeed };
            double [] hoods = new double[] { closeHood, defaultHood, centerHood, farHood, farHood };

            int index = -1;

            for (int i = 0; i < distances.length - 1; i++) {
                if (distances[i] < distanceShoot && distances[i+1] > distanceShoot) {
                    index = i;
                    break;
                }
            }

            if (index == -1) {
                if (distanceShoot > farDistance)
                    index = distances.length - 1;
                else
                    index = 0;
            }




            double extraDistance = distanceShoot - distances[index];
            double totalDelta = distances[index + 1] - distances[index];
            double coeff = extraDistance / totalDelta;

            telemetry.addData("Flyheel Index:", index);
            telemetry.addData("Flywheel Coeff:", coeff);

            calculatedVelocity = speeds[index] * (1 - coeff) + speeds[index + 1] * coeff;
            hoodPosition = hoods[index] * (1 - coeff) + hoods[index + 1] * coeff;

            if (hoodServo.getPosition() != hoodPosition)
                hoodServo.setPosition(hoodPosition);

            if (calculatedVelocity != aimingTarget) {
                aimingTarget = calculatedVelocity;
                changed = true;
            }
        } else {
            if (aimingTarget != idleSpeed)  {
                aimingTarget = idleSpeed;
                changed = true;
            }
        }

        if (changed) {
            leftMotor.setVelocity(aimingTarget);
            rightMotor.setVelocity(aimingTarget);
        }
        //  flywheel1.setVelocity(targetVelocity);

        // Changed is reused here
        changed = constants.d != kd ||
                constants.p != kp ||
                constants.i != ki ||
                constants.f != kf;

        if (changed) {
            constants.d = kd;
            constants.p = kp;
            constants.i = ki;
            constants.f = kf;
            leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    constants);
            rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    constants);



        }
    }

    public void start() {
        running = true;
    }

    public void idle(){
        running = true;
    }
   public void stop() {
       running = false;
       aimingTarget = 0;
    }
}
