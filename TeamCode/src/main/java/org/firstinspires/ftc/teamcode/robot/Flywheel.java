package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Flywheel {
    private final DcMotorEx rightMotor;
    private final Telemetry telemetry;

    private final Follower follower;
    private final Servo hoodServo;

    private final Pose targetPose;

    private static double farHood = 0.72, centerHood = 0.7, defaultHood = 0.35, closeHood = 0.20;
    private static double farSpeed = 1500, centerSpeed = 1250, defaultSpeed = 1050, closeSpeed = 1000;

    private static double farDistance = 115, centerDistance = 82, defaultDistance = 47, closeDistance = 32;

    public static final PIDFCoefficients constants = new PIDFCoefficients(300, 13, 5, 0);

    public static double aimingTarget;
    public static double idleSpeed = 300;
    public boolean running = false;

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose targetPose) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                constants);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, "hood");

        this.telemetry = telemetry;
        this.follower = follower;
        this.targetPose = targetPose;
    }

    public double distanceToGoal() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        double dx = x - targetPose.getX();
        double dy = y - targetPose.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }


    public void update() {
        double distanceShoot = distanceToGoal();
        telemetry.addData("fFlywheel TPS", rightMotor.getVelocity());
        telemetry.addData("Distance to goal", distanceShoot);

        boolean changed = false;

        double roboty = follower.getPose().getY();
        running = roboty > 30;

        if (running) {
            double calculatedVelocity = 0, hoodPosition = 0;

            double [] distances = new double[] { closeDistance, defaultDistance, centerDistance, farDistance, farDistance };
            double [] speeds = new double[] { closeSpeed, defaultSpeed, centerSpeed, farSpeed, farSpeed };
            double [] hoods = new double[] { closeHood, defaultHood, centerHood, farHood, farHood };

            int index = -1;

            for (int i = 0; i < distances.length - 1; i++) {
                if (distances[i] <= distanceShoot && distances[i+1] >= distanceShoot) {
                    index = i;
                    break;
                }
            }


            if (index == -1) {
                if (distanceShoot > farDistance)
                    index = distances.length - 2;
                else
                    index = 0;
            }

            double extraDistance = distanceShoot - distances[index];
            double totalDelta = distances[index + 1] - distances[index];
            double coeff = totalDelta != 0 ? extraDistance / totalDelta : 0;
            coeff = Math.max(coeff, 0);
            coeff = Math.min(coeff, 1);

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
            rightMotor.setVelocity(aimingTarget);
//            leftMotor.setVelocity(0);
//            rightMotor.setVelocity(0);
        }
        //  flywheel1.setVelocity(targetVelocity);

        // Changed is reused here
        //changed = constants.d != kd ||
        //        constants.p != kp ||
        //        constants.i != ki ||
        //        constants.f != kf;

        //if (changed) {
        //    constants.d = kd;
        //    constants.p = kp;
        //    constants.i = ki;
        //    constants.f = kf;
        //    rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
        //            constants);
        //}
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
