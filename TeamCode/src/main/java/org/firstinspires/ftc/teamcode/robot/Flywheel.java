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

    private final DcMotorEx leftMotor;
    private final Telemetry telemetry;

    private final Follower follower;
    private final Servo hoodServo;

    private final Pose targetPose;

    private static double farHood = 0.72, centerHood = 0.55, defaultHood = 0.48, closeHood = 0.28;
    private static double farSpeed = 1200, centerSpeed = 1300, defaultSpeed = 1175, closeSpeed = 1125;

    private static double farDistance = 115, centerDistance = 104, defaultDistance = 75, closeDistance = 58;

    public static double TURRET_TO_ODOM = -1.0881954;

    // DO NOT TOUCH THIS UNLESS NECESSARY
    public static double CORR_OFFSET_ANGLE = 0;



    public static final PIDFCoefficients constants = new PIDFCoefficients();

    public static double kp = 250, ki = 3, kd = 8, kf = 0;

    public static double aimingTarget;
    public static double idleSpeed = 300;
    public boolean running = false;

    private long overrideTarget = -1;

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose targetPose) {

        constants.d = kd;
        constants.p = kp;
        constants.i = ki;
        constants.f = kf;

        rightMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor = hardwareMap.get(DcMotorEx.class, "flywheel1");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        hoodServo = hardwareMap.get(Servo.class, "hood");

        this.telemetry = telemetry;
        this.follower = follower;
        this.targetPose = targetPose;
    }

    public void update() {
        double distanceShoot = distanceToGoal();
        telemetry.addData("Flywheel 1 TPS", rightMotor.getVelocity());
        telemetry.addData("Flywheel 2 TPS", leftMotor.getVelocity());
        telemetry.addData("Distance to goal", distanceShoot);

        boolean changed = false;

        double roboty = follower.getPose().getY();
        running = roboty > 0;

        if (running) {
            double calculatedVelocity = 0, hoodPosition = 0;

            double [] distances = new double[] { closeDistance, defaultDistance, centerDistance, farDistance, farDistance + 999};
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

            telemetry.addData("Flywheel Index:", index);
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

        if (changed && overrideTarget == -1) {
            rightMotor.setVelocity(aimingTarget);
            leftMotor.setVelocity(aimingTarget);
        }
        telemetry.addData("Flywheel Target TPS", -aimingTarget);

        if (overrideTarget != -1) {
            rightMotor.setVelocity(overrideTarget);
            leftMotor.setVelocity(overrideTarget);
        }

        changed = constants.d != kd ||
                constants.p != kp ||
                constants.i != ki ||
                constants.f != kf;

        if (changed) {
            constants.d = kd;
            constants.p = kp;
            constants.i = ki;
            constants.f = kf;
            rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    constants);
            leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    constants);
        }
    }

    public double distanceToGoal() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        double turretAngle = follower.getPose().getHeading() + Math.toRadians(CORR_OFFSET_ANGLE);
        double px = Math.cos(turretAngle) * TURRET_TO_ODOM;
        double py = Math.sin(turretAngle) * TURRET_TO_ODOM;

        x += px;
        y += py;

        double dx = x - targetPose.getX();
        double dy = y - targetPose.getY();

        return Math.sqrt(dx * dx + dy * dy);
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

    // Call with -1 to disable
    public void overrideTarget(long speed)  {
        this.overrideTarget = speed;
    }
}
