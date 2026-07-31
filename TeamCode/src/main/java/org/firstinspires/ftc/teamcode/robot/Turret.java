package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.field.poses.Poses;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Configurable
public class Turret {
    public static double TICKS_PER_180 = 13712;//23 77
    private final Telemetry telemetry;
    private final Follower follower;
    private final PedroMovement movement;
    private final PIDFController pid;

    private final DcMotor encoderMotor;
    private final CRServo headingServo0, headingServo1;

//    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.75, 0.0, 0.0755 , 0);
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.9, 0.0, 0.038 , 0);

    private TeamColor color;
    private Poses poses;

    public static double OFFSET = 0;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, PedroMovement movement, Poses poses,
                  boolean resetEncoders, TeamColor color) {
        this.color = color;

        encoderMotor = hardwareMap.get(DcMotor.class, "leftFront");

        headingServo0 = hardwareMap.get(CRServo.class, "heading0");
        headingServo0.setDirection(CRServo.Direction.REVERSE);

        headingServo1 = hardwareMap.get(CRServo.class, "heading1");
        headingServo1.setDirection(CRServo.Direction.REVERSE);

        this.telemetry = telemetry;
        this.follower = movement.getFollower();
        this.movement = movement;

        this.poses = poses;

        pid = new PIDFController(pidfCoefficients);

        if (resetEncoders) {
            encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        Pose curr = follower.getPose();
        double angleError = getAngleError(movement.getFollower());

        pid.updateError(angleError);
        double power = pid.run();

        setTurretPower(power);

        double distanceToGoal = curr.distanceFrom(poses.goal());

        telemetry.addData("Turret pow", power);
        telemetry.addData("Turret Encoder", getEncoder());
        telemetry.addData("Distance to goal", distanceToGoal);
    }

    private double getAngleError(Follower follower) {
        double angle = Math.toRadians(getEncoder() * 180 / TICKS_PER_180);

        double heading = follower.getHeading();
        double x = poses.goal().getX() - follower.getPose().getX();
        double y = poses.goal().getY() - follower.getPose().getY();

        telemetry.addData("Offset", OFFSET);

        double globalAngle = Math.atan2(y, x);
        double turretAngle = globalAngle - heading + OFFSET;

        double sine = Math.sin(turretAngle);
        double cosine = Math.cos(turretAngle);

        double turretWrapped = Math.atan2(sine, cosine);

        telemetry.addData("Global angle", Math.toDegrees(globalAngle));
        telemetry.addData("Heading angle", Math.toDegrees(heading));
        telemetry.addData("Turret angle unwrapped", Math.toDegrees(turretAngle));
        telemetry.addData("Turret target angle wrapped", Math.toDegrees(turretWrapped));
        telemetry.addData("Current angle", Math.toDegrees(angle));

        return MathUtil.clamp(turretWrapped - angle, -Math.PI, Math.PI * 0.67);
    }

    private long getEncoder() {
        return encoderMotor.getCurrentPosition();
    }

    private void setTurretPower(double power) {
        headingServo0.setPower(power);
        headingServo1.setPower(power);
    }

    public void offset(int multiplier) {
        OFFSET += Math.toRadians(2.5 * multiplier);
    }
}