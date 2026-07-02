package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "BOLTSANDSPEEEEED")
public class BoltsAndSpeed extends OpMode {

    // Hardware-map names for the two drive banks (4 motors per side).
    private static final String[] LEFT_MOTOR_NAMES = {"left0", "left1", "left2", "left3"};
    private static final String[] RIGHT_MOTOR_NAMES = {"right0", "right1", "right2", "right3"};

    // 8 dc motors in 2 arrays of 4. Read from the hardware map as DcMotor and cast to DcMotorEx
    // so we can query per-motor current.
    private final DcMotorEx[] leftMotors = new DcMotorEx[4];
    private final DcMotorEx[] rightMotors = new DcMotorEx[4];

    private Servo leftSteerServo;

    private Servo rightSteerServo;

    // Throttle magnitude below which the drive is treated as idle.
    private static final double THROTTLE_DEADZONE = 0.05;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        for (int i = 0; i < leftMotors.length; i++) {
            leftMotors[i] = (DcMotorEx) hardwareMap.get(DcMotor.class, LEFT_MOTOR_NAMES[i]);
            leftMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            leftMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightMotors[i] = (DcMotorEx) hardwareMap.get(DcMotor.class, RIGHT_MOTOR_NAMES[i]);
            rightMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftSteerServo = hardwareMap.get(Servo.class, "leftSteer");
        rightSteerServo = hardwareMap.get(Servo.class, "rightSteer");
    }

    @Override
    public void loop() {

        // calculation: make sum of all currents on DcMotors.
        double totalCurrent = 0; // in amps
        for (DcMotorEx motor : leftMotors)
            totalCurrent += motor.getCurrent(CurrentUnit.AMPS);
        for (DcMotorEx motor : rightMotors)
            totalCurrent += motor.getCurrent(CurrentUnit.AMPS);


        // find steer power
        double steerPower = 0;
        if (gamepad1.dpad_left)
            steerPower = -BNSConstants.dpadLeftSteerPower;
        if (gamepad1.dpad_right)
            steerPower = BNSConstants.dpadRightSteerPower;
        if (Math.abs(gamepad1.right_stick_x) > 0.05)
            steerPower = gamepad1.right_stick_x * BNSConstants.joystickSteerPower;

        // Per-side drive scaling for the Ackermann turn (inner side runs slower). 1 = no turn.
        double leftDriveScale = 1, rightDriveScale = 1;

        if (steerPower != 0) {
            double innerAckermann = Math.abs(steerPower) * BNSConstants.ackermannMobilityLimit;
            // Ackermann: cot(outer) = cot(inner) + W/L  ->  the outer wheel turns less than the inner.
            double outerAckermann = Math.atan(1.0 / (1.0 / Math.tan(innerAckermann) + BNSConstants.ackermannWidth / BNSConstants.ackermannLength));

            // Wheel-to-turn-center radius is proportional to 1/tan(steerAngle), so the inner side's
            // speed relative to the outer is tan(outer)/tan(inner) (< 1). Slow the inner side by that.
            if (BNSConstants.enableAckermannDifferentialPower) {
                double differential = Math.tan(outerAckermann) / Math.tan(innerAckermann);
                if (steerPower < 0) // going left -> left side is inner
                    leftDriveScale = differential;
                else                // going right -> right side is inner
                    rightDriveScale = differential;
            }


            double leftServoPos, rightServoPos;
            if (steerPower < 0) { // going left
                leftServoPos = BNSConstants.lerp(BNSConstants.leftServoCenter,
                        BNSConstants.leftServoSteerLeftLimit, innerAckermann / BNSConstants.ackermannMobilityLimit);
                rightServoPos = BNSConstants.lerp(BNSConstants.rightServoCenter,
                        BNSConstants.rightServoSteerLeftLimit, outerAckermann / BNSConstants.ackermannMobilityLimit);
            } else { // going right
                leftServoPos = BNSConstants.lerp(BNSConstants.leftServoCenter,
                        BNSConstants.leftServoSteerRightLimit, outerAckermann / BNSConstants.ackermannMobilityLimit);
                rightServoPos = BNSConstants.lerp(BNSConstants.rightServoCenter,
                        BNSConstants.rightServoSteerRightLimit, innerAckermann / BNSConstants.ackermannMobilityLimit);
            }

            leftSteerServo.setPosition(leftServoPos);
            rightSteerServo.setPosition(rightServoPos);

        } else {
            leftSteerServo.setPosition(BNSConstants.leftServoCenter);
            rightSteerServo.setPosition(BNSConstants.rightServoCenter);
        }


        // drive: forward/back on the left stick.
        double throttle = -gamepad1.left_stick_y;
        boolean throttleRequested = Math.abs(throttle) > THROTTLE_DEADZONE;

        double driveSpeed = 0;
        int activeMotorsPerSide = 0;
        double leftDrivePower = 0, rightDrivePower = 0;

        if (!throttleRequested) {
            // no throttle -> setPower(0) lets the BRAKE zero-power behaviour hold the robot
            for (DcMotorEx motor : leftMotors)
                motor.setPower(0);
            for (DcMotorEx motor : rightMotors)
                motor.setPower(0);
        } else {
            // active motors get the throttle (floored at motorInactivePower to beat static
            // friction), scaled per side for the Ackermann turn.
            double drivePower = Math.copySign(
                    Math.max(Math.abs(throttle), BNSConstants.motorInactivePower), throttle);
            leftDrivePower = drivePower * leftDriveScale;
            rightDrivePower = drivePower * rightDriveScale;
            double inactivePower = Math.copySign(BNSConstants.motorInactivePower, throttle);

            // tiered powerup: always 2 motors/side, +1 per crossed speed threshold (up to 4).
            driveSpeed = driveSpeed();
            activeMotorsPerSide = 2;
            if (driveSpeed > BNSConstants.powerupThreshold1) activeMotorsPerSide++;
            if (driveSpeed > BNSConstants.powerupThreshold2) activeMotorsPerSide++;

            applyTieredPower(leftMotors, leftDrivePower, inactivePower, activeMotorsPerSide);
            applyTieredPower(rightMotors, rightDrivePower, inactivePower, activeMotorsPerSide);
        }


        // telemetry:
        //
        telemetry.addData("Current", totalCurrent);
        telemetry.addData("Steer power", steerPower);
        telemetry.addData("Drive speed", driveSpeed);
        telemetry.addData("Active motors/side", activeMotorsPerSide);
        telemetry.addData("Drive power L/R", leftDrivePower + " / " + rightDrivePower);
        telemetry.update();


    }

    /**
     * Average valid wheel velocity (encoder ticks/sec). Readings outside
     * [encoderValidMin, encoderValidMax] are ignored; if none are valid the slowest of all
     * wheels is returned. Uses magnitudes so it works in both drive directions.
     */
    private double driveSpeed() {
        double validSum = 0, minAll = Double.MAX_VALUE;
        int validCount = 0;

        for (DcMotorEx[] bank : new DcMotorEx[][]{leftMotors, rightMotors}) {
            for (DcMotorEx motor : bank) {
                double v = Math.abs(motor.getVelocity());
                minAll = Math.min(minAll, v);
                if (v > BNSConstants.encoderValidMin && v < BNSConstants.encoderValidMax) {
                    validSum += v;
                    validCount++;
                }
            }
        }

        return validCount > 0 ? validSum / validCount : minAll;
    }

    /**
     * Drives the first {@code activeCount} motors of a bank at {@code activePower} and the rest at
     * {@code inactivePower}.
     */
    private void applyTieredPower(DcMotorEx[] motors, double activePower, double inactivePower, int activeCount) {
        for (int i = 0; i < motors.length; i++)
            motors[i].setPower(i < activeCount ? activePower : inactivePower);
    }
}
