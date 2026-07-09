package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Intake {
    public static double BARRIER_ON = 0.5;
    public static double BARRIER_OFF = 0.2;

    public static int TIME_TO_SHOOT = 400; //todo: tune

    private final DcMotor rightIntake;
    private final DcMotor leftIntake;
    private final Servo barrier;

    public static final double GATHER_POWER = 0.9;
    public static final double REJECT_POWER = -0.4;

    private final ElapsedTime timer;

    private final Command launchCommand;
    private final Command gatherCommand;
    private final Command rejectCommand;

    public Intake(HardwareMap hardwareMap) {
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        barrier = hardwareMap.get(Servo.class, "barrier");

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        timer = new ElapsedTime();

        stop();
        barrier.setPosition(BARRIER_ON);

        gatherCommand = Command.build()
                .setStart(this::start)
                .setDone(() -> leftIntake.getPower() == GATHER_POWER && rightIntake.getPower() == GATHER_POWER);

        rejectCommand = Command.build()
                .setStart(this::reject)
                .setDone(() -> leftIntake.getPower() == REJECT_POWER && rightIntake.getPower() == REJECT_POWER);

        launchCommand = Command.build()
                .setStart(() -> {
                    openBarrier();
                    timer.reset();
                })
                .setDone(() -> timer.milliseconds() >= TIME_TO_SHOOT)
                .setEnd((end) -> {
                    closeBarrier();
                });
    }


    private void start() {
        leftIntake.setPower(GATHER_POWER);
        rightIntake.setPower(GATHER_POWER);
    }

    private void stop() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    private void reject() {
        leftIntake.setPower(REJECT_POWER);
        rightIntake.setPower(REJECT_POWER);
    }

    private void openBarrier() {
        barrier.setPosition(BARRIER_OFF);
    }

    private void closeBarrier() {
        barrier.setPosition(BARRIER_ON);
    }

    public Command getLaunchCommand() {
        return launchCommand;
    }

    public Command getGatherCommand() {
        return gatherCommand;
    }

    public Command getRejectCommand() {
        return rejectCommand;
    }

}
