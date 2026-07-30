package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Intake {
    public static double BARRIER_ON = 0.5;
    public static double BARRIER_OFF = 0.2;

    public static int TIME_TO_SHOOT = 700;

    private final DcMotor rightIntake;
    private final DcMotor leftIntake;
    private final Servo barrier;

    public static final double GATHER_POWER = 0.9;
    public static final double REJECT_POWER = -0.4;

    private final ElapsedTime timer;

    private final Command togglePowerCommand;
    private final Command toggleReverseCommand;
    private final Command stopCommand;
    private final Command launchNoCloseCommand;

    private final Command manualOpenBarrierCommand;
    private final Command manualCloseBarrierCommand;
    private final Command startCommand;

    private boolean reversed;

    private boolean running;
    private Telemetry telemetry;


    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        barrier = hardwareMap.get(Servo.class, "barrier");

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        timer = new ElapsedTime();

        running = false;
        reversed = false;

        barrier.setPosition(BARRIER_ON);

        this.telemetry = telemetry;

        togglePowerCommand = Command.build()
                .setStart(() -> running = !running)
                .setDone(() -> true)
                .setEnd((__) -> updatePower());

        startCommand = Command.build()
                .setStart(() -> {
                    running = true;
                    reversed = false;
                    updatePower();
                })
                .setDone(() -> true);

        toggleReverseCommand = Command.build()
                .setStart(() -> reversed = !reversed)
                .setDone(() -> true)
                .setEnd((__) -> updatePower());

        launchNoCloseCommand = Command.build()
                .setStart(() -> {
                    running = true;
                    reversed = false;
                    updatePower();

                    openBarrier();
                    timer.reset();
                })
                .setDone(() -> timer.milliseconds() >= TIME_TO_SHOOT)
                .setEnd((__) -> {
                    closeBarrier();
                });

        stopCommand = Command.build()
                .setStart(() -> running = false)
                .setDone(() -> true)
                .setEnd((__) -> {
                    updatePower();
                });
        manualOpenBarrierCommand = Command.build()
                .setStart(() -> {
                    running = true;
                    reversed = false;
                    updatePower();
                    openBarrier();
                })
                .setDone(() -> true);

        manualCloseBarrierCommand = Command.build()
                .setStart(() -> {
                    running = false;
                    reversed = false;
                    updatePower();
                    closeBarrier();
                })
                .setDone(() -> true);
    }

    private void updatePower() {
        if(!running) {
            stop();
        } else {
            if (!reversed) start();
            else reject();
        }
    }

    public void update() {
        telemetry.addData("Running", running);
        telemetry.addData("Reverse", reversed);
    }

    public void manual(double pow) {
        leftIntake.setPower(pow);
        rightIntake.setPower(pow);
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

    public Command getTogglePowerCommand() {
        return togglePowerCommand;
    }
    public Command getToggleDirectionCommand() {
        return toggleReverseCommand;
    }

    public Command getStopCommand() {
        return stopCommand;
    }

    public Command getLaunchNoCloseCommand() {
        return launchNoCloseCommand;
    }

    public Command getManualOpenBarrierCommand() {
        return manualOpenBarrierCommand;
    }

    public Command getManualCloseBarrierCommand() {
        return manualCloseBarrierCommand;
    }

    public Command getStartCommand() {
        return startCommand;
    }
}
