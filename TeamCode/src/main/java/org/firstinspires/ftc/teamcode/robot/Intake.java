package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.StateMachine;

@Configurable
public class Intake {
    private static final double BARRIER_ON = 0.475, BARRIER_OFF = 0.7;

    private static final double BARRIER_VIBRATE_AMPLITUDE = 0.004;
    private static final double BARRIER_VIBRATE_TIME = 400;

    private double lastVibrate;

    private boolean isHoodRaised = false;
    private int TIME_TO_SHOOT = 1000, TIME_TO_START_FLYWHEEL = 1500, TIME_TO_REJECT = 100;

    private final DcMotor rightIntake;
    private final DcMotor leftIntake;
    private final Servo barrier;
    private final Flywheel flywheel;


    private enum State {
        IDLE,
        INTAKE,
        PREPARE_FOR_LAUNCH,
        LAUNCHING,
        REJECTING,
    }

    public enum Command{
        TOGGLE_INTAKE,
        LAUNCH,
        REJECT,
    }

    public void command(Command command){
        unexecutedCommand = command;
    }

    private Command unexecutedCommand = null;

    private final StateMachine<State> fsm = new StateMachine<>(State.IDLE);

    public Intake(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose targetPose) {
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        barrier = hardwareMap.get(Servo.class, "barrier");
        flywheel = new Flywheel(hardwareMap, telemetry, follower, targetPose);

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        setupFSM();
    }


    private void start() {
        leftIntake.setPower(1);
        rightIntake.setPower(1);
    }
    private void stop() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }
    private void reject() {
        leftIntake.setPower(-0.4);
        rightIntake.setPower(-0.4);
    }

    public void setupFSM(){

        fsm.onStateEnter(State.IDLE,   () -> {
            stop();
            barrier.setPosition(BARRIER_ON);
            flywheel.idle();
        });
        fsm.onStateUpdate(State.IDLE, () -> {
            if(unexecutedCommand == Command.TOGGLE_INTAKE){
                unexecutedCommand = null;
                return State.INTAKE;
            }
            if(unexecutedCommand == Command.LAUNCH){
                unexecutedCommand = null;
                return State.PREPARE_FOR_LAUNCH;
            }
            return null;
        });

        fsm.onStateEnter(State.INTAKE, () -> {
            start();
        });
        fsm.onStateUpdate(State.INTAKE, () -> {
           if(unexecutedCommand == Command.LAUNCH){
               unexecutedCommand = null;
               return  State.LAUNCHING;
           }
           if(unexecutedCommand == Command.TOGGLE_INTAKE){
               unexecutedCommand = null;
               return State.IDLE;
           }
           if (unexecutedCommand == Command.REJECT) {
               unexecutedCommand = null;
               return State.REJECTING;
           }
           return null;
        });

        fsm.onStateEnter(State.PREPARE_FOR_LAUNCH, () -> {
            flywheel.start();
        });
        fsm.onStateUpdate(State.PREPARE_FOR_LAUNCH,   (current, timeSinceTransition) -> {
            if(timeSinceTransition > TIME_TO_START_FLYWHEEL){
                return State.LAUNCHING;
            }
            return null;
        });

        fsm.onStateEnter(State.LAUNCHING, () -> {
            start();
            barrier.setPosition(BARRIER_OFF);
        });
        fsm.onStateUpdate(State.LAUNCHING,   (current, timeSinceTransition) -> {
            if (timeSinceTransition > TIME_TO_SHOOT){
                return State.INTAKE;
            }
            return null;
        });
        fsm.onStateExit(State.LAUNCHING, () -> {
            barrier.setPosition(BARRIER_ON);
            flywheel.idle();
        });

        fsm.onStateEnter(State.REJECTING, () -> reject());
        fsm.onStateUpdate(State.REJECTING, (current, timeSinceTransition) -> timeSinceTransition > 200 ? State.INTAKE : null);

        fsm.init();


    }

    public void update(){
        fsm.update();
        flywheel.update();

        double time = System.currentTimeMillis();

        double barpos = barrier.getPosition();
        if (Math.abs(barpos - BARRIER_OFF) < (BARRIER_VIBRATE_AMPLITUDE + 0.001) && time - lastVibrate > BARRIER_VIBRATE_TIME) {
            lastVibrate = time;
            if (barpos < BARRIER_OFF)
                barrier.setPosition(BARRIER_OFF + BARRIER_VIBRATE_AMPLITUDE);
            else
                barrier.setPosition(BARRIER_OFF - BARRIER_VIBRATE_AMPLITUDE);
        }
    }

    public long getShootTime() {
        return TIME_TO_SHOOT + TIME_TO_START_FLYWHEEL;
    }

}
