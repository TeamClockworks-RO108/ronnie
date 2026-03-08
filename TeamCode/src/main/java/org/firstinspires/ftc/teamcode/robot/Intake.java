package org.firstinspires.ftc.teamcode.robot;

import android.hardware.input.InputManager;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.Teleop;
import org.firstinspires.ftc.teamcode.util.StateMachine;

public class Intake {
    private final DcMotor leftIntake;
    private final DcMotor rightIntake;

    private double TIME_TO_SHOOT = 1000, TIME_TO_START_FLYWHEEL = 1000;

    private Flywheel flywheel;

    private Turret turret;

    private enum State {
        IDLE,
        INTAKE,

        PREPARE_FOR_LAUNCH,
        LAUNCHING,
    }

    public enum Command{
        TOGGLE_INTAKE,
        LAUNCH,
    }

    public void command(Command command){
        unexecutedCommand = command;
    }

    private Command unexecutedCommand = null;

    private final StateMachine<State> fsm = new StateMachine<>(State.IDLE);
    private boolean isOn = false;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel = new Flywheel(hardwareMap, telemetry);
        turret = new Turret(hardwareMap);
    }


    private void start() {
        isOn = true;
        leftIntake.setPower(1);
        rightIntake.setPower(1);
    }
    private void stop() {
        isOn = false;
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }


    public void setupFSM(){
        fsm.init();

        fsm.onStateEnter(State.IDLE,   () -> {
            stop();
            turret.lowerBarrier();
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
               return  State.PREPARE_FOR_LAUNCH;
           }

           if(unexecutedCommand == Command.TOGGLE_INTAKE){
               unexecutedCommand = null;
               return State.IDLE;
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
            turret.liftBarrier();

        });

        fsm.onStateUpdate(State.LAUNCHING,   (current, timeSinceTransition) -> {
            if (timeSinceTransition > TIME_TO_SHOOT){
                return State.INTAKE;
            }
            return null;
        });

        fsm.onStateExit(State.LAUNCHING, () -> {
            turret.lowerBarrier();
            flywheel.idle();
        });




    }

    public void updateFSM(){
        fsm.update();
        flywheel.update();
    }
}
