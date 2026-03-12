package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.util.StateMachine;

@Autonomous(name = "Goal BLUE")
public class GoalBlue extends AutoBase {
    protected final StateMachine<State> fsm = new StateMachine<State>(State.START_TO_SHOOT);
    protected enum State {
        INIT,
        START_TO_SHOOT, SHOOT_PRELOAD,
        INTAKE_A, INTAKE_TO_SHOOT_A, SHOOT_A,
        INTAKE_B, INTAKE_TO_SHOOT_B, SHOOT_B,
        INTAKE_C, INTAKE_TO_SHOOT_C, SHOOT_C,
        GO_HOME,
        DEAD
    }

    @Override
    protected void setColor() {
        color = TeamColor.BLUE;
    }

    @Override
    protected void setStartingPose() {
        startingPose = poses.goalStart;
    }

    @Override
    protected void setupFSM() {
        // preload cycle
        fsm.onStateEnter(State.START_TO_SHOOT, () -> {
            movement.followPath(paths.GoalStartToShoot);
            intake.command(Intake.Command.TOGGLE_INTAKE);
        });
        fsm.onStateUpdate(State.START_TO_SHOOT, (current, timeSinceTransition) ->
                !movement.isBusy() && timeSinceTransition > 2500 ? State.SHOOT_PRELOAD : null);
        fsm.onStateEnter(State.SHOOT_PRELOAD, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_PRELOAD, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.INTAKE_A : null;
        });

        // cycle A
        fsm.onStateEnter(State.INTAKE_A, () -> movement.followPath(paths.goalToIntake3));
        fsm.onStateUpdate(State.INTAKE_A, () -> !movement.isBusy() ? State.INTAKE_TO_SHOOT_A : null);
        fsm.onStateEnter(State.INTAKE_TO_SHOOT_A, () -> movement.followPath(paths.intake3ToShoot));
        fsm.onStateUpdate(State.INTAKE_TO_SHOOT_A, () -> !movement.isBusy() ? State.SHOOT_A : null);
        fsm.onStateEnter(State.SHOOT_A, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_A, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.INTAKE_B : null;
        });

        // cycle B
        fsm.onStateEnter(State.INTAKE_B, () -> movement.followPath(paths.goalToIntake2));
        fsm.onStateUpdate(State.INTAKE_B, () -> !movement.isBusy() ? State.INTAKE_TO_SHOOT_B : null);
        fsm.onStateEnter(State.INTAKE_TO_SHOOT_B, () -> movement.followPath(paths.intake2ToShoot));
        fsm.onStateUpdate(State.INTAKE_TO_SHOOT_B, () -> !movement.isBusy() ? State.SHOOT_B : null);
        fsm.onStateEnter(State.SHOOT_B, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_B, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.INTAKE_C : null;
        });

        //cycle C
        fsm.onStateEnter(State.INTAKE_C, () -> movement.followPath(paths.goalToIntake1));
        fsm.onStateUpdate(State.INTAKE_C, () -> !movement.isBusy() ? State.INTAKE_TO_SHOOT_C : null);
        fsm.onStateEnter(State.INTAKE_TO_SHOOT_C, () -> movement.followPath(paths.intake1ToShoot));
        fsm.onStateUpdate(State.INTAKE_TO_SHOOT_C, () -> !movement.isBusy() ? State.SHOOT_C : null);
        fsm.onStateEnter(State.SHOOT_C, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_C, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.GO_HOME : null;
        });

        fsm.onStateEnter(State.GO_HOME, () -> movement.followPath(paths.GoalShootToHome));
        fsm.onStateUpdate(State.GO_HOME, () -> !movement.isBusy() ? State.DEAD : null);

        fsm.onStateEnter(State.DEAD, () -> intake.command(Intake.Command.TOGGLE_INTAKE));


        fsm.init();
    }

    @Override
    protected void startFSM() {
        fsm.onStateUpdate(State.INIT, () -> State.START_TO_SHOOT);
    }

    @Override
    protected void updateFSM() {
        fsm.update();
    }
}
