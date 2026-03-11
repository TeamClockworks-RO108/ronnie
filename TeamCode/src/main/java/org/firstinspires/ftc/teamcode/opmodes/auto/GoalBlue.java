package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.util.StateMachine;

@Autonomous(name = "Goal BLUE")
public class GoalBlue extends AutoBase {
    protected final StateMachine<State> fsm = new StateMachine<State>(State.START);
    protected enum State {
        INIT,
        START, SHOOT_PRELOAD,
        FIRST_INTAKE, FIRST_INTAKE_TO_SHOOT, FIRST_SHOOT,
        SECOND_INTAKE, SECOND_INTAKE_TO_SHOOT, SECOND_SHOOT,
        THIRD_INTAKE, THIRD_INTAKE_TO_SHOOT, THIRD_SHOOT,
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
        fsm.onStateEnter(State.START, () -> {
            movement.followPath(paths.GoalStartToShoot);
            intake.command(Intake.Command.TOGGLE_INTAKE);
        });
        fsm.onStateUpdate(State.START, (current, timeSinceTransition) ->
                !movement.isBusy() && timeSinceTransition > 2000 ? State.SHOOT_PRELOAD : null);
        fsm.onStateEnter(State.SHOOT_PRELOAD, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_PRELOAD, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.FIRST_INTAKE : null;
        });

        fsm.onStateEnter(State.FIRST_INTAKE, () -> movement.followPath(paths.goalToIntake3));
        fsm.onStateUpdate(State.FIRST_INTAKE, () -> !movement.isBusy() ? State.FIRST_INTAKE_TO_SHOOT : null);
        fsm.onStateEnter(State.FIRST_INTAKE_TO_SHOOT, () -> movement.followPath(paths.goShootDefault3));
        fsm.onStateUpdate(State.FIRST_INTAKE_TO_SHOOT, () -> !movement.isBusy() ? State.FIRST_SHOOT : null);
        fsm.onStateEnter(State.FIRST_SHOOT, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.FIRST_SHOOT, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.GO_HOME : null;
        });

        fsm.onStateEnter(State.GO_HOME, () -> movement.followPath(paths.GoalShootToHome));
        fsm.onStateUpdate(State.GO_HOME, () -> !movement.isBusy() ? State.DEAD : null);

        fsm.onStateEnter(State.DEAD, () -> intake.command(Intake.Command.TOGGLE_INTAKE));


        fsm.init();
    }

    @Override
    protected void startFSM() {
        fsm.onStateUpdate(State.INIT, () -> State.START);
    }

    @Override
    protected void updateFSM() {
        fsm.update();
    }
}
