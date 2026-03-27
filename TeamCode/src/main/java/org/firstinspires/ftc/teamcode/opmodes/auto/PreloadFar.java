package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.util.StateMachine;

@Autonomous(name = "Far auto")
public class PreloadFar extends AutoBase{

    protected enum State{
        PRE_INIT,

        INIT,
        START_TO_SHOOT,
        SHOOT_PRELOAD,
        LEAVE,
        DEAD

    }

    protected StateMachine<State> fsm = new StateMachine<>(State.PRE_INIT);


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

        fsm.onStateUpdate(State.PRE_INIT, (current, timeSinceTransition) -> {
            if (timeSinceTransition > 500) {
                return State.INIT;
            }
            return null;
        });
        fsm.onStateEnter(State.START_TO_SHOOT, () -> {
            intake.command(Intake.Command.TOGGLE_INTAKE);
        });
        fsm.onStateUpdate(State.START_TO_SHOOT, (current, timeSinceTransition) ->
                !movement.isBusy() && timeSinceTransition > 5000?  State.SHOOT_PRELOAD : null);
        fsm.onStateEnter(State.SHOOT_PRELOAD, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_PRELOAD, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.LEAVE : null;
        });

        fsm.onStateEnter(State.LEAVE, () -> {movement.followPath(paths.leaveFar);});
        fsm.onStateUpdate(State.LEAVE,  (current, timeSinceTransition) -> { if( timeSinceTransition > 5000) {return State.DEAD; }  return null; });

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
