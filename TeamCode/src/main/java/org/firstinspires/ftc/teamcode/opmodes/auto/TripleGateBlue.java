package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.util.StateMachine;

@Autonomous (name = "Golden Gate Labubu BLUE")
public class TripleGateBlue extends AutoBase{

    protected StateMachine<State> fsm = new StateMachine<>(State.INIT);

    private double waitForTurret = 2000;

    protected enum  State {
        INIT,
        START_TO_SHOOT, SHOOT_PRELOAD,
        INTAKE_B, INTAKE_TO_SHOOT_B, OPEN_GATE_AFTER_B,  SHOOT_B,
//        GATE_INTAKE_A, GATE_INTAKE_TO_SHOOT_A, GATE_INTAKE_TO_SHOOT_A_GO, GATE_SHOOT_A,
//        GATE_INTAKE_B, GATE_INTAKE_TO_SHOOT_B, GATE_INTAKE_TO_SHOOT_B_GO,GATE_SHOOT_B,
//        GATE_INTAKE_C, GATE_INTAKE_TO_SHOOT_C,GATE_INTAKE_TO_SHOOT_C_GO, GATE_SHOOT_C,
        INTAKE_A, INTAKE_TO_SHOOT_A, SHOOT_A,

        GO_COLLECT_FROM_HUMAN_1, RETURN_TO_LAUNCH_1, LAUNCH_1,
        GO_COLLECT_FROM_HUMAN_2, RETURN_TO_LAUNCH_2, LAUNCH_2,


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
                !movement.isBusy() && timeSinceTransition > 400? State.SHOOT_PRELOAD : null);
        fsm.onStateEnter(State.SHOOT_PRELOAD, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_PRELOAD, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.INTAKE_A : null;
        });


        // cycle B
        fsm.onStateEnter(State.INTAKE_B, () -> { movement.followPath(paths.goalToIntake2);
            intake.overrideTarget(1240);});
        fsm.onStateUpdate(State.INTAKE_B, (current, timeSinceTransition) -> !movement.isBusy()? State.OPEN_GATE_AFTER_B : null);
        fsm.onStateEnter(State.OPEN_GATE_AFTER_B, () -> movement.followPath(paths.turnToOpenGate));
        fsm.onStateUpdate(State.OPEN_GATE_AFTER_B, (current, timeSinceTransition) -> !movement.isBusy()? State.INTAKE_TO_SHOOT_B : null);
        fsm.onStateEnter(State.INTAKE_TO_SHOOT_B, () -> movement.followPath(paths.intake2ToShoot));
        fsm.onStateUpdate(State.INTAKE_TO_SHOOT_B, (current, timeSinceTransition) -> !movement.isBusy() && timeSinceTransition > waitForTurret ?  State.SHOOT_B : null);
        fsm.onStateEnter(State.SHOOT_B, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_B, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.INTAKE_C: null;
        });
        //TUNED FOR DOUBLE GATE !!1!!!!!

        //cycle A
        fsm.onStateEnter(State.INTAKE_A, () -> { movement.followPath(paths.goalToIntake3);
            intake.overrideTarget(1325);} );
        fsm.onStateUpdate(State.INTAKE_A, () -> !movement.isBusy() ? State.INTAKE_TO_SHOOT_A : null);
        fsm.onStateEnter(State.INTAKE_TO_SHOOT_A, () -> movement.followPath(paths.intake3ToShoot));
        fsm.onStateUpdate(State.INTAKE_TO_SHOOT_A, (current, timeSinceTransition )  -> !movement.isBusy() && timeSinceTransition > waitForTurret? State.SHOOT_A  : null);
        fsm.onStateEnter(State.SHOOT_A, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_A, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.INTAKE_B : null;
        });

        //cycle C
        fsm.onStateEnter(State.INTAKE_C, () -> movement.followPath(paths.goTo3rdSpike));
        fsm.onStateUpdate(State.INTAKE_C, () -> !movement.isBusy() ? State.INTAKE_TO_SHOOT_C : null);
        fsm.onStateEnter(State.INTAKE_TO_SHOOT_C, () -> { movement.followPath(paths.intake1ToShoot);
        intake.command(Intake.Command.REJECT);});
        fsm.onStateUpdate(State.INTAKE_TO_SHOOT_C, (current, timeSinceTransition) -> !movement.isBusy()  && timeSinceTransition > waitForTurret ? State.SHOOT_C : null);
        fsm.onStateEnter(State.SHOOT_C, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_C, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.GO_COLLECT_FROM_HUMAN_1 : null;
        });

        // collectFromHuman
        fsm.onStateEnter(State.GO_COLLECT_FROM_HUMAN_1, () -> {movement.followPath(paths.goCollectFromHumanEvo);});
        fsm.onStateUpdate(State.GO_COLLECT_FROM_HUMAN_1, (current, timeSinceTransition) -> !movement.isBusy() ? State.RETURN_TO_LAUNCH_1: null);
        fsm.onStateEnter(State.RETURN_TO_LAUNCH_1,() -> {
            movement.followPath(paths.returnFromHuman);
            intake.command(Intake.Command.REJECT);} );
        fsm.onStateUpdate (State.RETURN_TO_LAUNCH_1,  (current, timeSinceTransition) ->  !movement.isBusy() && timeSinceTransition > waitForTurret ?  State.LAUNCH_1 : null );
        fsm.onStateEnter(State.LAUNCH_1,   () -> { intake.command(Intake.Command.LAUNCH);});
        fsm.onStateUpdate(State.LAUNCH_1, (current, timeSinceTransition) -> { return  timeSinceTransition > intake.getShootTime() ? State.GO_COLLECT_FROM_HUMAN_2: null; } );

        //collectFromHuman2
        fsm.onStateEnter(State.GO_COLLECT_FROM_HUMAN_2, () -> {movement.followPath(paths.goCollectFromHumanEvo2);});
        fsm.onStateUpdate(State.GO_COLLECT_FROM_HUMAN_2, (current, timeSinceTransition) -> !movement.isBusy() ? State.RETURN_TO_LAUNCH_2: null);
        fsm.onStateEnter(State.RETURN_TO_LAUNCH_2,() -> {
            movement.followPath(paths.returnFromHuman2);
            intake.command(Intake.Command.REJECT);
        } );
        fsm.onStateUpdate (State.RETURN_TO_LAUNCH_2,  (current, timeSinceTransition) ->  !movement.isBusy() && timeSinceTransition > waitForTurret ?  State.LAUNCH_2 : null );
        fsm.onStateEnter(State.LAUNCH_2,   () -> { intake.command(Intake.Command.LAUNCH);});
        fsm.onStateUpdate(State.LAUNCH_2, (current, timeSinceTransition) -> { return  timeSinceTransition > intake.getShootTime() ? State.DEAD: null; } );

        
//        fsm.onStateEnter(State.GO_HOME, () -> movement.followPath(paths.GoalShootToHome));
//        fsm.onStateUpdate(State.GO_HOME, () -> !movement.isBusy() ? State.DEAD : null);

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

    private void setupGateCycle(State INTAKE, State INTAKE_TO_SHOOT, State SHOOT, State NEXT_STATE) {
        fsm.onStateEnter(INTAKE, () -> movement.followPath(paths.shootTogate));
        fsm.onStateUpdate(INTAKE, (current, timeSinceTransition) -> !movement.isBusy()  && timeSinceTransition > 1800 ? INTAKE_TO_SHOOT : null);
        fsm.onStateEnter(INTAKE_TO_SHOOT, () -> {
            movement.followPath(paths.gateToShoot);
            intake.command(Intake.Command.REJECT);
        });
        fsm.onStateUpdate(INTAKE_TO_SHOOT, (current, timeSinceTransition) -> !movement.isBusy() || timeSinceTransition > 0 ? SHOOT : null);
        fsm.onStateEnter(SHOOT, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(SHOOT, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? NEXT_STATE : null;
        });
    }
}
