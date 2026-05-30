package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.util.StateMachine;

@Autonomous(name = "Gate BLUE")
public class HeavyGaterBlue extends AutoBase {
    private static final int GATE_INTAKE_WAIT = 700;

    private static final int failsafe = 2800;

    protected StateMachine<State> fsm = new StateMachine<>(State.INIT);
    private double waitForTurret = 0;

    protected enum State {
        INIT,
        START_TO_SHOOT, SHOOT_PRELOAD,
        INTAKE_SPIKE_1, INTAKE_TO_SHOOT_SPIKE_1,WAIT_FOR_SPIKE1, SHOOT_SPIKE_1,
        INTAKE_SPIKE_2, INTAKE_TO_SHOOT_SPIKE_2, OPEN_GATE_AFTER_SPIKE_2, WAIT_FOR_SPIKE2,  SHOOT_SPIKE_2, SHOOT_SPIKE_2_DELAY,
        OPEN_GATEPASS_1, INTAKE_GATEPASS_1, INTAKE_TO_SHOOT_GATEPASS_1, SHOOT_GATEPASS_1,
        OPEN_GATEPASS_2, INTAKE_GATEPASS_2, INTAKE_TO_SHOOT_GATEPASS_2, SHOOT_GATEPASS_2,
        OPEN_GATEPASS_3, INTAKE_GATEPASS_3, INTAKE_TO_SHOOT_GATEPASS_3, SHOOT_GATEPASS_3,
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
            turret.setOverridePose(paths.GoalStartToShoot.endPose());
            movement.followPath(paths.GoalStartToShoot);
            intake.command(Intake.Command.TOGGLE_INTAKE);


        });
        fsm.onStateUpdate(State.START_TO_SHOOT, (current, timeSinceTransition) ->
                !movement.isBusy() && timeSinceTransition > 400? State.SHOOT_PRELOAD : null);
        fsm.onStateEnter(State.SHOOT_PRELOAD, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_PRELOAD, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.INTAKE_SPIKE_1 : null;
        });

        // spike 1
        fsm.onStateEnter(State.INTAKE_SPIKE_1, () -> { movement.followPath(paths.goalToIntake3);
        }
        );
        fsm.onStateUpdate(State.INTAKE_SPIKE_1, () -> !movement.isBusy() ? State.INTAKE_TO_SHOOT_SPIKE_1 : null);
        fsm.onStateEnter(State.INTAKE_TO_SHOOT_SPIKE_1, () -> {movement.followPath(paths.intake3ToShoot);  turret.setOverridePose(paths.intake3ToShoot.endPose());});
        fsm.onStateUpdate(State.INTAKE_TO_SHOOT_SPIKE_1, (current, timeSinceTransition )  -> !movement.isBusy() && timeSinceTransition > waitForTurret? State.WAIT_FOR_SPIKE1 : null);
        fsm.onStateUpdate(State.WAIT_FOR_SPIKE1, (current, timeSinceTransition) -> {
            if(timeSinceTransition > 300){
                return State.SHOOT_SPIKE_1;
            }
            return null;
        });
        fsm.onStateEnter(State.SHOOT_SPIKE_1, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_SPIKE_1, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.INTAKE_SPIKE_2 : null;
        });

        // spike 2 + gate
        fsm.onStateEnter(State.INTAKE_SPIKE_2, () -> {
            movement.followPath(paths.goalToIntake2);
            //todo: alalalla
            turret.setOverridePose(paths.intake2ToShoot.endPose());


        });
        fsm.onStateUpdate(State.INTAKE_SPIKE_2, () -> !movement.isBusy()? State.OPEN_GATE_AFTER_SPIKE_2 : null);
        fsm.onStateEnter(State.OPEN_GATE_AFTER_SPIKE_2, () -> movement.followPath(paths.turnToOpenGate));
        fsm.onStateUpdate(State.OPEN_GATE_AFTER_SPIKE_2, (s, t) -> !movement.isBusy() || t > 1500 ? State.INTAKE_TO_SHOOT_SPIKE_2 : null);
        fsm.onStateEnter(State.INTAKE_TO_SHOOT_SPIKE_2, () -> movement.followPath(paths.intake2ToShoot));
        fsm.onStateUpdate(State.INTAKE_TO_SHOOT_SPIKE_2, (current, timeSinceTransition) -> !movement.isBusy() && timeSinceTransition > waitForTurret ?  State.SHOOT_SPIKE_2_DELAY : null);
        fsm.onStateUpdate(State.SHOOT_SPIKE_2_DELAY, (current, timeSinceTransition) -> timeSinceTransition > 350 ? State.SHOOT_SPIKE_2 : null);
        fsm.onStateEnter(State.SHOOT_SPIKE_2, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_SPIKE_2, (current, timeSinceTransition) -> timeSinceTransition > intake.getShootTime() ? State.OPEN_GATEPASS_1: null);

        // 1st gate pass
        fsm.onStateEnter(State.OPEN_GATEPASS_1, () -> {
            movement.followPath(paths.shootTogate);
            turret.setOverridePose(paths.gateToShoot.endPose());
        });
        fsm.onStateUpdate(State.OPEN_GATEPASS_1, ( current, timeSinceTransition ) -> {
            if (!movement.isBusy() || (timeSinceTransition > failsafe)) {
                return State.INTAKE_GATEPASS_1;
            }
            return null;
        }) ;
        fsm.onStateUpdate(State.INTAKE_GATEPASS_1, (current, timeSinceTransition) -> timeSinceTransition > GATE_INTAKE_WAIT ? State.INTAKE_TO_SHOOT_GATEPASS_1 : null);
        fsm.onStateEnter(State.INTAKE_TO_SHOOT_GATEPASS_1, () -> movement.followPath(paths.gateToShoot));
        fsm.onStateUpdate(State.INTAKE_TO_SHOOT_GATEPASS_1, () -> !movement.isBusy() ? State.SHOOT_GATEPASS_1 : null);
        fsm.onStateEnter(State.SHOOT_GATEPASS_1, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_GATEPASS_1, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.OPEN_GATEPASS_2 : null;
        });

        // 2nd gate pass
        fsm.onStateEnter(State.OPEN_GATEPASS_2, () -> {
            movement.followPath(paths.shootTogate);
        });
        fsm.onStateUpdate(State.OPEN_GATEPASS_2, ( current, timeSinceTransition ) -> {
            if (!movement.isBusy() || (timeSinceTransition > failsafe)) {
                return State.INTAKE_GATEPASS_2;
            }
            return null;
        }) ;
        fsm.onStateUpdate(State.INTAKE_GATEPASS_2, (current, timeSinceTransition) -> timeSinceTransition > GATE_INTAKE_WAIT ? State.INTAKE_TO_SHOOT_GATEPASS_2 : null);
        fsm.onStateEnter(State.INTAKE_TO_SHOOT_GATEPASS_2, () -> movement.followPath(paths.gateToShoot));
        fsm.onStateUpdate(State.INTAKE_TO_SHOOT_GATEPASS_2, () -> !movement.isBusy() ? State.SHOOT_GATEPASS_2 : null);
        fsm.onStateEnter(State.SHOOT_GATEPASS_2, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_GATEPASS_2, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.OPEN_GATEPASS_3 : null;
        });

        // 3rd gate pass + leave
        fsm.onStateEnter(State.OPEN_GATEPASS_3, () -> {
            movement.followPath(paths.shootTogate);
            turret.setOverridePose(paths.gateToLeave.endPose());
        });
        fsm.onStateUpdate(State.OPEN_GATEPASS_3, ( current, timeSinceTransition ) -> {
            if (!movement.isBusy() || (timeSinceTransition > failsafe)) {
                return State.INTAKE_GATEPASS_3;
            }
            return null;
        }) ;
        fsm.onStateUpdate(State.INTAKE_GATEPASS_3, (current, timeSinceTransition) -> timeSinceTransition > GATE_INTAKE_WAIT ? State.INTAKE_TO_SHOOT_GATEPASS_3 : null);
        fsm.onStateEnter(State.INTAKE_TO_SHOOT_GATEPASS_3, () -> movement.followPath(paths.gateToLeave));
        fsm.onStateUpdate(State.INTAKE_TO_SHOOT_GATEPASS_3, () -> !movement.isBusy() ? State.SHOOT_GATEPASS_3 : null);
        fsm.onStateEnter(State.SHOOT_GATEPASS_3, () -> intake.command(Intake.Command.LAUNCH));
        fsm.onStateUpdate(State.SHOOT_GATEPASS_3, (current, timeSinceTransition) -> {
            return timeSinceTransition > intake.getShootTime() ? State.DEAD : null;
        });

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
