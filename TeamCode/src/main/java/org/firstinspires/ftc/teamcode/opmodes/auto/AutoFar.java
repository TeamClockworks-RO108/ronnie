package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.field.Strategy.FAR;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.field.paths.Paths;
import org.firstinspires.ftc.teamcode.field.poses.Poses;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOpBase;
import org.firstinspires.ftc.teamcode.robot.DistanceSensor;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.util.RobotContext;

public abstract class AutoFar extends LinearOpMode {
    private PedroMovement movement;
    private Flywheel flywheel;
    private Turret turret;
    private Intake intake;

    private final Poses poses;
    private final Paths paths;

    private PathChain firstRowChain;
    private PathChain takeDumpChain;
    private PathChain takeHumanChain;
    private DistanceSensor sensor;

    private static final int WARMUP_TIME = 2500;

    private Wrapper inDump = new Wrapper(false);

    public static class Wrapper {
        private boolean val;

        public Wrapper(boolean init) {
            val = init;
        }
        public void set(boolean val) {
            this.val = val;
        }

        public boolean get() {
            return val;
        }
    }

    public AutoFar(TeamColor color) {
        poses = color.poses;
        paths = color.paths;
    }

    private void initComponents() {
        movement = new PedroMovement(hardwareMap, telemetry, poses.getAutoStart(FAR));

        flywheel = new Flywheel(hardwareMap, telemetry, movement.getFollower(), poses);
        turret = new Turret(hardwareMap, telemetry, movement, poses, true);
        intake = new Intake(hardwareMap);

        sensor = new DistanceSensor(hardwareMap, telemetry);
    }

    private void buildChains() {
        firstRowChain = paths.getFirstRow(movement.getFollower());
        takeHumanChain = paths.getHuman(movement.getFollower());
        takeDumpChain = paths.getDump(movement.getFollower(), sensor);
    }

    @Override
    public void runOpMode() {
        initComponents();
        buildChains();

        Command update = Command.build()
                .setExecute(this::update);

        Command takeFirstRow = Command.build()
                .setStart(() -> movement.getFollower().followPath(firstRowChain))
                .setDone(() -> !movement.getFollower().isBusy());

        Command takeDump = Command.build()
                .setStart(() -> movement.getFollower().followPath(takeDumpChain))
                .setDone(() -> !movement.getFollower().isBusy());

        Command takeHuman = Command.build()
                .setStart(() -> movement.getFollower().followPath(takeHumanChain))
                .setExecute(() -> {
                    inDump.set(true);
                })
                .setDone(() -> !movement.getFollower().isBusy())
                .setEnd((end) -> {
                   inDump.set(false);
                });

        Command checkForFull = Command.build()
                .setDone(() -> sensor.detectedFor(400) || !movement.getFollower().isBusy())
                .setEnd((e) -> {
                    if(!inDump.get()) return;

                    movement.getFollower().breakFollowing();
                    PathChain chain = paths.getReturn(movement.getFollower());
                    movement.getFollower().followPath(chain);
                });

        Command takeDumpChain =
                parallel(
                        sequential(
                                takeDump,
                                waitCommand(150),
                                intake.getLaunchCommand()
                        ),
                        checkForFull
                );

        Command auto = sequential(
                intake.getTogglePowerCommand(),
                waitCommand(WARMUP_TIME),

                intake.getLaunchCommand(),

                takeFirstRow,
                waitCommand(500),
                intake.getLaunchCommand(),

                //human preload
                takeHuman,
                waitCommand(150),
                intake.getLaunchCommand(),

                takeDumpChain,
                takeDumpChain,
                takeDumpChain,
                takeDumpChain
        );

        waitForStart();

        Scheduler.schedule(
                parallel(
                        auto,
                        update
                )
        );

        while (opModeIsActive()) {
            Scheduler.execute();
        }

        Scheduler.reset();
    }

    private void update() {
        flywheel.update();
        turret.update();
        movement.update();
        sensor.update();
        TeleOpBase.drawRobotDashboard(movement.getFollower());
        RobotContext.setLastPose(movement.getFollower().getPose());
        telemetry.update();
    }

    private Command waitCommand(long millis) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(timer::reset)
                .setDone(() -> timer.milliseconds() >= millis);
    }
}
