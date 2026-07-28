package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.field.Strategy.FAR;

import com.pedropathing.geometry.Pose;
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
    private PathChain takeDumpChain2;
    private PathChain takeHumanChain;
    private PathChain parkChain;
    private DistanceSensor sensor;

    private final TeamColor color;

    private static final Pose goal = new Pose(134, 135);

    private static final int WARMUP_TIME = 1800;

    private final Wrapper inDump = new Wrapper(false);

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
        this.color = color;
    }

    private void initComponents() {
        movement = new PedroMovement(hardwareMap, telemetry, poses.getAutoStart(FAR));

        flywheel = new Flywheel(hardwareMap, telemetry, movement.getFollower(), poses);
        turret = new Turret(hardwareMap, telemetry, movement, goal, true, color);
        intake = new Intake(hardwareMap, telemetry);

        sensor = new DistanceSensor(hardwareMap, telemetry);
    }

    private void buildChains() {
        firstRowChain = paths.getFirstRow(movement.getFollower());
        takeHumanChain = paths.getHuman(movement.getFollower());
        takeDumpChain = paths.getDump(movement.getFollower(), sensor);
        takeDumpChain2 = paths.getDump(movement.getFollower(), sensor);
        parkChain = paths.getPark(movement.getFollower());
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


        Command takeDump2 = Command.build()
                .setStart(() -> movement.getFollower().followPath(takeDumpChain2))
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
//                                waitCommand(150),
                                intake.getLaunchNoCloseCommand()
                        ),
                        checkForFull
                );
        Command takeDumpChain2 =
                parallel(
                        sequential(
                                takeDump2,
//                                waitCommand(150),
                                intake.getLaunchNoCloseCommand()
                        ),
                        checkForFull
                );

        Command park = Command.build()
                .setStart(() -> {
                    movement.getFollower().followPath(parkChain);
                })
                .setDone(() -> !movement.isBusy());

        Command auto = sequential(
                intake.getTogglePowerCommand(),
                waitCommand(WARMUP_TIME),

                intake.getLaunchNoCloseCommand(),

                takeFirstRow,
                intake.getLaunchNoCloseCommand(),

                //human preload
                takeHuman,
                intake.getLaunchNoCloseCommand(),

                takeDumpChain,
                takeDumpChain2,
                takeDumpChain,
                takeDumpChain2,
                takeDumpChain,
                takeDumpChain2,
                park
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
