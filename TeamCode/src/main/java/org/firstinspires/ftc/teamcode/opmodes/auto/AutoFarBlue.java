package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.field.poses.Poses;
import org.firstinspires.ftc.teamcode.field.poses.PosesBlue;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOpBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;
import org.firstinspires.ftc.teamcode.robot.Turret;

@Autonomous(name = "Auto Far Blue")
public class AutoFarBlue extends LinearOpMode {
    private PedroMovement movement;
    private final Pose startPose = new Pose(45.226, 7.7, Math.toRadians(90));

    private Flywheel flywheel;
    private Turret turret;
    private Intake intake;
    private static final Poses poses = TeamColor.BLUE.poses;
    private PathChain firstRowChain;

    private static final int WARMUP_TIME = 2500;

    private void initComponents() {
        movement = new PedroMovement(hardwareMap, telemetry, startPose);

        flywheel = new Flywheel(hardwareMap, telemetry, movement.getFollower(), poses.goal());
        turret = new Turret(hardwareMap, telemetry, movement.getFollower(), poses.goal());
        intake = new Intake(hardwareMap);
    }

    private void buildChains() {
        firstRowChain = movement.getFollower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(72.067, 42.647),
                                new Pose(2.057, 35.799)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(2.057, 35.799),
                                startPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
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

        Command auto = sequential(
                intake.getGatherCommand(),
                waitCommand(WARMUP_TIME),
                intake.getLaunchCommand(),
                takeFirstRow,
                waitCommand(500),
                intake.getLaunchCommand()
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
        movement.getFollower().breakFollowing();
    }

    private void update() {
        flywheel.update();
        turret.update();
        movement.update();
        TeleOpBase.drawRobotDashboard(movement.getFollower());
    }

    private Command waitCommand(long millis) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(timer::reset)
                .setDone(() -> timer.milliseconds() >= millis);
    }
}
