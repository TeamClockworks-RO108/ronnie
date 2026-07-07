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

import org.firstinspires.ftc.teamcode.field.TeleOpPoses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Turret;

@Autonomous(name = "Auto Far Blue")
public class AutoFarBlue extends LinearOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(41.613, 9, Math.toRadians(90));

    private Flywheel flywheel;
    private Turret turret;
    private Intake intake;
    private static final TeleOpPoses poses = new TeleOpPoses();
    private ElapsedTime timer;

    private PathChain firstRowChain;

    private void initComponents() {
        flywheel = new Flywheel(hardwareMap, telemetry, follower, poses.blueGoal);
        turret = new Turret(hardwareMap, telemetry, follower, poses.blueGoal);
        intake = new Intake(hardwareMap);
        timer = new ElapsedTime();
    }

    private void buildChains() {
        firstRowChain = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(49.927, 41.641),
                                new Pose(2.057, 35.799)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(2.057, 35.799),
                                new Pose(41.000, 9.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        initComponents();
        buildChains();

        Command update = Command.build()
                .setExecute(this::update);

        Command shoot = Command.build()
                .setStart(() -> {
                    timer.reset();
                    intake.command(Intake.Command.LAUNCH);
                })
                .setDone(() -> timer.milliseconds() >= 2500);


        Command warmup = Command.build()
                .setStart(() -> {
                    timer.reset();
                    intake.command(Intake.Command.TOGGLE_INTAKE);
                })
                .setDone(() -> timer.milliseconds() >= 2500);


        Command takeFirstRow = Command.build()
                .setStart(() -> follower.followPath(firstRowChain))
                .setDone(() -> !follower.isBusy());

        waitForStart();

        Command auto = sequential(
                warmup,
                shoot,
                takeFirstRow,
                shoot
        );

        Scheduler.schedule(
                parallel(
                        auto,
                        update
                )
        );

        while (opModeIsActive()) {
            Scheduler.execute();
        }
    }

    private void update() {
        flywheel.update();
        turret.update();
        intake.update();
        follower.update();
    }
}
