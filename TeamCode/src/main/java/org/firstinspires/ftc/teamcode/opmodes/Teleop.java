package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;
import org.firstinspires.ftc.teamcode.util.EdgeDetector;

@TeleOp(name = "TeleOp")
public class Teleop extends OpMode {
    private PedroMovement movement;
    private Intake intake;
    private EdgeDetector toggleIntake = new EdgeDetector(false);
    private EdgeDetector launch = new EdgeDetector(false);
    private EdgeDetector rotateTurretr = new EdgeDetector( false);
    private EdgeDetector rotateTurretl = new EdgeDetector( false);
    private ElapsedTime timer;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        movement = new PedroMovement(hardwareMap, telemetry, new Pose(25,  120, Math.toDegrees(180)));
        intake = new Intake(hardwareMap, telemetry, movement.getFollower());
        intake.setupFSM();

        toggleIntake.onPress(() -> intake.command(Intake.Command.TOGGLE_INTAKE));
        launch.onPress(() -> intake.command(Intake.Command.LAUNCH));

        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        movement.getFollower().startTeleOpDrive();
    }

    @Override
    public void loop() {
        movement.update(gamepad1, gamepad2);

        intake.updateFSM();

        toggleIntake.update(gamepad1.right_bumper);
        launch.update(gamepad1.cross);

        intake.rotateTurret(gamepad1.left_trigger - gamepad1.right_trigger);

        telemetry.addData("latency (ms)", timer.milliseconds());
        timer.reset();
    }
}
