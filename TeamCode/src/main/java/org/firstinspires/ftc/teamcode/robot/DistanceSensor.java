package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DistanceSensor {
    private final Telemetry telemetry;
    private final DigitalChannel sensor;
    private final ElapsedTime timer;
    private boolean read;

    public DistanceSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        sensor = hardwareMap.get(DigitalChannel.class, "Distance");
        read = false;
        timer = new ElapsedTime();
        this.telemetry = telemetry;
    }

    public void update() {
        if (!sensor.getState()) {
            read = false;
        }

        if (sensor.getState() && !read) {
            read = true;
            timer.reset();
        }

        telemetry.addData("Distance state", sensor.getState());
        telemetry.addData("Reading", read);
        if (read) {
            telemetry.addData("Reading for", timer.milliseconds());
        } else {
            telemetry.addData("Last read", "None");
        }
    }

    public boolean detectedFor(int time) {
        return sensor.getState() && timer.milliseconds() >= time && read;
    }
}
