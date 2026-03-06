package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    protected double kp;
    protected double ki;
    protected double kd;
    protected double targetValue = 0;
    protected double integral = 0;
    protected double lastError = 0;

    protected ElapsedTime timer;

    public PID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        timer = new ElapsedTime();
    }

    public double calculate(double inputValue) {
        double error = targetValue - inputValue;
        double derivative = (error - lastError) / timer.seconds();
        integral += error * timer.seconds();

        lastError = error;
        timer.reset();
        return kp * error + kd * derivative + ki * integral;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
    }

    public void setTargetValue(double targetValue) {
        this.targetValue = targetValue;
    }
}
