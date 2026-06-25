package org.firstinspires.ftc.teamcode.util;

public class MathUtil {
    public static double clamp(double val, double minVal, double maxVal) {
        return Math.max(minVal, Math.min(maxVal, val));
    }
}
