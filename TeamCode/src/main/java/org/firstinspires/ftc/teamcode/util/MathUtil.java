package org.firstinspires.ftc.teamcode.util;

public class MathUtil {
    public static double clamp(double val, double minVal, double maxVal) {
        if(val < minVal) return minVal;
        if(val > maxVal) return maxVal;
        return val;
    }
}
