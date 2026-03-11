package org.firstinspires.ftc.teamcode.util;

public class AngleUtil {

    /**
     * Normalize an angle between pi and -pi
     * @param angle
     * @return
     */
    public static double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < Math.PI) angle += 2 * Math.PI;
        return angle;
    }

}
