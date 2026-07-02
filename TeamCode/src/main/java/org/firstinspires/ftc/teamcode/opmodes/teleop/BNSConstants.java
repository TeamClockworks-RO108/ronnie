package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class BNSConstants {

    // IT IS IMPORTANT THAT THE NEXT 6 PARAMETERS ARE SYMMETRICAL
    // FOR A SINGLE SERVO (leftServo, rightServo)
    // left + right = center * 2
    // !!!!!!!!!!!!!!!!!!!!!!!!!
    // Tune centers first. Then set the left and right to the center value.
    // Then tune left values.
    public static double leftServoCenter = 0.3;
    
    public static double leftServoSteerLeftLimit = 0.5;

    public static double leftServoSteerRightLimit = 0.7;

    public static double rightServoCenter = 0.3;

    public static double rightServoSteerLeftLimit = 0.5;

    public static double rightServoSteerRightLimit = 0.7;


    public static double motorInactivePower = 0.1;


    public static double dpadLeftSteerPower = 0.3;

    public static double dpadRightSteerPower = 0.3;

    public static double joystickSteerPower = 0.8;


    // Allows the left and right wheels to be driven differently during steering
    // Enable during RALLY, Disable during DRAG    !!!
    public static boolean enableAckermannDifferentialPower = true;


    // Tiered powerup: encoder velocity (ticks/sec) at which an extra motor per side engages.
    // Below threshold1: 2 motors/side. Above threshold1: 3. Above threshold2: 4.
    public static double powerupThreshold1 = 1000;
    public static double powerupThreshold2 = 1500;

    // Encoder velocity readings outside this range (ticks/sec) are ignored when gauging speed.
    public static double encoderValidMin = 100;
    public static double encoderValidMax = 9000;













    // CAD TUNED CONSTANTS
    // DO NOT MODIFY FOR THE LOVE OF SAURON LORD OF MORDOR
    public static double ackermannWidth = 205.0 / 1000; // Meters
    public static double ackermannLength = 196.0 / 1000; // Meters

    public static double ackermannMobilityLimit = 30 * Math.PI / 180.0;



    public static double lerp(double a, double b, double gradient) {
        if (gradient > 1) gradient = 1;
        if (gradient < 0) gradient = 0;
        return a * (1 - gradient) + b * gradient;
    }

}
