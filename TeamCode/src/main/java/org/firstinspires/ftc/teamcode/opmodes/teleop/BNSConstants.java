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


    // Power for inactive motors.
    // Can raise to 0.2 if battery is healthy during starts
    public static double motorInactivePower = 0.1;


    // How hard do you want to steer during DRAG.
    // You will typically want to steer using short presses to keep the car on the road.
    // These two variables should be the same, but can be different if car is asymmetric.
    // Values between 0 .. 1
    public static double dpadLeftSteerPower = 0.3;
    public static double dpadRightSteerPower = 0.3;

    // How hard do you want to steer during RALLY using joystick
    // Values between 0 .. 1
    public static double joystickSteerPower = 0.8;


    // Allows the left and right wheels to be driven differently during steering
    // Enable during RALLY, Disable during DRAG    !!!
    public static boolean enableAckermannDifferentialPower = true;


    // Tiered powerup: encoder velocity (ticks/sec) at which an extra motor per side engages.
    // Below threshold1: 2 motors/side. Above threshold1: 3. Above threshold2: 4.
    // TUNING: the ideal number for these two thresholds should be at 30% (1) and
    // TUNING: set(1) at the speed at which current is 13A
    // TUNING: set(2) at the speed at which current is 15A
    // IMPORTANT TUNING PROCESS: DO NOT USE THROTTLE CONTROL
    // 0. Set these two at 9999999 (never activate during test) (IMPORTANT!)
    // 1. Run the robot with throttle all the way, at full blast.
    // 2. Look into Panels/Dashboard graph for the moment current dips below 13A
    // 3. The speed at that moment is the desired treshold.
    // 4. IMPORTANT! Set the powerupThreshold1 you found earlier before proceeding with threshold 2 !!!!
    // 5. Repeat steps 1-3, but with 15A and for variable powerupThreshold2
    // Note: you can lower the values later to get quicker acceleration. Doing so may fuck the battery and/or fuse.
    //                                                                   Lower at your own risk.
    //
    // If too low, you will burn the fuse, only on the ground, not when running in the air.
    // Tune with robot in the air or on the ground. Validate these are good ON THE GROUND.
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
