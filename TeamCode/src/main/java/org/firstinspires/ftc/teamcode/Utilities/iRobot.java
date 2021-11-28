package org.firstinspires.ftc.teamcode.Utilities;

/**
 * The is an interface (specifies behavior other classes must implement).
 *
 */
public interface iRobot {
    /**
     * drive is a TeleOp method that moves the robot.
     * All inputs are relative to the front of the robot.
     *
     * @param y Accepts between -1.0 and 1.0 Negative y is backwards, and positive y is forwards
     * @param x Accepts between -1.0 and 1.0 Negative x is left, and positive x is right
     */
    void drive(double y, double x);

    /**
     * rotate is a TeleOp method that rotates the robot.
     * All inputs are relative to the front of the robot being 0°.
     *
     * @param degrees Accepts between -1.0 and 1.0 Negative degrees is counterclockwise, and positive degrees is clockwise.
     */
    void rotate(double degrees);

    /**
     * driveXYR is a TeleOp method that moves the robot on the x axis the y axis and rotates.
     * All inputs are relative to the front of the robot.
     *
     * @param y Accepts between -1.0 and 1.0 Negative y is backwards, and positive y is forwards
     * @param x Accepts between -1.0 and 1.0 Negative x is left, and positive x is right
     * @param r Accepts between -1.0 and 1.0 Negative degrees is counterclockwise, and positive degrees is clockwise.
     */
    void driveXYR(double y, double x, double r);

    /**
     * driveStop is a TeleOp method that stops the robot driving by stopping the wheels
     */
    void driveStop();

    /**
     * stopAll is a TeleOp method that stops all functions of the robot by powering down all motors
     */
    void stopAll();
}
