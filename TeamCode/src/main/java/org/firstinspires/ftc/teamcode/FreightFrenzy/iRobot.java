package org.firstinspires.ftc.teamcode.FreightFrenzy;

/**
 * The is an interface (specifies behavior other classes must implement).
 *
 */
public interface iRobot {
    /**
     * drive is a TeleOp method that moves and possibly rotates the robot.
     * All inputs are relative to the front of the robot.
     *
     * @param y Accepts between -1.0 and 1.0 Negative y is backwards, and positive y is forwards
     * @param x Accepts between -1.0 and 1.0 Negative x is left, and positive x is right
     * @param r Accepts between -1.0 and 1.0 Negative r is turning left, and positive r is turning right
     */
    void drive (double y, double x, double r);

}
