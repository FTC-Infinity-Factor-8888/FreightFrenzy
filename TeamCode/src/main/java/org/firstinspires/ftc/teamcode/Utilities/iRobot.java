package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * The is an interface (specifies behavior other classes must implement).
 *
 */
public interface iRobot {
    /**
     * initHardware maps and initializes necessary hardware at the beginning of the match.
     */
    void initHardware();

    /**
     * drive is a Autonomous method that moves the robot forwards or backwards ONLY.
     * All inputs are relative to the front of the robot.
     *
     * @param distance Accepts a positive or negative number representing the number of inches to move
     *                 Negative is backwards, and positive is forwards
     */
    void drive(double distance);

    /**
     * TODO:JavaDoc
     * @param desiredHeading add description
     * @param power add description
     */
    void driveDelta (double desiredHeading, double power);

    /**
     *
     * @param direction 1 = forward, 0 = stop, -1 = backwards
     * @param accelSlope The acceleration slope
     * @param decelSlope    The deceleration slope
     * @return Returns true if drive should exit, false if it may continue
     */
    boolean driveAsserts(int direction, double accelSlope, double decelSlope);

    /**
     * drive is a Autonomous method that moves the robot right or left ONLY.
     * All inputs are relative to the front of the robot (looking at it from behind, as the robot)
     *
     * @param distance Accepts a positive or negative number representing the number of inches to move.
     *                 Negative is left, and positive is right
     */
    void strafe(double distance);

    /**
     * rotate is a Autonomous method that rotates the robot.
     * All inputs are relative to the front of the robot being 0°.
     *
     * @param degrees Accepts between -180 and 180 Negative degrees is counterclockwise, and positive degrees is clockwise.
     */
    void rotate(double degrees);

    /**
     * driveXYRB is a TeleOp method that moves the robot on the x axis the y axis and rotates.
     * All inputs are relative to the front of the robot.
     *  @param x Accepts between -1.0 and 1.0 Negative x is left, and positive x is right
     * @param y Accepts between -1.0 and 1.0 Negative y is backwards, and positive y is forwards
     * @param r (rotate)Accepts between -1.0 and 1.0 Negative degrees is counterclockwise, and positive degrees is clockwise.
     * @param b (boost) Accepts between 0 and 1.0. The percentage of extra speed that you want the robot to go past the normalSpeed.
     */
    void driveXYRB(double x, double y, double r, double b);

    /**
     * driveStop is a TeleOp method that stops the robot driving by stopping the wheels
     */
    void driveStop();

    /**
     * stopAll is a TeleOp method that stops all functions of the robot by powering down all motors
     */
    void stopAll();

    /** TODO: JavaDoc
     * normalizeHeading is a TeleOp method that...
     * @param heading add description
     */
    double normalizeHeading(double heading);

    /** TODO: JavaDoc
     *
     * @param desiredHeading is the desired heading for the robot
     */
    void hold(double desiredHeading);

    /** TODO: JavaDoc
     *
     */
    double getIMUHeading();

    /** TODO: JavaDoc
     *
     * @param mode one of the DcMotor modes outlined
     */
     void setMotorMode(DcMotor.RunMode mode);

    /**
     * Programs all four motors to run to position, based off of distance and direction.
     *
     * @param distance The distance you want to drive in inches
     * @param direction The direction each motor should turn. It is an array consisting of the
     *                  LfMotor, LrMotor, RfMotor, and RrMotor. The values can be -1 to move backwards,
     *                  1 to move forwards, or 0 to not move the motor at all.
     */
    void setMotorDistanceToTravel(double distance, int[] direction);

    /**
     * Powers all 4 of the robot's wheels.
     * If the motor mode is set to RUN_USING_ENCODER, then PTW sets the velocity.
     * If the motor mode is set to RUN_TO_POSITION, then PTW sets the power.
     *
     * The robot is only capable of accepting speeds of -1 --> 1.
     * If you give a value out of that range, PTW will scale down the numbers appropriately.
     *
     * @param lfPower power/velocity applied to the left front wheel.
     * @param lrPower power/velocity applied to the left rear wheel.
     * @param rfPower power/velocity applied to the right front wheel.
     * @param rrPower power/velocity applied to the right rear wheel.
     */
    void powerTheWheels(double lfPower, double lrPower, double rfPower, double rrPower);

    /**
     * TODO: JavaDoc
     *
     * @param distance add description
     * @param motorDirection add description
     * @return add description
     */
    boolean motorsShouldContinue(double distance, int[] motorDirection);

    /**
     * TODO: JavaDoc
     * @param motor add description
     * @param distance add description
     * @return add description
     */
    boolean checkMotorPosition(DcMotorEx motor, double distance);

    /**
     * @return returns the distance the robot has traveled in inches
     */
    double getMotorPosition();

    /**
     * TODO:JavaDoc
     * @param delta add description
     * @return add description
     */
    double powerPercentage(double delta);

    /**
     * TODO:JavaDoc
     * @param motor add description
     * @param tps add description
     */
    void setPIDFValues(DcMotorEx motor, int tps);

    /**
     * Displays telemetry information on the Driver Hub
     */
    void telemetryDashboard();

    /**
     * TODO:JavaDoc
     */
    void initializeIMU();


}
