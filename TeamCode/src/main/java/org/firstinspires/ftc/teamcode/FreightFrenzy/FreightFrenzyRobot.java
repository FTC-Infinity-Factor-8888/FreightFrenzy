package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.iRobot;

public class FreightFrenzyRobot implements iRobot {
    private final HardwareMap hardwareMap;
    public Telemetry telemetry;
    private DcMotorEx RFMotor;
    private DcMotorEx RRMotor;
    private DcMotorEx LFMotor;
    private DcMotorEx LRMotor;
    private DcMotor DWMotor;
    private DcMotorEx LiftMotor;
    private DcMotor SpintakeMotor;
    private BNO055IMU imu;


    private double lfSpeed; // Left Front motor speed.
    private double rfSpeed; // Right Front motor speed.
    private double lrSpeed; // Left Rear motor speed.
    private double rrSpeed; // Right Rear motor speed.

    private double MAX_ROBOT_SPEED = 0.80; // The maximum speed we want our robot to drive at.
    private double MIN_ROBOT_SPEED = 0.20; // The minimum speed we can have our robot to drive at.
    private double normalSpeed = 0.50; // The normal speed our robot should be driving at.
    private double accelerationSpeed = MAX_ROBOT_SPEED - normalSpeed; // The acceleration speed set on normal speed.

    private final double wheelCircumferenceInInches = (96 / 25.4) * Math.PI;
    //  private int maximumTicksPerSecond = 2800;
    private final double ticksPerMotorRevolution = 530.3;
    private final double ticksPerInch = ticksPerMotorRevolution / wheelCircumferenceInInches;

    private final int maxLiftPosition = 824;  // The maximum amount of degrees the motor turns before the lift
    // reaches its maximum height.
    private final int minLiftPosition = 0; // The minimum amount of degrees the motor turns before the lift
    // reaches its minimum height.
    private double duckWheelSpeed = 0.6; // The speed the wheel to turn the duck carousel moves at.
    double liftSpeed = 0.3; // The speed the lift moves at.
    double spintakeSpeed = 0.6;

    private double turnSpeed = 0.1;
    private double delta;
    private double deltaThreshold = 1;
    private double turnDeltaThreshold = 5;

    // todo set x!
    //  double x = Gamepad.
    public FreightFrenzyRobot(LinearOpMode creator) {
        this.hardwareMap = creator.hardwareMap;
    }

    private double getIMUHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    @Override
    public void initHardware() {
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LRMotor = hardwareMap.get(DcMotor.class, "LRMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RRMotor = hardwareMap.get(DcMotor.class, "RRMotor");
        DWMotor = hardwareMap.get(DcMotor.class, "DWMotor");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor"); // Port: 0 exp.d
        SpintakeMotor = hardwareMap.get(DcMotor.class, "SpintakeMotor");
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void drive(double distance) {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double desiredHeading = getIMUHeading();
        setMotorDistanceToTravel(distance, new int[]{1, 1, 1, 1});

        double absDistance = Math.abs(distance);
        double direction = (distance > 0) ? 1 : -1;

        double accelInches;
        double decelInches;

        double halfSlope = MAX_ROBOT_SPEED - MIN_ROBOT_SPEED;


        if (absDistance / 8 < 2) {
            accelInches = 2;
        }
        else {
            accelInches = absDistance / 8;
        }

        if (absDistance / 6 < 3) {
            decelInches = 3;
        }
        else {
            decelInches = absDistance / 6;
        }

        double wholeAccelSlope = halfSlope / accelInches;
        double wholeDecelSlope = -halfSlope / decelInches;

        System.out.println("FLLDrive: distance " + distance + ", wholeAccelSlope " + wholeAccelSlope + ", wholeDecelSlope " + wholeDecelSlope);

        powerTheWheels(MIN_ROBOT_SPEED, MIN_ROBOT_SPEED, MIN_ROBOT_SPEED, MIN_ROBOT_SPEED);
        telemetryDashboard("FLLDrive(" + distance + ")");
        while (creator.opModeIsActive() && motorsShouldContinue(distance, new int[]{1, 1, 1, 1})) {
            double motorPosition = getMotorPosition();
            double power = 0;

            if (motorPosition <= accelInches) {
                power = MIN_ROBOT_SPEED + wholeAccelSlope * motorPosition;
            }
            else if (motorPosition <= distance - accelInches - decelInches) {
                power = MAX_ROBOT_SPEED;
            }
            else if (motorPosition <= distance) {
                power = MIN_ROBOT_SPEED + wholeDecelSlope * motorPosition;
            }
            power *= direction;

            double currentHeading = getImuHeading();
            delta = normalizeHeading(desiredHeading - currentHeading);
            double adjustSpeed = 0;
            if (Math.abs(delta) > deltaThreshold) {
                adjustSpeed = correctionSpeed;
                if (delta > 0) {
                    adjustSpeed *= -1;
                }
            }
            leftSpeed = power + adjustSpeed;
            rightSpeed = power - adjustSpeed;

            System.out.println("FLLDrive: motorPosition " + motorPosition + " power " + power);
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            telemetryDashboard("FLLDrive(" + distance + ")");
        }

        if (!creator.opModeIsActive()) {
            throw new EmergencyStopException("FLLDrive");
        }

        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerTheWheels(0, 0, 0, 0);
    }

    @Override
    public void strafe(double distance) {

    }

    @Override
    public void rotate(double desiredHeading) {

    }

    public void duckWheelMotor(int direction) {
        double power = 0; // By default the wheel should not rotate.
        // If the rotate left button is pressed, rotate left.
        if(direction == 1) {
            power = duckWheelSpeed;
        }
        // If the rotate right button is pressed, rotate right.
        else if(direction == -1) {
            power = -duckWheelSpeed;
        }
        // Setting the power the duck wheel motor should move at.
        DWMotor.setPower(power);
    }

    public void liftMotor(int direction) {
        if (direction == 0) {
            LiftMotor.setPower(0);
        } else {
            // Move to the position as specified above.
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Setting the speed that the lift moves at.
            // If the direction is positive move the lift up.
            if (direction == 1) {
                LiftMotor.setTargetPosition(maxLiftPosition);
            }
            // If the direction is negative move the lift down.
            else if (direction == -1) {
                LiftMotor.setTargetPosition(minLiftPosition);
            }
            LiftMotor.setPower(liftSpeed);
        }
    }

    public void spinTakeMotor (int direction) {
        double power = 0;
        if(direction == 1) {
            power = spintakeSpeed;
        }
        else if(direction == -1) {
            power = -spintakeSpeed;
        }
        SpintakeMotor.setPower(power);
    }

    @Override
    public void driveXYRB(double x, double y, double r, double b) {
        /*
                Because we use Mecanum wheels, we can move forward, rotate, and strafe.
                Here, we are taking into account the direction each wheel should travel at in
                order to move in the direction we want the robot to move.
                 */
        lfSpeed = ((y - x - r) * normalSpeed);
        rfSpeed = ((y + x + r) * normalSpeed);
        lrSpeed = ((y + x - r) * normalSpeed);
        rrSpeed = ((y - x + r) * normalSpeed);

        if (Math.abs(lfSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(lfSpeed) > normalSpeed) {
                if (lfSpeed > 0) {
                    lfSpeed = normalSpeed;
                    LFMotor.setPower(lfSpeed + accelerationSpeed * b);
                } else if (lfSpeed < 0) {
                    lfSpeed = -normalSpeed;
                    LFMotor.setPower(lfSpeed - accelerationSpeed * b);
                } else {
                    LFMotor.setPower(0);
                }
            }
        } else {
            if (lfSpeed > 0) {
                LFMotor.setPower(lfSpeed + accelerationSpeed * b);
            } else if (lfSpeed < 0) {
                LFMotor.setPower(lfSpeed - accelerationSpeed * b);
            } else {
                LFMotor.setPower(0);
            }
        }

        if (Math.abs(rfSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(rfSpeed) > normalSpeed) {
                if (rfSpeed > 0) {
                    rfSpeed = normalSpeed;
                    RFMotor.setPower(rfSpeed + accelerationSpeed * b);
                } else if (rfSpeed < 0) {
                    rfSpeed = -normalSpeed;
                    RFMotor.setPower(rfSpeed - accelerationSpeed * b);
                } else {
                    RFMotor.setPower(0);
                }
            }
        } else {
            if (rfSpeed > 0) {
                RFMotor.setPower(rfSpeed + accelerationSpeed * b);
            } else if (rfSpeed < 0) {
                RFMotor.setPower(rfSpeed - accelerationSpeed * b);
            } else {
                RFMotor.setPower(0);
            }
        }

        if (Math.abs(lrSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(lrSpeed) > normalSpeed) {
                if (lrSpeed > 0) {
                    lrSpeed = normalSpeed;
                    LRMotor.setPower(lrSpeed + accelerationSpeed * b);
                } else if (lrSpeed < 0) {
                    lrSpeed = -normalSpeed;
                    LRMotor.setPower(lrSpeed - accelerationSpeed * b);
                } else {
                    LRMotor.setPower(0);
                }
            }
        } else {
            if (lrSpeed > 0) {
                LRMotor.setPower(lrSpeed + accelerationSpeed * b);
            } else if (lrSpeed < 0) {
                LRMotor.setPower(lrSpeed - accelerationSpeed * b);
            } else {
                LRMotor.setPower(0);
            }
        }

        if (Math.abs(rrSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(rrSpeed) > normalSpeed) {
                if (rrSpeed > 0) {
                    rrSpeed = normalSpeed;
                    RRMotor.setPower(rrSpeed + accelerationSpeed * b);
                } else if (rrSpeed < 0) {
                    rrSpeed = -normalSpeed;
                    RRMotor.setPower(rrSpeed - accelerationSpeed * b);
                } else {
                    RRMotor.setPower(0);
                }
            }
        } else {
            if (rrSpeed > 0) {
                RRMotor.setPower(rrSpeed + accelerationSpeed * b);
            } else if (rrSpeed < 0) {
                RRMotor.setPower(rrSpeed - accelerationSpeed * b);
            } else {
                RRMotor.setPower(0);
            }
        }
    }

    @Override
    public void driveStop() {
        RFMotor.setPower(0.0);
        RRMotor.setPower(0.0);
        LFMotor.setPower(0.0);
        LRMotor.setPower(0.0);
    }

    public void armsStop() {
        DWMotor.setPower(0.0);

    }

    @Override
    public void stopAll() {
        driveStop();
        armsStop();
    }

    public int getCurrentLiftPosition () {
        return LiftMotor.getCurrentPosition();
    }

    @Override
    public double normalizeHeading(double heading) {
        while (heading >= 180.0 || heading < -180.0) {
            if (heading >= 180.0) {
                heading -= 360.0;
            } else if (heading < -180.0) {
                heading += 360.0;
            }
        }
        return heading;
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        LFMotor.setMode(mode);
        LRMotor.setMode(mode);
        RFMotor.setMode(mode);
        RRMotor.setMode(mode);
    }

    /**
     * Programs all four motors to run to position, based off of distance and direction.
     *
     * @param distance The distance you want to drive in inches
     * @param direction The direction each motor should turn. It is an array consisting of the
     *                  LfMotor, LrMotor, RfMotor, and RrMotor. The values can be -1 to move backwards,
     *                  1 to move forwards, or 0 to not move the motor at all.
     */
    private void setMotorDistanceToTravel(double distance, int[] direction) {

        if(direction.length != 4) {
            throw new IllegalArgumentException("You must provide an array with exactly 4 elements!");
        }

        for(int i = 0; i < 4; i++) {
            if(direction[i] > 1 || direction[i] < -1) {
                throw new IllegalArgumentException("Elements must be -1, 0, or 1.");
            }
        }

        double distanceInTicks = distance * ticksPerInch;
        int leftFrontTargetPosition = (int) (LFMotor.getCurrentPosition() + distanceInTicks);
        int leftRearTargetPosition = (int) (LRMotor.getCurrentPosition() + distanceInTicks);
        int rightFrontTargetPosition = (int) (RFMotor.getCurrentPosition() + distanceInTicks);
        int rightRearTargetPosition = (int) (RRMotor.getCurrentPosition() + distanceInTicks);

        LFMotor.setTargetPosition(direction[0] * leftFrontTargetPosition);
        LRMotor.setTargetPosition(direction[1] * leftRearTargetPosition);
        RFMotor.setTargetPosition(direction[2] * rightFrontTargetPosition);
        RRMotor.setTargetPosition(direction[3] * rightRearTargetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

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
    private void powerTheWheels(double lfPower, double lrPower, double rfPower, double rrPower) {
        double leftMax = Math.max(Math.abs(lfPower), Math.abs(lrPower));
        double rightMax = Math.max(Math.abs(rfPower), Math.abs(rrPower));
        double max = Math.max (leftMax, rightMax);

        if (max > MAX_ROBOT_SPEED) {
            lfPower /= max;
            lrPower /= max;
            rfPower /= max;
            rrPower /= max;
        }

        if (LFMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)) {
            double lfVelocity = lfPower * maxRobotTps;
            double lrVelocity = lrPower * maximumRobotTps;
            double rfVelocity = rfPower * maximumRobotTps;
            double rrVelocity = rrPower * maximumRobotTps;

            if (creator.opModeIsActive()) {

                LFMotor.setVelocity(lfVelocity);
                LRMotor.setVelocity(lrVelocity);
                RFMotor.setVelocity(rfVelocity);
                RRMotor.setVelocity(rrVelocity);
            }
            else {
                throw new EmergencyStopException("PowerTheWheels");
            }
        }
        else {
            // We assume that we will be using RUN_TO_POSITION mode.
            if(creator.opModeIsActive()) {
                LFMotor.setPower(lfPower);
                LRMotor.setPower(lrPower);
                RFMotor.setPower(rfPower);
                RRMotor.setPower(rrPower);
            }
            else {
                throw new EmergencyStopException("PowerTheWheels");
            }
        }
    }

}
