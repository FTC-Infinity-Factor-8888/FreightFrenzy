package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.EmergencyStopException;
import org.firstinspires.ftc.teamcode.Utilities.iRobot;

public class FreightFrenzyRobot implements iRobot {
    private final LinearOpMode creator;
    private final HardwareMap hardwareMap;
    public Telemetry telemetry;
    private DcMotorEx rfMotor;
    private DcMotorEx rrMotor;
    private DcMotorEx lfMotor;
    private DcMotorEx lrMotor;
    private DcMotor dwMotor;
    private DcMotorEx LiftMotor;
    private DcMotor SpintakeMotor;
    private BNO055IMU imu;


    private final double MAX_ROBOT_SPEED = 0.80; // The maximum speed we want our robot to drive at.
    private final double MIN_ROBOT_SPEED = 0.40; // The minimum speed we can have our robot to drive at.
    private final double correctionSpeed = 0.1; //e
    private final double duckWheelSpeed = 0.65;


    private final double wheelCircumferenceInInches = (96 / 25.4) * Math.PI;
    // private int maximumRobotTps = 2610;
    private final int lfMotorMaxTps = 2655;
    private final int rfMotorMaxTps = 2650;
    private final int lrMotorMaxTps = 2610;
    private final int rrMotorMaxTps = 2615;
    private final double ticksPerMotorRevolution = 530.3;
    private final double ticksPerInch = ticksPerMotorRevolution / wheelCircumferenceInInches;
    private final double drivePositionPIDF = 5.0; // 4.5
    private final static double HOLD_TIME = 1000; // In ms

    double liftSpeed = 0.3; // The speed the lift moves at.
    double spintakeIntakeSpeed = 0.6;
    double spintakeOuttakeSpeed = -0.4;

    private double delta;
    private final double deltaThreshold = 1;

    public FreightFrenzyRobot(LinearOpMode creator) {
        this.creator = creator;
        this.hardwareMap = creator.hardwareMap;
        this.telemetry = creator.telemetry;
    }

    @Override
    public void initHardware() {
        lfMotor = hardwareMap.get(DcMotorEx.class, "LFMotor");
        lrMotor = hardwareMap.get(DcMotorEx.class, "LRMotor");
        rfMotor = hardwareMap.get(DcMotorEx.class, "RFMotor");
        rrMotor = hardwareMap.get(DcMotorEx.class, "RRMotor");
        dwMotor = hardwareMap.get(DcMotor.class, "DWMotor");
        LiftMotor = hardwareMap.get(DcMotorEx.class, "LiftMotor"); // Port: 0 exp.d
        SpintakeMotor = hardwareMap.get(DcMotor.class, "SpintakeMotor");
        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        System.out.print("LF: ");
        setPIDFValues(lfMotor, lfMotorMaxTps);
        System.out.print("RF: ");
        setPIDFValues(rfMotor, rfMotorMaxTps);
        System.out.print("LR: ");
        setPIDFValues(lrMotor, lrMotorMaxTps);
        System.out.print("RR: ");
        setPIDFValues(rrMotor, rrMotorMaxTps);

        initializeIMU();
    }

    private void initializeIMU() {
        BNO055IMU.Parameters imuParameters;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        telemetry.addData("Status", "Calibrating IMU...done");
        telemetry.update();
    }

    // TODO: JavaDoc
    public void telemetryDashboard() {
        telemetry.addData("Heading", "Desired: %.0f, Current: %.0f, Delta: %.0f",
                getIMUHeading(), getIMUHeading(), delta);

        telemetry.addData("Target", "LF: %d, LR: %d, RF: %d, RR: %d",
                lfMotor.getTargetPosition(), lrMotor.getTargetPosition(), rfMotor.getTargetPosition(), rrMotor.getTargetPosition());
        telemetry.addData("Position", "LF: %d, LR: %d, RF: %d, RR: %d",
                lfMotor.getCurrentPosition(), lrMotor.getCurrentPosition(), rfMotor.getCurrentPosition(), rrMotor.getCurrentPosition());
        telemetry.addData("Power", "LF: %.1f, LR: %.1f, RF: %.1f, RR: %.1f",
                lfMotor.getPower(), lrMotor.getPower(), rfMotor.getPower(), rrMotor.getPower());

        double imuHeading = getIMUHeading();

        telemetry.addData("IMU Heading", "%.0f", imuHeading);
        telemetry.update();
    }

    /*
    TODO: Drive method is not driving backwards.
    TODO: JavaDoc - Will not accept inches value between -5 and 5
    Things to check:
        setMotorDistanceToTravel();
        Setting the power at the beginning to move forwards regardless of the direction the motors should be moving in.
     */
    @Override
    public void drive(double distance) {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double desiredHeading = getIMUHeading();
        // TODO: Might be causing reverse problem
        setMotorDistanceToTravel(distance, new int[]{1, 1, 1, 1});

        double absDistance = Math.abs(distance);
        double direction = (distance > 0) ? 1 : -1;

        double leftSpeed;
        double rightSpeed;

        //rise-over-run code for accel/decel slope
        double accelRun = 2;
        double decelRun = 4;
        double accelRise = MAX_ROBOT_SPEED - MIN_ROBOT_SPEED;
        double decelRise = 0 - MAX_ROBOT_SPEED;
        //decel goes from max to 0


//        if (absDistance / 8 < 2) {
//            accelRun = 2;
//        }
//        else {
//            accelRun = absDistance / 8;
//        }
//
//        if (absDistance / 6 < 3) {
//            decelRun = 3;
//        }
//        else {
//            decelRun = absDistance / 6;
//        }

        double accelSlope = accelRise / accelRun;
        double decelSlope = decelRise / decelRun;
        if(direction > 0) {
            powerTheWheels(MIN_ROBOT_SPEED, MIN_ROBOT_SPEED, MIN_ROBOT_SPEED, MIN_ROBOT_SPEED);
        }
        else {
            powerTheWheels(-MIN_ROBOT_SPEED, -MIN_ROBOT_SPEED, -MIN_ROBOT_SPEED, -MIN_ROBOT_SPEED);
        }
        telemetryDashboard();
        while (creator.opModeIsActive() && motorsShouldContinue(distance, new int[]{1, 1, 1, 1})) {
            double motorPosition = getMotorPosition();
            double power = 0;

            if (motorPosition <= accelRun) {
                power = MIN_ROBOT_SPEED + accelSlope * motorPosition;
            }
            if (direction == -1) {
                if (motorPosition <= distance - accelRun + decelRun) {
                    power = MAX_ROBOT_SPEED;
                }
            }
            else {
                if (motorPosition <= distance - accelRun - decelRun) {
                    power = MAX_ROBOT_SPEED;
                }
            }
            if (motorPosition <= distance) {
                power = MIN_ROBOT_SPEED + decelSlope * motorPosition;
            }
            power *= direction;

            double currentHeading = getIMUHeading();
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
            telemetry.addData("Left Power: ", leftSpeed);
            telemetry.addData("Right Power: ", rightSpeed);
            telemetry.update();
//            telemetryDashboard();
        }

        if (!creator.opModeIsActive()) {
            throw new EmergencyStopException("FLLDrive");
        }

        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerTheWheels(0, 0, 0, 0);
    }

    @Override
    public void strafe(double distance) {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double strafeSlippage = 1.1;
        double desiredHeading = getIMUHeading();
        double leftSpeed;
        double rightSpeed;
        double strafePositionPIDF = 6.5;
        lfMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        rfMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        lrMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        rrMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        setMotorDistanceToTravel(distance * strafeSlippage, new int[]{-1, 1, 1, -1});
        double speed = MIN_ROBOT_SPEED;
        leftSpeed = speed;
        rightSpeed = -speed;
        powerTheWheels(rightSpeed, leftSpeed, leftSpeed, rightSpeed);
        telemetryDashboard();
        while (creator.opModeIsActive() && motorsShouldContinue(distance, new int[]{-1, 1, 1, -1})) {
            double imuHeading = getIMUHeading();
            delta = normalizeHeading(desiredHeading - imuHeading);
            double adjustSpeed = 0;
            if (Math.abs(delta) > deltaThreshold) {
                adjustSpeed = correctionSpeed;
                if (delta > 0) {
                    adjustSpeed *= -1;
                }
            }
            leftSpeed = speed + adjustSpeed;
            rightSpeed = -speed - adjustSpeed;
            powerTheWheels(rightSpeed, leftSpeed, leftSpeed, rightSpeed);
            telemetryDashboard();

            double strafeRobotSpeed = 0.5;
            if (speed < strafeRobotSpeed) {
                double driveAccelerationIncrement = 0.075;
                speed += driveAccelerationIncrement;
            }
        }
        if (!creator.opModeIsActive()) {
            throw new EmergencyStopException("Strafe");
        }
        // Reset motor mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerTheWheels(0, 0, 0, 0);
        lfMotor.setPositionPIDFCoefficients(drivePositionPIDF);
        rfMotor.setPositionPIDFCoefficients(drivePositionPIDF);
        lrMotor.setPositionPIDFCoefficients(drivePositionPIDF);
        rrMotor.setPositionPIDFCoefficients(drivePositionPIDF);
    }

    @Override
    public void rotate(double desiredHeading) {
        double minTurnSpeed = 0.1;
        double currentHeading = getIMUHeading();
        double leftSpeed;
        double rightSpeed;
        delta = normalizeHeading(desiredHeading - currentHeading);
        double priorDelta = delta;
        int ringingCount = 0;
        double turnDeltaThreshold = 5;
        while (creator.opModeIsActive() && Math.abs(delta) > turnDeltaThreshold && ringingCount <= 3) {
            currentHeading = getIMUHeading();
            delta = normalizeHeading(desiredHeading - currentHeading);
            double deltaPercentage =  powerPercentage(delta);
            double turnSpeed = 0.1;
            double currentTurnSpeed = turnSpeed * deltaPercentage + minTurnSpeed;
            if (delta < 0) {
                currentTurnSpeed = -currentTurnSpeed;
            }
            leftSpeed = -currentTurnSpeed;
            rightSpeed = currentTurnSpeed;
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            telemetryDashboard();
            if (Math.signum(delta) != Math.signum(priorDelta) && delta != 0 && priorDelta != 0) {
                ringingCount++;
            }
            priorDelta = delta;
        }
        if(!creator.opModeIsActive()) {
            throw new EmergencyStopException("Turn");
        }

        powerTheWheels(0, 0, 0, 0);
        hold(desiredHeading);
    }

    public void duckWheelMotor(int direction) {
        double power = 0; // By default, the wheel should not rotate.
        // If the rotate-left button is pressed, rotate left.
        // reaches its minimum height.
        // The speed the wheel to turn the duck carousel moves at.
        if(direction == 1) {
            power = duckWheelSpeed;
        }
        // If the rotate-right button is pressed, rotate right.
        else if(direction == -1) {
            power = -duckWheelSpeed;
        }
        // Setting the power the duck wheel motor should move at.
        dwMotor.setPower(power);
    }

    public void duckWheelAutonomous(double rotations) {
        dwMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dwMotor.setTargetPosition((int)(rotations * 360));
        dwMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dwMotor.setPower(duckWheelSpeed);
    }

    public void liftMotor(int direction) {
        if (direction == 0) {
            LiftMotor.setPower(0);
        }
        else {
            // Move to the position as specified above.

            // Setting the speed that the lift moves at.
            // If the direction is positive move the lift, up.
            if (direction == 1) {
                // The maximum amount of degrees the motor turns before the lift
                int maxLiftPosition = 824;
                LiftMotor.setTargetPosition(maxLiftPosition);
            }
            // If the direction is negative move the lift down.
            else if (direction == -1) {
                // reaches its maximum height.
                // The minimum amount of degrees the motor turns before the lift
                int minLiftPosition = 0;
                LiftMotor.setTargetPosition(minLiftPosition);
            }
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotor.setPower(liftSpeed);
        }
    }

    public void spinTakeMotor (int direction) {
        double power = 0;
        if(direction == 1) {
            power = spintakeIntakeSpeed;
        }
        else if(direction == -1) {
            power = spintakeOuttakeSpeed;
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
        // Left Front motor speed.
        // The normal speed our robot should be driving at.
        double normalSpeed = 0.50;
        double lfSpeed = -((y - x - r) * normalSpeed);
        // Right Front motor speed.
        double rfSpeed = -((y + x + r) * normalSpeed);
        // Left Rear motor speed.
        double lrSpeed = -((y + x - r) * normalSpeed);
        // Right Rear motor speed.
        double rrSpeed = -((y - x + r) * normalSpeed);

        // The acceleration speed set on normal speed.
        double accelerationSpeed = MAX_ROBOT_SPEED - normalSpeed;
        if (Math.abs(lfSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(lfSpeed) > normalSpeed) {
                if (lfSpeed > 0) {
                    lfSpeed = normalSpeed;
                    lfMotor.setPower(lfSpeed + accelerationSpeed * b);
                } else if (lfSpeed < 0) {
                    lfSpeed = -normalSpeed;
                    lfMotor.setPower(lfSpeed - accelerationSpeed * b);
                } else {
                    lfMotor.setPower(0);
                }
            }
        } else {
            if (lfSpeed > 0) {
                lfMotor.setPower(lfSpeed + accelerationSpeed * b);
            } else if (lfSpeed < 0) {
                lfMotor.setPower(lfSpeed - accelerationSpeed * b);
            } else {
                lfMotor.setPower(0);
            }
        }

        if (Math.abs(rfSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(rfSpeed) > normalSpeed) {
                if (rfSpeed > 0) {
                    rfSpeed = normalSpeed;
                    rfMotor.setPower(rfSpeed + accelerationSpeed * b);
                } else if (rfSpeed < 0) {
                    rfSpeed = -normalSpeed;
                    rfMotor.setPower(rfSpeed - accelerationSpeed * b);
                } else {
                    rfMotor.setPower(0);
                }
            }
        } else {
            if (rfSpeed > 0) {
                rfMotor.setPower(rfSpeed + accelerationSpeed * b);
            } else if (rfSpeed < 0) {
                rfMotor.setPower(rfSpeed - accelerationSpeed * b);
            } else {
                rfMotor.setPower(0);
            }
        }

        if (Math.abs(lrSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(lrSpeed) > normalSpeed) {
                if (lrSpeed > 0) {
                    lrSpeed = normalSpeed;
                    lrMotor.setPower(lrSpeed + accelerationSpeed * b);
                } else if (lrSpeed < 0) {
                    lrSpeed = -normalSpeed;
                    lrMotor.setPower(lrSpeed - accelerationSpeed * b);
                } else {
                    lrMotor.setPower(0);
                }
            }
        } else {
            if (lrSpeed > 0) {
                lrMotor.setPower(lrSpeed + accelerationSpeed * b);
            } else if (lrSpeed < 0) {
                lrMotor.setPower(lrSpeed - accelerationSpeed * b);
            } else {
                lrMotor.setPower(0);
            }
        }

        if (Math.abs(rrSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(rrSpeed) > normalSpeed) {
                if (rrSpeed > 0) {
                    rrSpeed = normalSpeed;
                    rrMotor.setPower(rrSpeed + accelerationSpeed * b);
                } else if (rrSpeed < 0) {
                    rrSpeed = -normalSpeed;
                    rrMotor.setPower(rrSpeed - accelerationSpeed * b);
                } else {
                    rrMotor.setPower(0);
                }
            }
        } else {
            if (rrSpeed > 0) {
                rrMotor.setPower(rrSpeed + accelerationSpeed * b);
            } else if (rrSpeed < 0) {
                rrMotor.setPower(rrSpeed - accelerationSpeed * b);
            } else {
                rrMotor.setPower(0);
            }
        }
    }

    @Override
    public void driveStop() {
        rfMotor.setPower(0.0);
        rrMotor.setPower(0.0);
        lfMotor.setPower(0.0);
        lrMotor.setPower(0.0);
    }

    public void armsStop() {
        dwMotor.setPower(0.0);
        LiftMotor.setPower(0.0);
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
            }
            else {
                heading += 360.0;
            }
        }
        return heading;
    }
    public void hold(double desiredHeading) {
        ElapsedTime timer;

        double leftSpeed;
        double rightSpeed;
        double currentHeading = getIMUHeading();
        delta = normalizeHeading(desiredHeading - currentHeading);
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        while (creator.opModeIsActive() && Math.abs(delta) > 0.5 && timer.time() < HOLD_TIME) {
            double holdSpeed = 0.15;
            if (delta > 0) {
                leftSpeed = -holdSpeed;
                rightSpeed = holdSpeed;
            } else {
                leftSpeed = holdSpeed;
                rightSpeed = -holdSpeed;
            }
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            creator.sleep(75);
            powerTheWheels(0, 0, 0, 0);
            telemetryDashboard();
            currentHeading = getIMUHeading();
            delta = normalizeHeading(desiredHeading - currentHeading);
        }

        if(!creator.opModeIsActive()) {
            throw new EmergencyStopException("Hold");
        }

        powerTheWheels(0, 0, 0, 0);
    }

    private double getIMUHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        lfMotor.setMode(mode);
        lrMotor.setMode(mode);
        rfMotor.setMode(mode);
        rrMotor.setMode(mode);
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
        int leftFrontTargetPosition = (int) (lfMotor.getCurrentPosition() + distanceInTicks);
        int leftRearTargetPosition = (int) (lrMotor.getCurrentPosition() + distanceInTicks);
        int rightFrontTargetPosition = (int) (rfMotor.getCurrentPosition() + distanceInTicks);
        int rightRearTargetPosition = (int) (rrMotor.getCurrentPosition() + distanceInTicks);

        lfMotor.setTargetPosition(direction[0] * leftFrontTargetPosition);
        lrMotor.setTargetPosition(direction[1] * leftRearTargetPosition);
        rfMotor.setTargetPosition(direction[2] * rightFrontTargetPosition);
        rrMotor.setTargetPosition(direction[3] * rightRearTargetPosition);

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

        if (lfMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)) {
            double lfVelocity = lfPower * lfMotorMaxTps;
            double lrVelocity = lrPower * lrMotorMaxTps;
            double rfVelocity = rfPower * rfMotorMaxTps;
            double rrVelocity = rrPower * rrMotorMaxTps;

            if (creator.opModeIsActive()) {

                lfMotor.setVelocity(lfVelocity);
                lrMotor.setVelocity(lrVelocity);
                rfMotor.setVelocity(rfVelocity);
                rrMotor.setVelocity(rrVelocity);
            }
            else {
                throw new EmergencyStopException("PowerTheWheels");
            }
        }
        else {
            // We assume that we will be using RUN_TO_POSITION mode.
            if(creator.opModeIsActive()) {
                lfMotor.setPower(lfPower);
                lrMotor.setPower(lrPower);
                rfMotor.setPower(rfPower);
                rrMotor.setPower(rrPower);
            }
            else {
                throw new EmergencyStopException("PowerTheWheels");
            }
        }
    }

    private boolean motorsShouldContinue(double distance, int[] motorDirection) {
        boolean motorsAreBusy = lfMotor.isBusy() && rfMotor.isBusy() && lrMotor.isBusy() && rrMotor.isBusy();
        boolean aMotorHasPassedPosition = false;
        if (motorsAreBusy) {
            aMotorHasPassedPosition = checkMotorPosition(lfMotor, distance * motorDirection[0])
                    || checkMotorPosition(lrMotor, distance * motorDirection[1])
                    || checkMotorPosition(rfMotor, distance * motorDirection[2])
                    || checkMotorPosition(rrMotor, distance * motorDirection[3]);
        }
        return motorsAreBusy && !aMotorHasPassedPosition;
    }

    private boolean checkMotorPosition(DcMotorEx motor, double distance) {
        //checks to see if we have gotten there yet
        if (distance > 0) {
            return motor.getCurrentPosition() > motor.getTargetPosition();
        }
        else{
            return motor.getCurrentPosition() < motor.getTargetPosition();
        }
    }

    /**
     * @return returns the distance the robot has traveled in inches
     */
    private double getMotorPosition() {
        double lfPosition = lfMotor.getCurrentPosition();
        double rfPosition = rfMotor.getCurrentPosition();
        double lrPosition = lrMotor.getCurrentPosition();
        double rrPosition = rrMotor.getCurrentPosition();

        double motorPositionAverage = (lfPosition + rfPosition + lrPosition + rrPosition) / 4;

        return motorPositionAverage / ticksPerInch;
    }

    private double powerPercentage(double delta) {
        double powerPercent = -0.000027 * Math.pow(Math.abs(delta) - 180, 2) + 1;
        if (powerPercent > 1 || powerPercent < 0) {
            System.out.println("*** WARNING! POWER PERCENT IS OUT OF RANGE: delta = " + delta + ", " +
                    "powerPercent = " + powerPercent + " ***");
        }

        return powerPercent;
    }

    private void setPIDFValues(DcMotorEx motor, int tps) {
        double D = 0;
        double F = 32767.0 / tps;
        double P = 0.1 * F;
        double I = 0.1 * P;
        System.out.printf("Max %d, P %.4f, I %.4f, D %.0f, F %.4f\n", tps, P, I, D, F);
        motor.setVelocityPIDFCoefficients(P, I, D, F);
        motor.setPositionPIDFCoefficients(drivePositionPIDF);
    }
}
