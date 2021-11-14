package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "AutonomousMain Java")
public class AutonomousMain extends LinearOpMode {

    private DcMotor lfMotor;
    private DcMotor rfMotor;
    private DcMotor lrMotor;
    private DcMotor rrMotor;

    private BNO055IMU imu;
    //private Servo sweeper;

    double leftFrontMotorVelocity;
    double leftRearMotorVelocity;
    double rightFrontMotorVelocity;
    double rightRearMotorVelocity;

    float currentHeading;
    float desiredHeading;
    double delta;
    double deltaThreshhold;

    double distanceInTicks;
    double ticksPerInch;

    double leftSpeed;
    double rightSpeed;
    double holdSpeed;
    double correctionSpeed;

    //Maximum amount of ticks/second, and also compensate for joystick direction.
    private int maximumRobotTps = -2350;
    private int maximumHalfTps = maximumRobotTps / 2;

    private float intakeStepAmount = 0.05f;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double TurnSpeed;
        boolean robotCanKeepGoing;
        double ticksPerMotorRev;
        double WheelCircumferenceInInches;
        double RobotSpeed;
        double StrafeSpeed;
        float StrafeHeading;

        lfMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        lrMotor = hardwareMap.get(DcMotor.class, "LR Motor");
        rrMotor = hardwareMap.get(DcMotor.class, "RR Motor");
        //shooter = hardwareMap.get(DcMotor.class, "RingShooter");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Initialize variables
        robotCanKeepGoing = true;
        ticksPerMotorRev = 530.3;
        // Convert 75mm wheel to inches
        WheelCircumferenceInInches = 9.6125;
        ticksPerInch = ticksPerMotorRev / WheelCircumferenceInInches;
        deltaThreshhold = 1;
        correctionSpeed = 0.1;
        RobotSpeed = 0.5;
        TurnSpeed = 0.5;
        holdSpeed = 0.1;
        StrafeSpeed = 0.5;
        StrafeHeading = currentHeading;
        initializeMotors();
        initializeIMU();
        telemetry.addData("Status", "Ready to start - v.6.0.1");
        telemetry.update();
        waitForStart();

        double currentRobotTps;
        double intakePosition = 0;

        if (opModeIsActive()) {


        }
        currentHeading = getHeading();
        delta = desiredHeading - currentHeading;
        telemetry.addData("Desired Heading", desiredHeading);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Delta", delta);
        telemetry.update();
        sleep(5000);
    }

    /**
     * Configure motor direction and modes.
     */
    private void initializeMotors() {
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void debug(String text) {
        telemetry.addData("Debug", text);
        telemetry.update();
    }

    private void initializeIMU() {
        BNO055IMU.Parameters imuParameters;

        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
    }

    /**
     * Return the robot's current heading, as an angle in degrees,
     * with 0 as the heading at the time of IMU initialization.
     * Angles are positive in a counter-clockwise direction.
     */
    private float getHeading() {
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    private void PowerTheWheels(double LFPower, double LRPower, double RFPower, double RRPower) {
        lfMotor.setPower(LFPower);
        lrMotor.setPower(LRPower);
        rfMotor.setPower(RFPower);
        rrMotor.setPower(RRPower);
    }
}
