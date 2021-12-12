package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    DcMotor RFMotor;
    DcMotor RRMotor;
    DcMotor LFMotor;
    DcMotor LRMotor;
    DcMotor DWMotor;
    private BNO055IMU imu;

    private double wheelCircumferenceInInches = (96/25.4) * Math.PI;
  //  private int maximumTicksPerSecond = 2800;
    private double ticksPerMotorRevolution = 530.3;
    private double ticksPerInch = ticksPerMotorRevolution / wheelCircumferenceInInches;

    private double turnSpeed = 0.1;
    private double delta;
    private double deltaThreshold = 1;
    private double turnDeltaThreshold = 5;

   // todo set x!
   //  double x = Gamepad.
    public FreightFrenzyRobot (LinearOpMode creator){
        this.hardwareMap = creator.hardwareMap;
    }

    private double getIMUHeading () {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    private void initHardware () {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RRMotor = hardwareMap.get(DcMotor.class, "RRMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LRMotor = hardwareMap.get(DcMotor.class, "LRMotor");
        DWMotor = hardwareMap.get(DcMotor.class, "DWMotor");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void duckWheel (double power) {
        DWMotor.setPower(power);
    }

    public void liftMotor(double power) {

    }

    @Override
    public void drive(double y, double x) {

    }

    private double powerPercentage(double delta) {
        double powerPercent = -0.000027 * Math.pow(Math.abs(delta) - 180, 2) + 1;
        if (powerPercent > 1 || powerPercent < 0) {
            System.out.println("*** WARNING! POWER PERCENT IS OUT OF RANGE: delta = " + delta + ", " +
                    "powerPercent = " + powerPercent + " ***");
        }

        return powerPercent;
    }

    public void rotate(double desiredHeading) {
        double minTurnSpeed = 0.1;
        double currentHeading = getIMUHeading();
        delta = normalizeHeading(desiredHeading - currentHeading);
        double priorDelta = delta;
        int ringingCount = 0;
        while (SithApprenticeAutonomous.opModeIsActive() && Math.abs(delta) > turnDeltaThreshold && ringingCount <= 3) {
            currentHeading = getIMUHeading();
            delta = normalizeHeading(desiredHeading - currentHeading);
            double deltaPercentage =  powerPercentage(delta);
            double currentTurnSpeed = turnSpeed * deltaPercentage + minTurnSpeed;
            if (delta < 0) {
                currentTurnSpeed = -currentTurnSpeed;
            }
            leftSpeed = -currentTurnSpeed;
            rightSpeed = currentTurnSpeed;
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            telemetryDashboard("Turn(" + (int) desiredHeading + ")");
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

    @Override
    public void driveXYR(double y, double x, double r) {

    }

    @Override
    public void driveStop() {
        RFMotor.setPower(0.0);
        RRMotor.setPower(0.0);
        LFMotor.setPower(0.0);
        LRMotor.setPower(0.0);
    }

    @Override
    public void stopAll() {
        driveStop();
    }

    @Override

    public double normalizeHeading(double heading) {
        while (heading >= 180.0 || heading < -180.0) {
            if (heading >= 180.0) {
                heading -= 360.0;
            }
            else if (heading < -180.0) {
                heading += 360.0;
                //Helen is mad at the computer
            }
        }
        return heading;
    }
}
