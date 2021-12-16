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
    private DcMotor RFMotor;
    private DcMotor RRMotor;
    private DcMotor LFMotor;
    private DcMotor LRMotor;
    private DcMotor DWMotor;
    private DcMotor LiftMotor;
    private BNO055IMU imu;


    private double lfSpeed; // Left Front motor speed.
    private double rfSpeed; // Right Front motor speed.
    private double lrSpeed; // Left Rear motor speed.
    private double rrSpeed; // Right Rear motor speed.

    private double maxSpeed = 0.80; // The maximum speed we want our robot to drive at.
    private double normalSpeed = 0.50; // The normal speed our robot should be driving at.
    private double accelerationSpeed = maxSpeed - normalSpeed; // The acceleration speed set on normal speed.

    private final double wheelCircumferenceInInches = (96 / 25.4) * Math.PI;
    //  private int maximumTicksPerSecond = 2800;
    private final double ticksPerMotorRevolution = 530.3;
    private final double ticksPerInch = ticksPerMotorRevolution / wheelCircumferenceInInches;

    private final int maxLiftPosition = 824;  // The maximum amount of degrees the motor turns before the lift
    // reaches its maximum height.
    private final int minLiftPosition = 0; // The minimum amount of degrees the motor turns before the lift
    // reaches its minimum height.

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
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RRMotor = hardwareMap.get(DcMotor.class, "RRMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LRMotor = hardwareMap.get(DcMotor.class, "LRMotor");
        DWMotor = hardwareMap.get(DcMotor.class, "DWMotor");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor"); // Port: 0 exp.d

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void duckWheel(double power) {
        DWMotor.setPower(power);
    }

    public void liftMotor(double power, int direction) {
        if (direction == 0) {
            LiftMotor.setPower(0);
        }
        else {

            // Move to the position as specified above.
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Setting the speed that the lift moves at.

            // If the up button is pressed move the lift up.
            if (direction == 1) {
                LiftMotor.setTargetPosition(maxLiftPosition);
            }
            // If the down button is pressed move the lift down.
            else if (direction == -1) {
                LiftMotor.setTargetPosition(minLiftPosition);
            }
            LiftMotor.setPower(power);
        }

    }

    @Override
    public void drive(double y, double x) {

    }

    @Override
    public void driveXYRB(double y, double x, double r, double b) {
        /*
                Because we use Mecanum wheels, we can move forward, rotate, and strafe.
                Here, we are taking into account the direction each wheel should travel at in
                order to move in the direction we want the robot to move.
                 */
        lfSpeed = ((y - x - r) * normalSpeed);
        rfSpeed = ((y + x + r) * normalSpeed);
        lrSpeed = ((y + x - r) * normalSpeed);
        rrSpeed = ((y - x + r) * normalSpeed);

        if (Math.abs(lfSpeed) + accelerationSpeed * b > maxSpeed) {
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

        if (Math.abs(rfSpeed) + accelerationSpeed * b > maxSpeed) {
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

        if (Math.abs(lrSpeed) + accelerationSpeed * b > maxSpeed) {
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

        if (Math.abs(rrSpeed) + accelerationSpeed * b > maxSpeed) {
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

    @Override
    public void rotate(double desiredHeading) {
    }
}
