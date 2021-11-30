
package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode {

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double lfSpeed;
        double rfSpeed;
        double lrSpeed;
        double rrSpeed;

        double maxSpeed = 0.80;
        double normalSpeed = 0.50;
        double duckWheelSpeed = 0.25;
        double liftSpeed = 0.05;

        DcMotor RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        DcMotor RRMotor = hardwareMap.get(DcMotor.class, "RRMotor");
        DcMotor LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        DcMotor LRMotor = hardwareMap.get(DcMotor.class, "LRMotor");
        DcMotor DuckWheelMotor = hardwareMap.get(DcMotor.class, "DWMotor");
        DcMotor LiftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        // Put initialization blocks here.
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                double forwardInput = gamepad1.left_stick_y;
                double strafeInput = gamepad1.left_stick_x;
                double rotateInput = gamepad1.right_stick_x;

                double accelerator = gamepad1.right_trigger;
                if (accelerator > maxSpeed) {
                    accelerator = maxSpeed;
                }


                lfSpeed = (forwardInput + strafeInput - rotateInput * normalSpeed);
                rfSpeed = (forwardInput - strafeInput + rotateInput * normalSpeed);
                lrSpeed = (forwardInput - strafeInput - rotateInput * normalSpeed);
                rrSpeed = (forwardInput + strafeInput + rotateInput * normalSpeed);

                double leftMax = Math.max(Math.abs(lfSpeed), Math.abs(lrSpeed));
                double rightMax = Math.max(Math.abs(rfSpeed), Math.abs(rrSpeed));
                double max = Math.max (leftMax, rightMax);

                if(max > 1.0) {
                    lfSpeed /= max;
                    rfSpeed /= max;
                    lrSpeed /= max;
                    rrSpeed /= max;
                }

                if (gamepad1.x) { // Might change to y
                    DuckWheelMotor.setPower(1 * duckWheelSpeed);
                }

                int liftPosition0 = 1; // 1 is a mockup value
                int liftPosition1 = 2; // 2 is a mockup value
                int liftPosition2 = 3; // 3 is a mockup value
                int liftPosition3 = 4; // 4 is a mockup value
                int currentLiftPosition = liftPosition0;
                int desiredLiftPosition = 0;
                if (gamepad1.right_bumper) {
                    if (currentLiftPosition != liftPosition3) {
                        desiredLiftPosition = liftPosition1;
                        currentLiftPosition++;
                    }
                else if (gamepad1.left_bumper) {
                        if (currentLiftPosition != liftPosition0) {

                        }
                    }
                }

                if (currentLiftPosition != 0) {
                    LiftMotor.setTargetPosition(desiredLiftPosition);
                }

                LFMotor.setPower(lfSpeed + lfSpeed * accelerator);
                RFMotor.setPower(rfSpeed + rfSpeed * accelerator);
                LRMotor.setPower(lrSpeed + lrSpeed * accelerator);
                RRMotor.setPower(rrSpeed + rrSpeed * accelerator);

                // TODO: Add Telemetry Data
                telemetry.addData("LF Motor", lfSpeed);
                telemetry.addData("RF Motor", rfSpeed);
                telemetry.addData("LR Motor", lrSpeed);
                telemetry.addData("RR Motor", rrSpeed);
                telemetry.addData("Lift", LiftMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
