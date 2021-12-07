
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

        double liftSpeed = 0.3;
        int maxLiftPosition = 923;
        int minLiftPosition = 0;


        DcMotor RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        DcMotor RRMotor = hardwareMap.get(DcMotor.class, "RRMotor");
        DcMotor LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        DcMotor LRMotor = hardwareMap.get(DcMotor.class, "LRMotor");
        DcMotor DuckWheelMotor = hardwareMap.get(DcMotor.class, "DWMotor");
        DcMotor LiftMotor = hardwareMap.get(DcMotor.class, "liftMotor");


        // Put initialization blocks here.
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean currentLiftUp = false;
        boolean currentLiftDown = false;
        boolean currentDuckWheel = false;

        boolean priorLiftUp = false;
        boolean priorLiftDown = false;
        boolean priorDuckWheel = false;

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                double forwardInput = gamepad1.left_stick_y;
                double strafeInput = gamepad1.left_stick_x;
                double rotateInput = gamepad1.right_stick_x;
                double accelerator = gamepad1.right_trigger;
                currentLiftUp = gamepad1.right_bumper;
                currentLiftDown = gamepad1.left_bumper;
                currentDuckWheel = gamepad1.x;

                /*
                Lift requirements:
                    The first button pressed sets the direction.
                    Holding both buttons stops the motion.
                    Releasing one of those two buttons heads in the direction of the remaining button.
                    Releasing both buttons stops the motion.

                Lift requirements (video game option, not chosen):
                    The first button to be pressed sets the direction that we are going.
                    The other button is ignored while the first button is being held down.
                    Both buttons must be released to stop the lift
                    or if it reaches its target position.
                    The lift must be stopped in order to switch direction.

                DuckWheel requirements:

                 */
                if(currentLiftUp != priorLiftUp || currentLiftDown != priorLiftDown) {
                    // 0 = no motion, 1 = up, -1 = down
                    int direction = (currentLiftUp?1:0) + (currentLiftDown?-1:0); // Ask Pranai

                    LiftMotor.setPower(liftSpeed);
                    int currentLiftPosition = LiftMotor.getCurrentPosition();

                    if(direction == 1) {
                        LiftMotor.setTargetPosition(maxLiftPosition);
                    }
                    else if(direction == -1) {
                        LiftMotor.setTargetPosition(minLiftPosition);
                    }
                    else {
                        LiftMotor.setTargetPosition(currentLiftPosition);
                    }
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if(currentDuckWheel != priorDuckWheel) {
                    double power = (currentDuckWheel?duckWheelSpeed:0); // Also ask Pranai
                    DuckWheelMotor.setPower(power);
                }

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

                if(max > 0.5) {
                    lfSpeed /= max;
                    rfSpeed /= max;
                    lrSpeed /= max;
                    rrSpeed /= max;
                }

                LFMotor.setPower(lfSpeed + lfSpeed * accelerator);
                RFMotor.setPower(rfSpeed + rfSpeed * accelerator);
                LRMotor.setPower(lrSpeed + lrSpeed * accelerator);
                RRMotor.setPower(rrSpeed + rrSpeed * accelerator);

                // TODO: Add Telemetry Data
                telemetry.addData("LF Motor", lfSpeed + lfSpeed * accelerator);
                telemetry.addData("RF Motor", rfSpeed + rfSpeed * accelerator);
                telemetry.addData("LR Motor", lrSpeed + lrSpeed * accelerator);
                telemetry.addData("RR Motor", rrSpeed + rrSpeed * accelerator);
                telemetry.addData("Lift", LiftMotor.getCurrentPosition());
                telemetry.addData("Accelerator", gamepad1.right_trigger);
                telemetry.update();

                priorLiftUp = currentLiftUp;
                priorLiftDown = currentLiftDown;
                priorDuckWheel = currentDuckWheel;
            }
        }
    }
}
