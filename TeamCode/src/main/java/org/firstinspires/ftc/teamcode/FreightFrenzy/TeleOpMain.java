
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
        double lfSpeed; // Left Front motor speed.
        double rfSpeed; // Right Front motor speed.
        double lrSpeed; // Left Rear motor speed.
        double rrSpeed; // Right Rear motor speed.

        double maxSpeed = 0.80; // The maximum speed we want our robot to drive at.
        double normalSpeed = 0.50; // The normal speed our robot should be driving at.
        double accelerationSpeed = maxSpeed - normalSpeed; // The acceleration speed set on normal speed.
        double duckWheelSpeed = 0.4; // The speed the wheel to turn the duck carousel moves at.

        double liftSpeed = 0.3; // The speed the lift moves at.
        int maxLiftPosition = 923;  // The maximum amount of degrees the motor turns before the lift
        // reaches its maximum height.
        int minLiftPosition = 0; // The minimum amount of degrees the motor turns before the lift
        // reaches its minimum height.

        // Declaring our motors. "exp." means expansion hub.
        DcMotor RFMotor = hardwareMap.get(DcMotor.class, "RFMotor"); // Port: 2
        DcMotor RRMotor = hardwareMap.get(DcMotor.class, "RRMotor"); // Port: 3
        DcMotor LFMotor = hardwareMap.get(DcMotor.class, "LFMotor"); // Port: 0
        DcMotor LRMotor = hardwareMap.get(DcMotor.class, "LRMotor"); // Port: 1
        DcMotor DWMotor = hardwareMap.get(DcMotor.class, "DWMotor"); // Port: 1 exp.
        DcMotor LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor"); // Port: 0 exp.


        // Put initialization blocks here.

        // We reverse these motors because of the way that they are mounted.
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Declaring the buttons may quickly change.
        boolean currentLiftUp;
        boolean currentLiftDown;
        boolean duckWheelLeft;
        boolean duckWheelRight;

        // Declaring the former values of the buttons so we can tell if they changed.
        boolean priorLiftUp = false;
        boolean priorLiftDown = false;
        boolean priorDuckWheelLeft = false;
        boolean priorDuckWheelRight = false;

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                double forwardInput = gamepad1.left_stick_y; // Controls for moving back-and-forth.
                double strafeInput = gamepad1.left_stick_x; // Controls for strafing.
                double rotateInput = gamepad1.right_stick_x; // Controls for pivoting.
                // Controls to allow our robot to reach speeds up to maxSpeed.
                double accelerator = gamepad1.right_trigger;
                currentLiftUp = gamepad1.right_bumper; // Controls for moving the lift up.
                currentLiftDown = gamepad1.left_bumper; // Controls for moving the lift down.
                duckWheelLeft = gamepad1.x; // Controls for rotating the duck wheel left.
                duckWheelRight = gamepad1.b; // Controls for moving the duck wheel right.


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

                // To control the lift.
                // Checking to see whether the buttons are still pressed.
                if(currentLiftUp != priorLiftUp || currentLiftDown != priorLiftDown) {
                    // 0 = no motion, 1 = up, -1 = down
                    // Checking to see which buttons are pushed.
                    int direction = (currentLiftUp?1:0) + (currentLiftDown?-1:0);

                    // Setting the speed that the lift moves at.
                    LiftMotor.setPower(liftSpeed);
                    // The current degrees that the LiftMotor is at.
                    int currentLiftPosition = LiftMotor.getCurrentPosition();

                    // If the up button is pressed move the lift up.
                    if(direction == 1) {
                        LiftMotor.setTargetPosition(maxLiftPosition);
                    }
                    // If the down button is pressed move the lift down.
                    else if(direction == -1) {
                        LiftMotor.setTargetPosition(minLiftPosition);
                    }
                    // If no button (or both buttons [are]) is pressed, stay where you are.
                    else {
                        LiftMotor.setTargetPosition(currentLiftPosition);
                    }
                    // Move to the position as specified above.
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                // To control the duck wheel.
                // Checking to see whether the buttons are still pressed.
                if(duckWheelLeft != priorDuckWheelLeft || duckWheelRight != priorDuckWheelRight) {
                    // 1 = rotate left, -1 = rotate right, 0 = don't move.
                    // Checking to see which buttons are pressed.
                    int direction = (duckWheelLeft?1:0) + (duckWheelRight?-1:0);
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


                /*
                Because we use Mecanum wheels, we can move forward, rotate, and strafe.
                Here, we are taking into account the direction each wheel should travel at in
                order to move in the direction we want the robot to move.
                 */
                lfSpeed = ((forwardInput - strafeInput - rotateInput) * normalSpeed);
                rfSpeed = ((forwardInput + strafeInput + rotateInput) * normalSpeed);
                lrSpeed = ((forwardInput + strafeInput - rotateInput) * normalSpeed);
                rrSpeed = ((forwardInput - strafeInput + rotateInput) * normalSpeed);

                if(Math.abs(lfSpeed) + accelerationSpeed * accelerator > maxSpeed) {
                    if(Math.abs(lfSpeed) > normalSpeed) {
                        if(lfSpeed > 0) {
                            lfSpeed = normalSpeed;
                            LFMotor.setPower(lfSpeed + accelerationSpeed * accelerator);
                        }
                        if(lfSpeed < 0) {
                            lfSpeed = -normalSpeed;
                            LFMotor.setPower(lfSpeed - accelerationSpeed * accelerator);
                        }
                    }
                }
                else {
                    LFMotor.setPower(lfSpeed + accelerationSpeed * accelerator);
                }

                if(Math.abs(rfSpeed) + accelerationSpeed * accelerator > maxSpeed) {
                    if(Math.abs(rfSpeed) > normalSpeed) {
                        if(rfSpeed > 0) {
                            rfSpeed = normalSpeed;
                            RFMotor.setPower(rfSpeed + accelerationSpeed * accelerator);
                        }
                        if(rfSpeed < 0) {
                            rfSpeed = -normalSpeed;
                            RFMotor.setPower(rfSpeed - accelerationSpeed * accelerator);
                        }
                    }
                }
                else {
                    RFMotor.setPower(rfSpeed + accelerationSpeed * accelerator);
                }

                if(Math.abs(lrSpeed) + accelerationSpeed * accelerator > maxSpeed) {
                    if(Math.abs(lrSpeed) > normalSpeed) {
                        if(lrSpeed > 0) {
                            lrSpeed = normalSpeed;
                            LRMotor.setPower(lrSpeed + accelerationSpeed * accelerator);
                        }
                        if(lrSpeed < 0) {
                            lrSpeed = -normalSpeed;
                            LRMotor.setPower(lrSpeed - accelerationSpeed * accelerator);
                        }
                    }
                }
                else {
                    LRMotor.setPower(lrSpeed + accelerationSpeed * accelerator);
                }

                if(Math.abs(rrSpeed) + accelerationSpeed * accelerator > maxSpeed) {
                    if(Math.abs(rrSpeed) > normalSpeed) {
                        if(rrSpeed > 0) {
                            rrSpeed = normalSpeed;
                            RRMotor.setPower(rrSpeed + accelerationSpeed * accelerator);
                        }
                        if(rrSpeed < 0) {
                            rrSpeed = -normalSpeed;
                            RRMotor.setPower(rrSpeed - accelerationSpeed * accelerator);
                        }
                    }
                }
                else {
                    RRMotor.setPower(rrSpeed + accelerationSpeed * accelerator);
                }

                /* Here we show values on the driver hub that may be useful to know while driving
                the robot or during testing. */
                telemetry.addData("LF Motor", LFMotor.getPower());
                telemetry.addData("RF Motor", RFMotor.getPower());
                telemetry.addData("LR Motor", LRMotor.getPower());
                telemetry.addData("RR Motor", RRMotor.getPower());
                telemetry.addData("Lift", LiftMotor.getCurrentPosition());
                telemetry.addData("Accelerator", gamepad1.right_trigger);
                telemetry.update();

                /* Here we set the current button positions to the prior button position so we have
                updated data as we loop back.

                Note: This loops multiple times per millisecond.
                 */
                priorLiftUp = currentLiftUp;
                priorLiftDown = currentLiftDown;
                priorDuckWheelLeft = duckWheelLeft;
                priorDuckWheelRight = duckWheelRight;
            }
        }
    }
}
