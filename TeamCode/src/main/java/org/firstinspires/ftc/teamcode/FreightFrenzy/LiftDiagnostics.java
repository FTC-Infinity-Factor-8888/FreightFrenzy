
package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LiftDiagnostics")
public class LiftDiagnostics extends LinearOpMode {
    FreightFrenzyRobot food;

     //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {
        // Put initialization blocks here.

        // Declaring the buttons that may quickly change.
        boolean currentLiftUp;
        boolean currentLiftDown;
        boolean liftOverride;

        // Declaring the former values of the buttons, so we can tell if they changed.
        boolean priorLiftUp = false;
        boolean priorLiftDown = false;
        boolean liftPositionSave = false;

        //yoda has arrived.
        food = new FreightFrenzyRobot(this);
        food.initHardware();

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                currentLiftUp = gamepad2.right_bumper; // Controls for moving the vertical-lift up.
                currentLiftDown = gamepad2.left_bumper; // Controls for moving the lift down.
                liftPositionSave = gamepad2.x; // Controls for rotating the duck wheel anti-clockwise.
                liftOverride = gamepad2.dpad_up;
                /*
                Lift requirements:
                    The first button pressed sets the direction.
                    Holding both buttons stops the motion.
                    Releasing one of those two buttons heads in the direction of the remaining button.
                    Releasing both buttons stops the motion.

                    pressing x will save the lift position for autonomous

                Lift requirements (video game option, not chosen):
                    The first button to be pressed sets the direction that we are going.
                    The other button is ignored while the first button is being held down.
                    Both buttons must be released to stop the lift
                    or if it reaches its target position.
                    The lift must be stopped in order to switch direction.
                */
                // To control the lift.
                // Checking to see whether the buttons are still pressed.
                if(currentLiftUp != priorLiftUp || currentLiftDown != priorLiftDown) {
                    // 0 = no motion, 1 = up, -1 = down
                    // Checking to see which buttons are pushed.
                    int direction = (currentLiftUp?1:0) + (currentLiftDown?-1:0);
                    if(liftOverride) {
                        food.liftMotorOverride(direction);
                    }
                    else {
                        food.liftMotor(direction);
                        telemetry.addData("Lift:", food.getCurrentLiftPosition());

                        if(liftPositionSave) {
                            System.out.println("SAVE POSITION");
                            telemetry.addData("Lift:", food.getCurrentLiftPosition());
                        }
                    }
                }

                telemetry.update();

                /* Here we set the current button positions to the prior button position, so we have
                updated data as we loop back.

                Note: This loops multiple times per millisecond.
                 */
                priorLiftUp = currentLiftUp;
                priorLiftDown = currentLiftDown;
            }
        }
    }
}
