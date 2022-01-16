package org.firstinspires.ftc.teamcode.FreightFrenzy.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "LiftTest")
public class LiftTest extends LinearOpMode {

  private DcMotor LFMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int minLiftPosition;
    int maxLiftPosition;
    double liftSpeed;

    LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");

    // Put initialization blocks here.
    LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        minLiftPosition = 0;
        maxLiftPosition = 923;
        liftSpeed = 0.2;
        // Put loop blocks here.
        telemetry.addData("Version: ", "v.3.1.0");
        if (gamepad1.right_bumper) {
          while (LFMotor.getCurrentPosition() < maxLiftPosition) {
            if (LFMotor.getCurrentPosition() < maxLiftPosition) {
              LFMotor.setPower(liftSpeed);
              if (gamepad1.right_bumper == false) {
                break;
              }
            }
            if (gamepad1.b) {
              while (opModeIsActive()) {
                LFMotor.setPower(0);
                LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addData("Status", " EMERGENCY BRAKE IS ACTIVE");
                telemetry.addData("ERROR", "Please restart the OpMode");
                telemetry.update();
              }
            }
          }
        } else if (gamepad1.left_bumper) {
          while (LFMotor.getCurrentPosition() > minLiftPosition) {
            if (LFMotor.getCurrentPosition() > minLiftPosition) {
              LFMotor.setPower(-liftSpeed);
              if (gamepad1.left_bumper == false) {
                break;
              }
            } else {
              break;
            }
            if (gamepad1.b) {
              while (opModeIsActive()) {
                LFMotor.setPower(0);
                LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addData("Status", " EMERGENCY BRAKE IS ACTIVE");
                telemetry.addData("ERROR", "Please restart the OpMode");
                telemetry.update();
              }
            }
          }
        } else {
          LFMotor.setPower(0);
        }
        telemetry.addData("Lift Position", LFMotor.getCurrentPosition());
        telemetry.update();
      }
    }
  }
}
