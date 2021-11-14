
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
        double rSpeed;
        double lSpeed;
        double maxSpeed = 0.80;
        double normalSpeed = 0.50;
        double duckWheelSpeed = 0.25;

        DcMotor RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        DcMotor RRMotor = hardwareMap.get(DcMotor.class, "RRMotor");
        DcMotor LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        DcMotor LRMotor = hardwareMap.get(DcMotor.class, "LRMotor");
        DcMotor DuckWheelMotor = hardwareMap.get(DcMotor.class, "DWMotor");

        // Put initialization blocks here.
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (gamepad1.right_trigger >= 0.5) {
                    rSpeed = gamepad1.right_stick_y * maxSpeed;
                    lSpeed = gamepad1.left_stick_y * maxSpeed;
                }
                else {
                    rSpeed = gamepad1.right_stick_y * normalSpeed;
                    lSpeed = gamepad1.left_stick_y * normalSpeed;
                }
                if (gamepad1.y) {
                    DuckWheelMotor.setPower(1 * duckWheelSpeed);
                }
                if (gamepad1.right_bumper) {
                    if (gamepad1.right_trigger >= 0.5) {
                        LFMotor.setPower(-1 * maxSpeed);
                        LRMotor.setPower(1 * maxSpeed);
                        RFMotor.setPower(1 * maxSpeed);
                        RRMotor.setPower(-1 * maxSpeed);
                    }
                    else {
                        LFMotor.setPower(-1 * normalSpeed);
                        LRMotor.setPower(1 * normalSpeed);
                        RFMotor.setPower(1 * normalSpeed);
                        RRMotor.setPower(-1 * normalSpeed);
                    }
                }
                else if (gamepad1.left_bumper) {
                    if (gamepad1.right_trigger >= 0.5) {
                        LFMotor.setPower(1 * maxSpeed);
                        LRMotor.setPower(-1 * maxSpeed);
                        RFMotor.setPower(-1 * maxSpeed);
                        RRMotor.setPower(1 * maxSpeed);
                    }
                    else {
                        LFMotor.setPower(1 * normalSpeed);
                        LRMotor.setPower(-1 * normalSpeed);
                        RFMotor.setPower(-1 * normalSpeed);
                        RRMotor.setPower(1 * normalSpeed);
                    }
                }
                else {
                    LFMotor.setPower(lSpeed);
                    LRMotor.setPower(lSpeed);
                    RFMotor.setPower(rSpeed);
                    RRMotor.setPower(rSpeed);
                }
                telemetry.addData("Left stick", lSpeed);
                telemetry.addData("Right stick", rSpeed);
                telemetry.update();
            }
        }
    }
}