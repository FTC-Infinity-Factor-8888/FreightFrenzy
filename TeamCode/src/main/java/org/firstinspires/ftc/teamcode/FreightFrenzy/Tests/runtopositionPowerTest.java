package org.firstinspires.ftc.teamcode.FreightFrenzy.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class runtopositionPowerTest extends LinearOpMode {
    DcMotor DuckWheelMotor;

    @Override
    public void runOpMode() {
        DuckWheelMotor = hardwareMap.get(DcMotor.class, "DWMotor");
        DuckWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DuckWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        if(opModeIsActive()) {
            DuckWheelMotor.setTargetPosition(1000);
            DuckWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DuckWheelMotor.setPower(1);

            while(opModeIsActive() && DuckWheelMotor.isBusy()) {
                idle();
            }
        }

        if(opModeIsActive()) {
            DuckWheelMotor.setTargetPosition(0);
            while(opModeIsActive() && DuckWheelMotor.isBusy()) {
                idle();
            }
        }


    }

}
