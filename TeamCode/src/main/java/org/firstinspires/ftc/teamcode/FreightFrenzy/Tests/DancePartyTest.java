package org.firstinspires.ftc.teamcode.FreightFrenzy.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.Utilities.EmergencyStopException;

@Autonomous (name = "DancePartyTest")
public class DancePartyTest extends LinearOpMode {
    FreightFrenzyRobot ewok;
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();

        waitForStart();        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();

        waitForStart();
        if(opModeIsActive()){
            ewok.drive(15);
            ewok.drive(-15);
            ewok.rotate(100);
            ewok.rotate(-100);
            ewok.duckWheelMotor(1);
            ewok.duckWheelMotor(0);
            ewok.duckWheelMotor(-1);
            ewok.duckWheelMotor(0);
            ewok.strafe(15);
            ewok.strafe(-15);
            ewok.liftMotor(1);
            ewok.liftMotor(-1);
            ewok.stopAll();
        }
    }
}