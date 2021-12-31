package org.firstinspires.ftc.teamcode.FreightFrenzy;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utilities.EmergencyStopException;

@Autonomous (name = "Test-2.1")
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
            ewok.duckWheelMotor(1);
            ewok.armsStop();
            ewok.rotate(90);
            ewok.rotate(-100);
            ewok.drive(-10);
            ewok.strafe(15);
            ewok.stopAll();
            ewok.driveStop();
            ewok.armsStop();
            throw new EmergencyStopException("The dance party is done.");
        }

        if(opModeIsActive()){
            ewok.drive(15);
            ewok.duckWheelMotor(1);
            ewok.armsStop();
            ewok.rotate(90);
            ewok.rotate(-100);
            ewok.drive(-10);
            ewok.strafe(15);
            ewok.stopAll();
            ewok.driveStop();
            ewok.armsStop();
            throw new EmergencyStopException("The dance party is done.");
        }
    }
}