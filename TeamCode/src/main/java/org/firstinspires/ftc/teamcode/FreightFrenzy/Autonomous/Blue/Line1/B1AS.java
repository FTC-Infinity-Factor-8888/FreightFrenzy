package org.firstinspires.ftc.teamcode.FreightFrenzy.Autonomous.Blue.Line1;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.FreightFrenzy.LiftPosition;
import org.firstinspires.ftc.teamcode.FreightFrenzy.SpintakeDirection;

@Autonomous (name = "B1AS")
public class B1AS extends LinearOpMode {
    FreightFrenzyRobot ewok;
    //start line 1 is the closest to the warehouse for both alliances (lined up with middle barcode)
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();
        System.out.println("Bucket moved to DRIVE Position");

        waitForStart();
        if(opModeIsActive()) {
            ewok.fllDrive(8);
            ewok.rotate(-50);
            ewok.liftMotorAuto(LiftPosition.THIRD);
            ewok.drive(13.5);
            ewok.spinTakeAuto(3, SpintakeDirection.OUTTAKE);
        }
    }
}