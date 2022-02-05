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
            ewok.drive(8);
            System.out.println("DEBUG: Finished drive(8)");
            ewok.rotate(-50);
            System.out.println("DEBUG: Finished rotate(-50)");
            ewok.liftMotorAuto(LiftPosition.THIRD);
            System.out.println("DEBUG: Finished lift(THIRD)");
            ewok.drive(13.5);
            System.out.println("DEBUG: Finished drive(13.5)");
//            ewok.spinTakeAuto(3, SpintakeDirection.OUTTAKE);
//            System.out.println("DEBUG: Finished outtake");
            ewok.drive(-13.5);
            System.out.println("DEBUG: Finished drive(-13.5)");
            ewok.rotate(90);
            System.out.println("DEBUG: Finished rotate(90)");
            ewok.strafe(13);
            System.out.println("DEBUG: Finished strafe(13)");
            ewok.drive(30);
            System.out.println("DEBUG: Finished drive(30)");
            ewok.done();
        }
    }
}