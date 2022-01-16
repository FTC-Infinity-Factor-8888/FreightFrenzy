package org.firstinspires.ftc.teamcode.FreightFrenzy.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;

@Autonomous (name = "FBDrive")
public class ForwardBackwardDrive extends LinearOpMode {
    FreightFrenzyRobot ewok;
    //start line 1 is the closest to the warehouse for both alliances (lined up with middle barcode)
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();

        waitForStart();
        if(opModeIsActive()){
            System.out.println("ROBOT IS FORWARDING");
            ewok.drive(30);
            System.out.println("ROBOT HAS FINISHED FORWARDING");
            System.out.println("ROBOT IS STOPPING");
            ewok.driveStop();
            System.out.println("ROBOT HAS STOPPED");
            System.out.println("ROBOT IS REVERSING");
            ewok.drive(-30);
            System.out.println("ROBOT HAS FINISHED REVERSING");
            System.out.println("SUCCESS!");
        }
    }
}