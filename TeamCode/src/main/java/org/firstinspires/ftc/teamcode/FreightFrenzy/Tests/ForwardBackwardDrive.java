package org.firstinspires.ftc.teamcode.FreightFrenzy.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;

<<<<<<< Updated upstream
@Autonomous (name = "FBDrive")
=======
@Autonomous (name = "WarehouseLine1")
>>>>>>> Stashed changes
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
            ewok.drive(30);
            ewok.drive(-30);
        }
    }
}