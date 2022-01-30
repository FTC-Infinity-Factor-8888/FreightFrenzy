package org.firstinspires.ftc.teamcode.FreightFrenzy.Autonomous.Blue.Line1;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;

@Autonomous (name = "B1DWSU")
public class B1DWSU extends LinearOpMode {
        FreightFrenzyRobot ewok;
        //start line 1 is the closest to the warehouse for both alliances (lined up with middle barcode)
        @Override
        public void runOpMode() throws InterruptedException {
                //we have landed on a moon of Endor.
                ewok = new FreightFrenzyRobot(this);
                ewok.initHardware();

                waitForStart();
                if(opModeIsActive()){

                }
        }
}