package org.firstinspires.ftc.teamcode.FreightFrenzy.Autonomous.Red.Line1;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.DWDirection;
import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.FreightFrenzy.LiftPosition;

@Autonomous (name = "R1ASWH")
public class R1ASWH extends LinearOpMode {
    FreightFrenzyRobot ewok;
    //start line 1 is the closest to the warehouse for both alliances (lined up with middle barcode)
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();
        ewok.liftMotorAuto(LiftPosition.DRIVE);
        System.out.println("Bucket moved to DRIVE Position");

        waitForStart();
        if(opModeIsActive()) {

        }
    }
}