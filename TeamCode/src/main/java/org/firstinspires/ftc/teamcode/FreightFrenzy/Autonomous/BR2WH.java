package org.firstinspires.ftc.teamcode.FreightFrenzy.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;

@Autonomous (name = "BR2WH")
public class BR2WH extends LinearOpMode {
    FreightFrenzyRobot ewok;
    //start line 1 is the closest to the warehouse for both alliances (lined up with middle barcode)
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();

        waitForStart();
        if(opModeIsActive()){
            // In seconds
            int countDown = 16; // Decrement happens before display, so start one higher
            while (countDown-- > 0 && opModeIsActive()) {
                telemetry.addData("Sleeping before moving to Warehouse", countDown);
                telemetry.update();
                sleep(1000);
            }

            // Check to see if we are still active
            if (opModeIsActive()) {
                ewok.drive(75);
                ewok.done();
            }
        }
    }
}
