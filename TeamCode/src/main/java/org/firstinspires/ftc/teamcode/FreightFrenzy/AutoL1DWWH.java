package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto - L1 - DW > WH")
public class AutoL1DWWH extends LinearOpMode {
    FreightFrenzyRobot robot;
    //start line 1 is the closest to the warehouse for both alliances (lined up with middle barcode)
    //start line 2 is the closest to the duck wheel for both alliances (lined up with the middle barcode)
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new FreightFrenzyRobot(this); //saying which program to run, and how to get to it.
        robot.initHardware(); //initialize the hardware (tells the computer where everything is)

        waitForStart(); //waiting for us to start the program
        if(opModeIsActive()){ //checks to see if we have pressed play
            robot.drive(-27); //drives 27 inches backwards
            robot.duckWheelAutonomous(5); //spins the wheel 5 times
            robot.strafe(2); //strafes right 2 inches
            robot.drive(160); //drives 160 inches forwards
        }
    }
}

