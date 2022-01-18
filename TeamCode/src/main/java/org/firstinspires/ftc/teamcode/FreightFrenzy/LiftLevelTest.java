package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "LiftLevelTest")
public class LiftLevelTest extends LinearOpMode {
    FreightFrenzyRobot ewok;
    //start line 1 is the closest to the warehouse for both alliances (lined up with middle barcode)
    //start line 2 is the closest to the duck wheel for both alliances (lined up with the middle barcode)
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();

        waitForStart();
        if(opModeIsActive()){
            ewok.liftMotorAuto(LiftPosition.DRIVE);
            sleep(3000);
            if (!opModeIsActive()){
                return;
            }
            ewok.liftMotorAuto(LiftPosition.FIRST);
            sleep(3000);
            if (!opModeIsActive()){
                return;
            }
            ewok.liftMotorAuto(LiftPosition.SECOND);
            sleep(3000);
            if (!opModeIsActive()){
                return;
            }
            ewok.liftMotorAuto(LiftPosition.THIRD);
            sleep(3000);
            if (!opModeIsActive()){
                return;
            }
            ewok.liftMotorAuto(LiftPosition.CAPPING);
            sleep(3000);
            if (!opModeIsActive()){
                return;
            }
            ewok.liftMotorAuto(LiftPosition.FLOOR);
            sleep(3000);
            if (!opModeIsActive()){
                return;
            }
        }
    }
}