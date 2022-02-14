package org.firstinspires.ftc.teamcode.FreightFrenzy.Autonomous.Blue.Line1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.FreightFrenzy.LiftPosition;
import org.firstinspires.ftc.teamcode.FreightFrenzy.SpintakeDirection;


// @Autonomous(name = "BL1ASWH")
public class BL1ASWH extends LinearOpMode {
    FreightFrenzyRobot ewok;

    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();
        System.out.println("Bucket moved to DRIVE Position");

        waitForStart();
        if (opModeIsActive()) {
            ewok.liftMotorAuto(LiftPosition.DRIVE);
            ewok.strafe(-41);
            ewok.rotate(180);
            ewok.liftMotorAuto(LiftPosition.THIRD);
            ewok.fllDrive(7);
            ewok.spinTakeAuto(2, SpintakeDirection.OUTTAKE);
            ewok.liftMotorAuto(LiftPosition.DRIVE);
            ewok.strafe(-41.5);
            ewok.drive(-38);
            ewok.liftMotorAuto(LiftPosition.FLOOR);
        }
    }
}