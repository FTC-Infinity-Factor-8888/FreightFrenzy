package org.firstinspires.ftc.teamcode.FreightFrenzy.Autonomous.Red.Line1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.FreightFrenzy.LiftPosition;
import org.firstinspires.ftc.teamcode.FreightFrenzy.SpintakeDirection;


@Autonomous(name = "RP1ASWH")
public class RP1ASWH extends LinearOpMode {
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
            ewok.drive(21);
            ewok.liftMotorAuto(LiftPosition.THIRD);
            ewok.strafe(21);
            ewok.spinTakeAuto(2, SpintakeDirection.OUTTAKE);
            ewok.strafe(-21);
            ewok.liftMotorAuto(LiftPosition.DRIVE);
            ewok.rotate(-90);
            ewok.strafe(-23);
            ewok.drive(27);
            ewok.liftMotorAuto(LiftPosition.FLOOR);
        }
    }
}