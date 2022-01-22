package org.firstinspires.ftc.teamcode.FreightFrenzy.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.DWDirection;
import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.FreightFrenzy.LiftPosition;
import org.firstinspires.ftc.teamcode.FreightFrenzy.SpintakeDirection;

@Autonomous (name = "DancePartyTest")
public class DancePartyTest extends LinearOpMode {
    FreightFrenzyRobot ewok;
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();

        waitForStart();        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();

        waitForStart();
        if(opModeIsActive()){
            ewok.liftMotorAuto(LiftPosition.DRIVE);
            ewok.drive(15);
            ewok.drive(-15);
            ewok.rotate(100);
            ewok.rotate(-100);
            ewok.duckWheelAuto(10, DWDirection.BLUE);
            ewok.duckWheelAuto(10, DWDirection.RED);
            ewok.duckWheelAuto(5, DWDirection.BLUE);
            ewok.duckWheelAuto(5, DWDirection.RED);
            ewok.spinTakeAuto(5, SpintakeDirection.INTAKE);
            ewok.spinTakeAuto(5, SpintakeDirection.OUTTAKE);
            ewok.strafe(15);
            ewok.strafe(-15);
            ewok.liftMotorAuto(LiftPosition.FIRST);
            ewok.liftMotorAuto(LiftPosition.THIRD);
            ewok.liftMotorAuto(LiftPosition.SECOND);
            ewok.liftMotorAuto(LiftPosition.CAPPING);
            ewok.liftMotorAuto(LiftPosition.FLOOR);
            ewok.stopAll();
        }
    }
}