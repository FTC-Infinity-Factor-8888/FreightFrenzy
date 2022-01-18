package org.firstinspires.ftc.teamcode.FreightFrenzy.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.FreightFrenzy.LiftPosition;
import org.firstinspires.ftc.teamcode.Utilities.EmergencyStopException;

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
            ewok.duckWheelAutonomous(10);
            ewok.duckWheelAutonomous(-10);
            ewok.duckWheelAutonomous(5);
            ewok.duckWheelAutonomous(-5);
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