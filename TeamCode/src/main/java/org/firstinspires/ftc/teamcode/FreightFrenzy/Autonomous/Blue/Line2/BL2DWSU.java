package org.firstinspires.ftc.teamcode.FreightFrenzy.Autonomous.Blue.Line2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.AllianceColor;
import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.FreightFrenzy.LiftPosition;


@Autonomous(name = "BL2DWSU")
public class BL2DWSU extends LinearOpMode {
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
            ewok.strafe(-24);
            ewok.rotate(-45);
            ewok.drive(-27, 0.3, 0.4);
            ewok.duckWheelAuto(10, AllianceColor.BLUE);
            ewok.rotate(0);
            ewok.strafe(-22);
            ewok.drive(-9);
            ewok.liftMotorAuto(LiftPosition.FLOOR);
        }
    }
}