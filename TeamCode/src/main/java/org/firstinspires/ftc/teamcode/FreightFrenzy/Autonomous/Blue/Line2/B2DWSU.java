package org.firstinspires.ftc.teamcode.FreightFrenzy.Autonomous.Blue.Line2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.DWDirection;
import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;


@Autonomous(name = "B2DWSU")
public class B2DWSU extends LinearOpMode {
    FreightFrenzyRobot ewok;

    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();
        System.out.println("Bucket moved to DRIVE Position");

        waitForStart();
        if (opModeIsActive()) {
            ewok.drive(-26);
            ewok.duckWheelAuto(4, DWDirection.BLUE);
            ewok.strafe(-25);
            ewok.drive(-8);
        }
    }
}