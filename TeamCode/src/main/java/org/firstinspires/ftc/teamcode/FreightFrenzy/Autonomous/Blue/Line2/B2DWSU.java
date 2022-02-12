package org.firstinspires.ftc.teamcode.FreightFrenzy.Autonomous.Blue.Line2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.AllianceColor;
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
            ewok.strafe(-3);
            ewok.drive(-22);
            ewok.duckWheelAuto(8, AllianceColor.BLUE);
            ewok.strafe(-28);
            ewok.drive(-6);
        }
    }
}