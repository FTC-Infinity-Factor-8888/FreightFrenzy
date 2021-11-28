package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utilities.iRobot;

public class AutonomousMain extends LinearOpMode {
    public void food() {
        iRobot robot = new FreightFrenzyRobot(this);
        robot.drive(0.1,0.0);
        robot.driveStop();
        robot.rotate(-0.01);
        robot.driveStop();
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
