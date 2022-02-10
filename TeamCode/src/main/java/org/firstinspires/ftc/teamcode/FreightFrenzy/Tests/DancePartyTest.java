package org.firstinspires.ftc.teamcode.FreightFrenzy.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.AllianceColor;
import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.FreightFrenzy.LiftPosition;
import org.firstinspires.ftc.teamcode.FreightFrenzy.SpintakeDirection;

@Autonomous (name = "DancePartyTest")
public class DancePartyTest extends LinearOpMode {
    FreightFrenzyRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new FreightFrenzyRobot(this);
        robot.initHardware();


        waitForStart();
        if(opModeIsActive()){
            /*DW stands for duck wheel, the wheel that spins the turntable, Spintake is the intake
            and outtake system on the arm bucket, lift is the elevator lift arm, and robot calls
            the interface implementation */
            robot.liftMotorAuto(LiftPosition.DRIVE);
            robot.drive(15);
            robot.drive(-15);
            robot.rotate(100);
            robot.rotate(-100);
            robot.duckWheelAuto(10, AllianceColor.BLUE);
            robot.duckWheelAuto(10, AllianceColor.RED);
            robot.duckWheelAuto(5, AllianceColor.BLUE);
            robot.duckWheelAuto(5, AllianceColor.RED);
            robot.spinTakeAuto(5, SpintakeDirection.INTAKE);
            robot.spinTakeAuto(5, SpintakeDirection.OUTTAKE);
            robot.strafe(15);
            robot.strafe(-15);
            robot.liftMotorAuto(LiftPosition.FIRST);
            robot.liftMotorAuto(LiftPosition.THIRD);
            robot.liftMotorAuto(LiftPosition.SECOND);
            robot.liftMotorAuto(LiftPosition.CAPPING);
            robot.liftMotorAuto(LiftPosition.FLOOR);
            robot.stopAll();
        }
    }
}