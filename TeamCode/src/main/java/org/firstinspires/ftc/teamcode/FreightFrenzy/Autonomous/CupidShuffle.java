package org.firstinspires.ftc.teamcode.FreightFrenzy.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.FreightFrenzy.LiftPosition;
import org.firstinspires.ftc.teamcode.FreightFrenzy.SpintakeDirection;

@Autonomous (name = "CupidShuffle")
public class CupidShuffle extends LinearOpMode {
    FreightFrenzyRobot dance;
    //start in middle of area
    @Override
    public void runOpMode() throws InterruptedException {
        dance = new FreightFrenzyRobot(this);
        dance.initHardware();
        System.out.println("Let's dance!");

        waitForStart();
        if(opModeIsActive()) {
            int shuffleDist = 10;//begin dance
            dance.strafe(-1*shuffleDist); //to the right

            dance.strafe(-1*shuffleDist); //to the right

            dance.strafe(-1*shuffleDist); //to the right

            dance.strafe(-1*shuffleDist); //to the right


            dance.strafe(shuffleDist); //to the left
            dance.strafe(shuffleDist); //to the left

            dance.strafe(shuffleDist); //to the left

            dance.strafe(shuffleDist); //to the left


            
            dance.liftMotorAuto(LiftPosition.FIRST);//now kick
            dance.spinTakeAuto(1, SpintakeDirection.OUTTAKE);
            dance.liftMotorAuto(LiftPosition.DRIVE);//now kick


            dance.drive(10);
            dance.strafe(10);
            dance.drive(-10);
            dance.strafe(-10);
            dance.drive(10);
            dance.strafe(10);
            dance.drive(-10);
            dance.strafe(-10);



        }
    }
}
