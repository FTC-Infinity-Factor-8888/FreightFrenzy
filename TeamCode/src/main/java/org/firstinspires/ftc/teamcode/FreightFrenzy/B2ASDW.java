package org.firstinspires.ftc.teamcode.FreightFrenzy;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "B2ASDW")
public class B2ASDW extends LinearOpMode {
    FreightFrenzyRobot ewok;
    //start line 1 is the closest to the warehouse for both alliances (lined up with middle barcode)
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new FreightFrenzyRobot(this);
        ewok.initHardware();
        ewok.liftMotorAuto(LiftPosition.DRIVE);
        System.out.println("Bucket moved to DRIVE Position");

        waitForStart();
        if(opModeIsActive()) {
            ewok.drive(6.1);
            System.out.println("DEBUG: Robot drove 6.1 inches");
            ewok.rotate(-45);
            System.out.println("DEBUG: Robot turned -45");
            ewok.liftMotorAuto(LiftPosition.THIRD);
            System.out.println("DEBUG: Lift moved to third");
            ewok.strafe(10);
            System.out.println("DEBUG: Robot strafed 10");
            ewok.drive(-20);
            System.out.println("DEBUG: Robot drove -20");
            ewok.duckWheelAuto(10,DWDirection.BLUE);
            System.out.println("DEBUG: Robot spun the DW");
        }
    }
}