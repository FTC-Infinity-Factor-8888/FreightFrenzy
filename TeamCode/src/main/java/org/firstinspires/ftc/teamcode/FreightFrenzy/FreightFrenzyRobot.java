package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.iRobot;

public class FreightFrenzyRobot implements iRobot {
    private final HardwareMap hardwareMap;
    DcMotor RFMotor;
    DcMotor RRMotor;
    DcMotor LFMotor;
    DcMotor LRMotor;
    DcMotor DWMotor;

    public FreightFrenzyRobot (LinearOpMode creator){
        this.hardwareMap = creator.hardwareMap;
    }

    private void initHardware () {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RRMotor = hardwareMap.get(DcMotor.class, "RRMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LRMotor = hardwareMap.get(DcMotor.class, "LRMotor");
        DWMotor = hardwareMap.get(DcMotor.class, "DWMotor");
    }

    public void duckWheel (double power) {
        DWMotor.setPower(power);
    }

    public void slideMotor ( double power) {

    }

    @Override
    public void drive(double y, double x) {
        if( x < 0.03) {
            RFMotor.setPower(y);
            RRMotor.setPower(y);
            LFMotor.setPower(y);
            LRMotor.setPower(y);
        }
        else if( y < 0.03) {
            RFMotor.setPower(x);
            RRMotor.setPower(x);
            LFMotor.setPower(x);
            LRMotor.setPower(x);
        }

    }


    @Override
    public void rotate(double degrees) {

    }

    @Override
    public void driveXYR(double y, double x, double r) {

    }

    @Override
    public void driveStop() {
        RFMotor.setPower(0.0);
        RRMotor.setPower(0.0);
        LFMotor.setPower(0.0);
        LRMotor.setPower(0.0);
    }

    @Override
    public void stopAll() {
        driveStop();
    }
}
