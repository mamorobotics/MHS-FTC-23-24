package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {
    public enum DriveTrainSide {
        RIGHT, LEFT
    }

    private static DcMotor FL, BL, FR, BR;

    public void setDriveTrain(HardwareMap hardwareMap, String frontLeft, String backLeft, String frontRight, String backRight) {
        FL = hardwareMap.get(DcMotor.class, frontLeft);
        BL = hardwareMap.get(DcMotor.class, backLeft);
        FR = hardwareMap.get(DcMotor.class, frontRight);
        BR = hardwareMap.get(DcMotor.class, backRight);
    }
    
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        FL.setZeroPowerBehavior(zeroPowerBehavior);
        FR.setZeroPowerBehavior(zeroPowerBehavior);
        BL.setZeroPowerBehavior(zeroPowerBehavior);
        BR.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        FL.setMode(runMode);
        FR.setMode(runMode);
        BL.setMode(runMode);
        BR.setMode(runMode);
    }

    public void reverse(DriveTrainSide side) {
        switch (side){
            case RIGHT:
                FR.setDirection(DcMotor.Direction.REVERSE);
                BR.setDirection(DcMotor.Direction.REVERSE);
                break;
            case LEFT:
                FL.setDirection(DcMotor.Direction.REVERSE);
                BL.setDirection(DcMotor.Direction.REVERSE);
                break;
        }
    }

    public void move(double x, double y, double r, double speed){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        double frontLeftPower = (y + x + r) * speed / denominator;
        double backLeftPower = (y - x + r) * speed / denominator;
        double frontRightPower = (y - x - r) * speed / denominator;
        double backRightPower = (y + x - r) * speed / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);
    }
}
