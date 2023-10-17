package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@ com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class DriverMode extends OpMode {
    static DriveTrain driveTrain;
    static double speed = 0.65;

    @Override
    public void init() {
        driveTrain.setDriveTrain(hardwareMap, "leftFront", "leftBack", "rightFront", "rightBack");
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.reverse(DriveTrain.DriveTrainSide.LEFT);
    }

    @Override
    public void loop() {
        driveTrain.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
    }
}
