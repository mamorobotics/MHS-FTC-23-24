package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@ com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class DriverMode extends OpMode {
    static DriveTrain driveTrain = new DriveTrain();
    static double speed = 0.4;

    static DcMotor barLift;
    static DcMotor scoop;

    @Override
    public void init() {
        driveTrain.setDriveTrain(hardwareMap, "leftFront", "leftRear", "rightFront", "rightRear");
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveTrain.reverse(DriveTrain.DriveTrainSide.LEFT);

        scoop = hardwareMap.get(DcMotorEx.class, "scoop");
        scoop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scoop.setDirection(DcMotorSimple.Direction.REVERSE);
        scoop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        barLift = hardwareMap.get(DcMotorEx.class, "barLift");
        barLift.setDirection(DcMotorSimple.Direction.REVERSE);
        barLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        driveTrain.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed + ((1 - speed) * gamepad1.left_trigger));

        //runBarLift();
        runScoop();

        runTelemetry();
    }

    private void runTelemetry()
    {
        telemetry.addData("Scoop Encoder Position: ", scoop.getCurrentPosition());
        telemetry.addData("Bar Encoder Position: ", barLift.getCurrentPosition());
        telemetry.update();
    }

    private void runScoop()
    {
        int scoopTopPos = 1565;
        int scoopBottomPos = 512;
        double attcSpeed = 0.15;

        if(gamepad2.dpad_up) {
            scoop.setTargetPosition(scoopTopPos);
            scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoop.setPower(attcSpeed);
        }
        if(gamepad2.dpad_right) {
            scoop.setTargetPosition(scoopBottomPos);
            scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoop.setPower(attcSpeed);
        }
        if(gamepad2.dpad_down) {
            scoop.setTargetPosition(0);
            scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoop.setPower(attcSpeed);
        }
    }

    private void runBarLift()
    {
        int barTopPos = 1258;
        double attcSpeed = 0.5;

        if(gamepad2.a) {
            barLift.setTargetPosition(barTopPos);
            barLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            barLift.setPower(attcSpeed);
        }
        if(gamepad2.b) {
            barLift.setTargetPosition(0);
            barLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            barLift.setPower(attcSpeed);
        }
    }
}
