package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@ com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class DriverMode extends OpMode {
    static DriveTrain driveTrain = new DriveTrain();
    static double speed = 0.5;

    static double scoopSpeed = 0.3;

    static boolean gate = false;

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

        barLift = hardwareMap.get(DcMotorEx.class, "barLift");
        barLift.setDirection(DcMotorSimple.Direction.REVERSE);
        barLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        driveTrain.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x * 1.2, speed);

        runBarLift();
        runScoop();
        runGateScoop();

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

        if(gamepad2.dpad_up) {
            scoop.setTargetPosition(scoopTopPos);
            scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoop.setPower(scoopSpeed);
        }
        if(gamepad2.dpad_right) {
            scoop.setTargetPosition(scoopBottomPos);
            scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoop.setPower(scoopSpeed - 0.1);
        }
        if(gamepad2.dpad_down) {
            scoop.setTargetPosition(0);
            scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoop.setPower(scoopSpeed);
        }
        if(gamepad2.dpad_left) {
            scoop.setTargetPosition(1830);
            scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoop.setPower(scoopSpeed);
        }
    }

    private void runGateScoop()
    {
        int lowerOpenGate = 180;
        int upperOpenGate = 700; 

        if (gamepad2.x)
        {
            gate = true;
            scoop.setTargetPosition(lowerOpenGate);
            scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoop.setPower(scoopSpeed);
        } else if (gate)
        {
            gate = false;
            scoop.setTargetPosition(upperOpenGate);
            scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoop.setPower(0.15);
        }
    }

    private void runBarLift()
    {
        int barTopPos = 1258;

        if(gamepad2.a) {
            barLift.setTargetPosition(barTopPos);
            barLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            barLift.setPower(0.5);
        }
        if(gamepad2.b) {
            barLift.setTargetPosition(0);
            barLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            barLift.setPower(0.5);
        }
    }
}
