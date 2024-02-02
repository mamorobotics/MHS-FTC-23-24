package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@ com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class DriverMode extends OpMode {
    static DriveTrain driveTrain = new DriveTrain();
    static double speed = 0.4;
    static Servo airServo;
    static Servo clawServo;
    static DcMotor barLift;

    @Override
    public void init() {
        driveTrain.setDriveTrain(hardwareMap, "leftFront", "leftRear", "rightFront", "rightRear");
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveTrain.reverse(DriveTrain.DriveTrainSide.LEFT);
        //airServo = hardwareMap.get(Servo.class, "airServo");
        //clawServo = hardwareMap.get(Servo.class, "clawServo");
        barLift = hardwareMap.get(DcMotorEx.class, "barLift");
        barLift.setDirection(DcMotorSimple.Direction.REVERSE);
        barLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        int pos = 1258;
        driveTrain.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
        if(gamepad1.a){
            barLift.setTargetPosition(pos);
            barLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            barLift.setPower(0.5);
        }
        if(gamepad1.b){
            barLift.setTargetPosition(0);
            barLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            barLift.setPower(0.5);
        }
        if(gamepad1.x){
            airServo.setPosition(.3);
        }
        telemetry.addData("Bar Lift Encoder Position", barLift.getCurrentPosition());
        telemetry.addData("Bar Lift Desired Position", barLift.getTargetPosition());
        telemetry.update();
    }
}
