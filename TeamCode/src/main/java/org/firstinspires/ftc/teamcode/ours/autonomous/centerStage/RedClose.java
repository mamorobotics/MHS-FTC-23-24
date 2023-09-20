package org.firstinspires.ftc.teamcode.ours.autonomous.centerStage;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedClose")
public class RedClose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d startPos = new Pose2d(60, 12, Math.toRadians(90));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startPos)
                .lineToSplineHeading(new Pose2d(61,61, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(seq1);
        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(seq1.end())
                .splineToSplineHeading(new Pose2d(60, 36, Math.toRadians(90)), 1.5707963267948966)
                .lineToConstantHeading(new Vector2d(36, 36))
                .lineToLinearHeading(new Pose2d(36, 50, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(seq2);
        TrajectorySequence seq3 = drive.trajectorySequenceBuilder(seq2.end())
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36, -60, Math.toRadians(270)))
                .build();
        drive.followTrajectorySequence(seq3);
        TrajectorySequence seq4 = drive.trajectorySequenceBuilder(seq3.end())
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36, 50, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(seq4);
        TrajectorySequence seq5 = drive.trajectorySequenceBuilder(seq4.end())
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36, -60, Math.toRadians(270)))
                .build();
        drive.followTrajectorySequence(seq5);
        TrajectorySequence seq6 = drive.trajectorySequenceBuilder(seq5.end())
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36, 50, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(seq5);
    }
}
