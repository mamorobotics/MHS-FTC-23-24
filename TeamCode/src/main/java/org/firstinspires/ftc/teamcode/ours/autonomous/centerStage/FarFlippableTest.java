package org.firstinspires.ftc.teamcode.ours.autonomous.centerStage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Queue;

public class FarFlippableTest extends LinearOpMode {
    public Queue<Trajectory> farQueueBuilder(boolean flipped) {
        int multiplier = flipped ? 1 : -1;

        Pose2d startPos = new Pose2d(60, -36, Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                .lineToConstantHeading(new Vector2d(61 * multiplier, -61))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToConstantHeading(new Vector2d(36 * multiplier, -36), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(36 * multiplier, 50, Math.toRadians(90)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(36 * multiplier, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36 * multiplier, -60, Math.toRadians(270)))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(36 * multiplier, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36 * multiplier, 50, Math.toRadians(90)))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(36 * multiplier, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36 * multiplier, -60, Math.toRadians(270)))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(36 * multiplier, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36 * multiplier, 50, Math.toRadians(90)))
                .build();

        Queue<Trajectory> trajectoryQueue = null;
        trajectoryQueue.add(traj1);
        trajectoryQueue.add(traj2);
        trajectoryQueue.add(traj3);
        trajectoryQueue.add(traj4);
        trajectoryQueue.add(traj5);
        trajectoryQueue.add(traj6);
        return trajectoryQueue;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
