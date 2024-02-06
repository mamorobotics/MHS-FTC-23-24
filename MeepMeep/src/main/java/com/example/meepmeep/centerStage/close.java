package com.example.meepmeep.centerStage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class close {
    public static void main(String[] args)
    {
        MeepMeep meepMeep = new MeepMeep(600);

        //blue
        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(30, 30, 39.66141269883581, Math.toRadians(180), 4.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(70,70,Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(40,44, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(30, 36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-12, 36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-12, 60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-48, 36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-48, 36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-12, 60, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-12, 36, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(40,44, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(70,70,Math.toRadians(270)))
                                .build()
                        );

        //red
        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(30, 30, 39.66141269883581, Math.toRadians(180), 4.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(70,-70,Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(40,-44, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(30, -36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-12, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-48, -36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-48, -36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-12, -60, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(40,-44, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(70,-70,Math.toRadians(90)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot1)
                .addEntity(bot2)
                .start();
    }
}
