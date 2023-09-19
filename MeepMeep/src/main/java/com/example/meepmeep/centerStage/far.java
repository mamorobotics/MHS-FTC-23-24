package com.example.meepmeep.centerStage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class far {
    public static void main(String[] args)
    {
        MeepMeep meepMeep = new MeepMeep(600);

        //blue
        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 13.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60,12,Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-36, 50, Math.toRadians(90)))
                                .build()
                );

        //red
        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 13.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60,12,Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(36, 50, Math.toRadians(90)))
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
