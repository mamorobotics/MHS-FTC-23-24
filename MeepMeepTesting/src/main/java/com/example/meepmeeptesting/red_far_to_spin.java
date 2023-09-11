package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class red_far_to_spin {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 16.34)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                                .forward(3)
                                .lineToSplineHeading(new Pose2d(2,-37,Math.toRadians(315)))
                                .addTemporalMarker(()->{
                                    //put the item to the right height
                                })
                                .forward(10)
                                .lineToLinearHeading(new Pose2d(-55,-55,Math.toRadians(180)))
                                .addDisplacementMarker(() -> {
                                    //spin the thing
                                })

                                .back(10)
                                .lineToLinearHeading(new Pose2d(-60,-35,Math.toRadians(0)))
                                .build()
                        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}