package com.example.meepmeep.centerStage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class blue {
    public static void main(String[] args)
    {
        MeepMeep meepMeep = new MeepMeep(600);

        //top
        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 13.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60,12,Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-61,61, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-60, 36, Math.toRadians(90)), 1.5707963267948966)
                                .lineToConstantHeading(new Vector2d(-36, 36))
                                .lineToLinearHeading(new Pose2d(-36, 50, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-36, 50, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-36, 50, Math.toRadians(90)))
                                .build()
//                                .addDisplacementMarker(() -> {
//                                    //LM.setTargetPosition(-2000);
//                                    //LM.setPower(-1);
//                                })
//                                .lineToSplineHeading(new Pose2d(36, -12, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(36, 12, Math.toRadians(270)))
//                                .addDisplacementMarker(() -> {
//                                    //LM.setTargetPosition(-3500);
//                                    //LM.setPower(-1);
//                                })
//                                .turn(Math.toRadians(-45))
//                                .waitSeconds(0.5)
//                                .addDisplacementMarker(() -> {
//                                    //LM.setTargetPosition(0);
//                                    //LM.setPower(1);
//                                })
//                                .turn(Math.toRadians(135))
//                                .lineToLinearHeading(new Pose2d(57, 12, Math.toRadians(0)))
//                                .waitSeconds(0.5)
//                                .addDisplacementMarker(() -> {
//                                    //LM.setTargetPosition(-3500);
//                                    //LM.setPower(-1);
//                                })
//                                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(0)))
//                                .turn(Math.toRadians(-135))
//                                .waitSeconds(0.5)
//                                .addDisplacementMarker(() -> {
//                                    //LM.setTargetPosition(0);
//                                    //LM.setPower(1);
//                                })
//                                .turn(Math.toRadians(-45))
//                                .lineToSplineHeading((new Pose2d(36,36,Math.toRadians(180))))
//                                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(62, 36, Math.toRadians(180)))
//                                .build()
                        );

        //bottom
        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 13.50)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-60,-36,Math.toRadians(0)))
                                        .lineToSplineHeading(new Pose2d(-61,-61, Math.toRadians(90)))
                                        .build()
//                                .addDisplacementMarker(() -> {
//                                    //LM.setTargetPosition(-2000);
//                                    //LM.setPower(-1);
//                                })
//                                .lineToSplineHeading(new Pose2d(36, -12, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(36, 12, Math.toRadians(270)))
//                                .addDisplacementMarker(() -> {
//                                    //LM.setTargetPosition(-3500);
//                                    //LM.setPower(-1);
//                                })
//                                .turn(Math.toRadians(-45))
//                                .waitSeconds(0.5)
//                                .addDisplacementMarker(() -> {
//                                    //LM.setTargetPosition(0);
//                                    //LM.setPower(1);
//                                })
//                                .turn(Math.toRadians(135))
//                                .lineToLinearHeading(new Pose2d(57, 12, Math.toRadians(0)))
//                                .waitSeconds(0.5)
//                                .addDisplacementMarker(() -> {
//                                    //LM.setTargetPosition(-3500);
//                                    //LM.setPower(-1);
//                                })
//                                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(0)))
//                                .turn(Math.toRadians(-135))
//                                .waitSeconds(0.5)
//                                .addDisplacementMarker(() -> {
//                                    //LM.setTargetPosition(0);
//                                    //LM.setPower(1);
//                                })
//                                .turn(Math.toRadians(-45))
//                                .lineToSplineHeading((new Pose2d(36,36,Math.toRadians(180))))
//                                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(62, 36, Math.toRadians(180)))
//                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot1)
                .addEntity(bot2)
                .start();
    }
}
