package org.firstinspires.ftc.teamcode.ours.autonomous.centerStage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "RedFar")
public class RedFar extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    Pose2d startpos = new Pose2d(-72 + (14.25 / 2), -72 + 8, Math.toRadians(90));


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    private static final String[] LABELS = {
            "dumbell",
    };

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private int minConfidenceValue = 90;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Vector2d centerPoint = null;

        initTfod();
        while(!isStarted() || centerPoint == null){
            centerPoint = telemetryTfod();
            telemetry.update();
        }
        visionPortal.close();

        waitForStart();

        if(centerPoint.getX() < 500){
            telemetry.addData("left", "");
        }
        else if(centerPoint.getX() > 500){
            telemetry.addData("center", "");
        }
        else{
            telemetry.addData("right", "");
        }
        telemetry.update();

        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        Trajectory left = drive.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(90)))
                .strafeLeft(27, drive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(left);

        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        Trajectory back = drive.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(90)))
                .back(2, drive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(back);

        drive.setPoseEstimate(startpos);
        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startpos)
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(24, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(24, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-48, -36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-48, -36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(24, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(40,-44, Math.toRadians(0)))
                .lineToSplineHeading(startpos)
                .build();
        drive.followTrajectorySequence(seq1);
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName("RedAndBlueV2.tflite")
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private Vector2d telemetryTfod() {
        double x = 1000000000;
        double y = 1000000000;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        if(x==1000000000){return null;}
        return new Vector2d(x,y);
    }
}