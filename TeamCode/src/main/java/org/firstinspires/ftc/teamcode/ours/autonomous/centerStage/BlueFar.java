package org.firstinspires.ftc.teamcode.ours.autonomous.centerStage;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "BlueFar")
public class BlueFar extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private int minConfidenceValue = 90;
    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d startPos = new Pose2d(-60, -36, Math.toRadians(90));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int screenWidth = 1280;
        Vector2d centerPoint = null;

        initTfod();
        while(!isStarted() || centerPoint == null){
            centerPoint = telemetryTfod();
            telemetry.update();
        }
        visionPortal.close();

        waitForStart();

        TrajectorySequence seqLeft = drive.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(-61, -61))
                .build();

        TrajectorySequence seqMiddle = drive.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(-61, -61))
                .build();

        TrajectorySequence seqRight = drive.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(-61, -61))
                .build();

        Queue<Trajectory> trajectoryQueue;



        if(centerPoint == null){

        }
        else if(centerPoint.getX() < screenWidth/3){
            trajectoryQueue.add(trajectory1);
            trajectoryQueue.add(trajectory2);
            trajectoryQueue.add(trajectory3);
            trajectoryQueue.add(trajectory4);
        }
        else if(centerPoint.getX() > screenWidth/3 && centerPoint.getX() < (screenWidth/3)*2){
            drive.followTrajectorySequence(seqMiddle);
        }
        else{
            drive.followTrajectorySequence(seqRight);
        }

        drive.followTrajectory(trajectoryQueue.poll());

        /*
        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(seq1.end())
                .splineToConstantHeading(new Vector2d(-36, -36), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-36, 50, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(seq2);
        TrajectorySequence seq3 = drive.trajectorySequenceBuilder(seq2.end())
                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(270)))
                .build();
        drive.followTrajectorySequence(seq3);
        TrajectorySequence seq4 = drive.trajectorySequenceBuilder(seq3.end())
                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-36, 50, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(seq4);
        TrajectorySequence seq5 = drive.trajectorySequenceBuilder(seq4.end())
                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(270)))
                .build();
        drive.followTrajectorySequence(seq5);
        TrajectorySequence seq6 = drive.trajectorySequenceBuilder(seq5.end())
                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-36, 50, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(seq5);
         */
    }
    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                //.setModelLabels(LABELS)
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
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private Vector2d telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        Recognition object = currentRecognitions.get(0);
        double x = (object.getLeft() + object.getRight()) / 2 ;
        double y = (object.getTop()  + object.getBottom()) / 2 ;

        telemetry.addData(""," ");
        telemetry.addData("Image", "%s (%.0f %% Conf.)", object.getLabel(), object.getConfidence() * 100);
        telemetry.addData("- Position", "%.0f / %.0f", x, y);
        telemetry.addData("- Size", "%.0f x %.0f", object.getWidth(), object.getHeight());

        if(object.getConfidence() * 100 > minConfidenceValue){
            return new Vector2d(x,y);
        }
        return null;
    }
}
