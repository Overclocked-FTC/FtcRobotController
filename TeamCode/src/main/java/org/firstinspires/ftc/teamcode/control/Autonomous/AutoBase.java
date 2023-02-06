package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.Provider;
import org.firstinspires.ftc.teamcode.hardware.manipulators.Grabber_911;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public abstract class AutoBase extends LinearOpMode {

    // Declare OpMode members.
    Provider robot = new Provider();

    // Variables
    String signalZone = "";
    String targetZone = "Zone 3";

    // Open CV Variables
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int tagNumber = 3; // Tag ID 3 from the 36h11 family

    AprilTagDetection tagOfInterest = null;


    // Methods
    // Initialization method
    public void auto_init() {
        // Initialize the robot
        robot.init(hardwareMap);

        // Initialize encoders
        robot.towers.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Initialize lift motor encoder

        // Initialize servos
        robot.claw.grabber.setPosition(Grabber_911.GRABBER_CLOSE);

        // Tell that everything has been initialized
        telemetry.addData("Status", "Waiting for camera");
        telemetry.update();

        // Initialize OpenCV and April Tags
        init_camera();
    }

    // Initialize the camera
    public void init_camera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    // Initialize detection
    public void init_detection() {
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    tagNumber = tag.id;
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }

                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
    }

    // Camera detection method
    public void detect_april_tag() {
        /* Update the telemetry */
        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null) {
            telemetry.addData("Signal Zone", "3");
            signalZone = "Signal zone three (3)";
            targetZone = "Zone 3";
        } else {
            if (tagNumber == 1) {
                telemetry.addData("Signal Zone", "1");
                signalZone = "Signal zone one";
                targetZone = "Zone 1";
            } else if (tagNumber == 2) {
                telemetry.addData("Signal Zone", "2");
                signalZone = "Signal zone two";
                targetZone = "Zone 2";
            } else {
                telemetry.addData("Signal Zone", "3");
                signalZone = "Signal zone three";
                targetZone = "Zone 3";
            }
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    // Drive methods
    // Manipulation methods
    public void lift_towers(double position) {
        robot.towers.towers_lift(position);
        while (opModeIsActive() && robot.towers.liftMotor.isBusy()) {
            telemetry.addData("Action", "Lifting towers");
            telemetry.update();
        }
    }

    public void open_grabber() {
        robot.claw.grabber.setPosition(Grabber_911.GRABBER_OPEN);
        telemetry.addData("Action", "opened grabber");
        telemetry.update();
    }

    public void close_grabber() {
        robot.claw.grabber.setPosition(Grabber_911.GRABBER_CLOSE);
        telemetry.addData("Action", "closed grabber");
        telemetry.update();
    }
}

