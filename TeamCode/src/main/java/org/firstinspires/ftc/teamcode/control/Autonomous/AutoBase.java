package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.control.Provider;
import org.firstinspires.ftc.teamcode.hardware.manipulators.Grabber_911;

import java.util.List;

public abstract class AutoBase extends LinearOpMode {
    // Declare OpMode members.
    Provider robot = new Provider();

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    // Vectors
    Vector2d vSignalZone1 = new Vector2d(11.75, -11.75);
    Vector2d vSignalZone2 = new Vector2d(35.25, -11.75);
    Vector2d vSignalZone3 = new Vector2d(58.75, -11.75);

    // Variables
    boolean isSignalDetected = false;
    String signalZone = "";
//    Vector2d targetZone = vSignalZone3;
    String targetZone = "Zone 3";

    private static final String VUFORIA_KEY =
            "AbfoyYX/////AAABmb/61+6Y2U5Lr+ETwpWurGhmj+twGo3rVHrd61Dn3Gm9bQzp1GCXxWVz+LRj1iQ2pmB0bFiBTqUjXIKtubsE/xcdnG0/ZTHPZkO2jcWObwVsdMDkvP7eHw/VW+XsfyBn687dYVHantczOsr1MC46u8wmBncQXDeRwWSZjM1HjIiWaRPqcE6ksSwBLgZ3N/U+qsPonAkjcS1IHugS78zc4YTTfiVpNsxy8COx7jyCEXqVkIob0kgQXkXdqdfTn3n2Vd48vCKdvjE362R1ltxQzJ+eqzHdK4eIcBIPhIy/TPnu3UuHNmGU+gM/bawBSOM8ylYPhA1CHlutClEIbYK9LYNBjUPYYsG28+GbcPD6fsAH";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //Methods
    //Initialization method
    public void auto_init() {
        //Initialize the robot
        robot.init(hardwareMap);

        //Initialize encoders
        robot.towers.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Initialize lift motor encoder

        //Initialize servos
        robot.claw.grabber.setPosition(Grabber_911.GRABBER_CLOSE);

        //Tell that everything has been initialized
        telemetry.addData("Status", "Waiting for camera");
        telemetry.update();

        //Initialize Tensor Flow
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2, 16.0 / 9.0);
        }
    }

    // TODO: Add camera detection stuff

    //Camera detection method
    public void detect_zone_pos() {
        //Code that finds which barcode the duck/shipping element is on
        robot.runtime.reset();
        if (opModeIsActive()) {
            while (opModeIsActive() && !isSignalDetected) {
                telemetry.addData("Time", "%2.5f S Elapsed", robot.runtime.seconds());
                telemetry.update();

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);


                            // check label to see if the camera now sees a Duck
                            if (recognition.getLabel().equals("1 Bolt") || recognition.getLabel().equals("2 Bulb") || recognition.getLabel().equals("3 Panel")) {
                                isSignalDetected = true;
                                telemetry.addData("Object Detected", "Signal");
                            }


                            // Determine which signal image is showing
                            if (recognition.getLabel().equals("1 Bolt")) {
                                telemetry.addData("Signal Zone", "1");
                                signalZone = "Signal zone one";
                                targetZone = "Zone 1";
                            } else if (recognition.getLabel().equals("2 Bulb")) {
                                telemetry.addData("Signal Zone", "2");
                                signalZone = "Signal zone two";
                                targetZone = "Zone 2";
                            } else if (recognition.getLabel().equals("3 Panel")) {
                                telemetry.addData("Signal Zone", "3");
                                signalZone = "Signal zone three";
                                targetZone = "Zone 3";
                            }
                        }
                        telemetry.update();
                    }
                }
                if (robot.runtime.seconds() > 2) {
                    isSignalDetected = true;
                    telemetry.addData("Signal Zone", "3");
                    signalZone = "Signal zone three";
                    targetZone = "Zone 3";
                }
                telemetry.addData("Yeet", signalZone);
                telemetry.update();
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    //Drive methods
    //Manipulation methods
    public void lift_towers(double position) {
        robot.towers.towers_lift(position);
        while (opModeIsActive() && robot.towers.liftMotor.isBusy()) {
            telemetry.addData("Action", "Lifting towers");
            telemetry.update();
        }
//        robot.towers.liftMotor.setPower(0);
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

