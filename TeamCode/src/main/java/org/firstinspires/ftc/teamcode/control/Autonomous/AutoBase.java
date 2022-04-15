package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.control.Provider;
import org.firstinspires.ftc.teamcode.hardware.manipulators.Grabber_3000;

import java.util.List;

public abstract class AutoBase extends LinearOpMode {
    // Declare OpMode members.
    Provider robot = new Provider();

    //Tensor Flow variables
    public static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    public static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    boolean isDuckDetected = false;
    String levelPosition = "";
    double targetLevel;

    public static final String VUFORIA_KEY =
            "AbfoyYX/////AAABmb/61+6Y2U5Lr+ETwpWurGhmj+twGo3rVHrd61Dn3Gm9bQzp1GCXxWVz+LRj1iQ2pmB0bFiBTqUjXIKtubsE/xcdnG0/ZTHPZkO2jcWObwVsdMDkvP7eHw/VW+XsfyBn687dYVHantczOsr1MC46u8wmBncQXDeRwWSZjM1HjIiWaRPqcE6ksSwBLgZ3N/U+qsPonAkjcS1IHugS78zc4YTTfiVpNsxy8COx7jyCEXqVkIob0kgQXkXdqdfTn3n2Vd48vCKdvjE362R1ltxQzJ+eqzHdK4eIcBIPhIy/TPnu3UuHNmGU+gM/bawBSOM8ylYPhA1CHlutClEIbYK9LYNBjUPYYsG28+GbcPD6fsAH";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    //Methods
    //Initialization method
    public void auto_init() {
        //Initialize the robot
        robot.init(hardwareMap);

        //Initialize encoders
        robot.arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Initialize arm encoder

        //Initialize servos
        robot.claw.grabber.setPosition(Grabber_3000.GRABBER_CLOSE);

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
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2, 16.0/9.0);
        }
    }

    //Camera detection method
    public void detect_barcode_pos() {
        //Code that finds which barcode the duck/shipping element is on
        robot.runtime.reset();
        if (opModeIsActive()) {
            while (opModeIsActive() && !isDuckDetected) {
                telemetry.addData("Time", "%2.5f S Elapsed", robot.runtime.seconds());
                telemetry.update();

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;

                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;

                            // check label to see if the camera now sees a Duck
                            if (recognition.getLabel().equals("Duck")) {
                                isDuckDetected = true;
                                telemetry.addData("Object Detected", "Duck");
                            } else {
                                isDuckDetected = false;
                            }

                            //Determine where the duck is based of the X-axis of the duck
                            if (recognition.getLeft() < 408) {
                                telemetry.addData("Arm Position", "1");
                                levelPosition = "Level one";
                                targetLevel = robot.arm.armPos1;
                            } else if (recognition.getLeft() > 408 && recognition.getLeft() < 500) {
                                telemetry.addData("Arm Position", "2");
                                levelPosition = "Level two";
                                targetLevel = robot.arm.armPos2;
                            }
                        }
                        telemetry.update();
                    }
                }
                if (robot.runtime.seconds() > 2) {
                    isDuckDetected = true;
                    telemetry.addData("Arm Position", "3");
                    levelPosition = "Level three";
                    targetLevel = robot.arm.armPos3;
                }
            }
        }
        telemetry.addData("Yeet", levelPosition);
        telemetry.update();
    }

    //Vuforia and tensor flow methods
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

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    //Drive methods
    //Here are all the methods used to drive the robot in auto
    public void drive_forward(double power) {
        robot.drive.driveLF.setPower(power);
        robot.drive.driveRF.setPower(power);
        robot.drive.driveLB.setPower(power);
        robot.drive.driveRB.setPower(power);
    }

    public void drive_forward_time(double power, long time) {
        drive_forward(power);
        drive_time(time);
        stop_motors();
    }

    public void drive_backward(double power) {
        robot.drive.driveLF.setPower(-power);
        robot.drive.driveRF.setPower(-power);
        robot.drive.driveLB.setPower(-power);
        robot.drive.driveRB.setPower(-power);
    }

    public void drive_backward_time(double power, long time) {
        drive_backward(power);
        drive_time(time);
        stop_motors();
    }

    public void strafe_left(double power) {
        robot.drive.driveLF.setPower(-power);
        robot.drive.driveRF.setPower(power);
        robot.drive.driveLB.setPower(power);
        robot.drive.driveRB.setPower(-power);
    }

    public void strafe_left_time(double power, long time) {
        strafe_left(power);
        drive_time(time);
        stop_motors();
    }

    public void strafe_right(double power) {
        robot.drive.driveLF.setPower(power);
        robot.drive.driveRF.setPower(-power);
        robot.drive.driveLB.setPower(-power);
        robot.drive.driveRB.setPower(power);
    }

    public void strafe_right_time(double power, long time) {
        strafe_right(power);
        drive_time(time);
        stop_motors();
    }

    public void turn_left(double power) {
        robot.drive.driveLF.setPower(-power);
        robot.drive.driveRF.setPower(power);
        robot.drive.driveLB.setPower(-power);
        robot.drive.driveRB.setPower(power);
    }

    public void turn_left_time(double power, long time) {
        turn_left(power);
        drive_time(time);
        stop_motors();
    }

    public void turn_right(double power) {
        robot.drive.driveLF.setPower(power);
        robot.drive.driveRF.setPower(-power);
        robot.drive.driveLB.setPower(power);
        robot.drive.driveRB.setPower(-power);
    }

    public void turn_right_time(double power, long time) {
        turn_right(power);
        drive_time(time);
        stop_motors();
    }

    public void drive_time(long time) {
        robot.runtime.reset();
        while (opModeIsActive() && (robot.runtime.milliseconds() < time)) {
            telemetry.addData("Time", "%2.5f S Elapsed", robot.runtime.seconds());
            telemetry.addData("driveLF", robot.drive.driveLF.getCurrentPosition());
            telemetry.addData("driveRF", robot.drive.driveRF.getCurrentPosition());
            telemetry.addData("driveLB", robot.drive.driveLB.getCurrentPosition());
            telemetry.addData("driveRB", robot.drive.driveRB.getCurrentPosition());
            telemetry.update();
        }
    }

    public void stop_motors() {
        robot.drive.driveLF.setPower(0);
        robot.drive.driveRF.setPower(0);
        robot.drive.driveLB.setPower(0);
        robot.drive.driveRB.setPower(0);
    }

    //Manipulation methods
    public void move_arm(double position) {
        robot.arm.arm_move(position);
        while (opModeIsActive() && robot.arm.armMotor.isBusy()){
            telemetry.addData("Action", "moving arm");
            telemetry.update();
        }
        robot.arm.armMotor.setPower(0);
    }

    public void open_grabber() {
        robot.claw.grabber.setPosition(Grabber_3000.GRABBER_OPEN);
        telemetry.addData("Action", "opened grabber");
        telemetry.update();
    }

    public void close_grabber() {
        robot.claw.grabber.setPosition(Grabber_3000.GRABBER_CLOSE);
        telemetry.addData("Action", "closed grabber");
        telemetry.update();

    }

    public void spin_duck(double power, long time) {
        robot.carousel.spinnerDuck.setPower(power);
        drive_time(time);
        robot.carousel.spinnerDuck.setPower(0);
    }
}
