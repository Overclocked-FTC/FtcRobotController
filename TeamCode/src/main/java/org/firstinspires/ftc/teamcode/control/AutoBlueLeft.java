/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


/*
Autonomous Objective
Objective: Find which level of the shipping hub the crate needs to get on and place the crate on the
hub. Drive over to the warehouse and park inside of it.
Steps to complete the objective:
1. Use camera to scan where the team shipping element is
2. Drive forward plowing through the team shipping element until in line with the shipping hub
3. Turn 90 degrees to the left to face shipping hub
4. Raise arm to correct position to put the crate on the shipping hub
5. Drive forward until the grabber is above the shipping hub
6. Drop the crate
7. Back up away from the shipping hub
8. Lower the arm back to pos0
9. Strafe back to the side of the wall, hitting wall to align with it
10. drive into the warehouse
11. Strafe into the corner of the warehouse to get out of the way of alliance team's autonomous
12. Done
 */

@Autonomous(name = "Auto Blue Left", preselectTeleOp = "TeleOp_Iterative")
public class AutoBlueLeft extends LinearOpMode {

    // Declare OpMode members.
    Provider robot = new Provider();
    private ElapsedTime runtime = new ElapsedTime();

    //Tensor Flow variables
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
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
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    boolean isDuckDetected = false;
    String levelPosition = "";
    double targetLevel;

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

    @Override
    public void runOpMode() {
        //Initialize the robot
        robot.init(hardwareMap);

        //Initialize encoders
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Initialize arm encoder

        //Initialize servos
        robot.grabber.setPosition(robot.GRABBER_CLOSE);

        //Tell that everything has been initialized
        telemetry.addData("Status", "Initialized");
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

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Code that finds which barcode the duck is on
        runtime.reset();
        if (opModeIsActive()) {
            while (opModeIsActive() && !isDuckDetected) {
                telemetry.addData("Time", "%2.5f S Elapsed", runtime.seconds());
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
                                targetLevel = robot.armPos1;
                            } else if (recognition.getLeft() > 408) {
                                telemetry.addData("Arm Position", "2");
                                levelPosition = "Level two";
                                targetLevel = robot.armPos2;
                            }
                        }
                        telemetry.update();
                    }
                }
                if (runtime.seconds() > 4) {
                    isDuckDetected = true;
                    telemetry.addData("Arm Position", "3");
                    levelPosition = "Level three";
                    targetLevel = robot.armPos3;
                }
            }
        }
        telemetry.addData("Yeet", levelPosition);
        telemetry.update();

        //Code that makes the robot move
        drive_forward_time(0.25, 500);
        drive_forward_time(0.5, 950);
        move_arm(targetLevel);
        turn_right_time(0.4, 650);
        drive_forward_time(0.25, 200);
        open_grabber();
        sleep(750);
        drive_backward_time(0.5, 250);
        move_arm(robot.armPos0);
        strafe_right_time(0.75, 1200);
        strafe_right_time(0.25, 250);
        sleep(200);
        drive_backward_time(0.5, 1000);
        strafe_left_time(0.75, 1000);
        drive_backward_time(0.5, 500);

        //Done with Autonomous
        sleep(5000);
    }

    //Here are all the methods used to drive the robot in auto
    public void drive_forward(double power) {
        robot.driveLF.setPower(power);
        robot.driveRF.setPower(power);
        robot.driveLB.setPower(power);
        robot.driveRB.setPower(power);
    }

    public void drive_forward_time(double power, long time) {
        drive_forward(power);
        drive_time(time);
        stop_motors();
    }

    public void drive_backward(double power) {
        robot.driveLF.setPower(-power);
        robot.driveRF.setPower(-power);
        robot.driveLB.setPower(-power);
        robot.driveRB.setPower(-power);
    }

    public void drive_backward_time(double power, long time) {
        drive_backward(power);
        drive_time(time);
        stop_motors();
    }

    public void strafe_left(double power) {
        robot.driveLF.setPower(-power);
        robot.driveRF.setPower(power);
        robot.driveLB.setPower(power);
        robot.driveRB.setPower(-power);
    }

    public void strafe_left_time(double power, long time) {
        strafe_left(power);
        drive_time(time);
        stop_motors();
    }

    public void strafe_right(double power) {
        robot.driveLF.setPower(power);
        robot.driveRF.setPower(-power);
        robot.driveLB.setPower(-power);
        robot.driveRB.setPower(power);
    }

    public void strafe_right_time(double power, long time) {
        strafe_right(power);
        drive_time(time);
        stop_motors();
    }

    public void turn_left(double power) {
        robot.driveLF.setPower(-power);
        robot.driveRF.setPower(power);
        robot.driveLB.setPower(-power);
        robot.driveRB.setPower(power);
    }

    public void turn_left_time(double power, long time) {
        turn_left(power);
        drive_time(time);
        stop_motors();
    }

    public void turn_right(double power) {
        robot.driveLF.setPower(power);
        robot.driveRF.setPower(-power);
        robot.driveLB.setPower(power);
        robot.driveRB.setPower(-power);
    }

    public void turn_right_time(double power, long time) {
        turn_right(power);
        drive_time(time);
        stop_motors();
    }

    public void move_arm(double position) {
        robot.arm_move(position);
        while (opModeIsActive() && robot.armMotor.isBusy()){
            telemetry.addData("Action", "moving arm");
            telemetry.update();
        }
        robot.armMotor.setPower(0);
    }

    public void open_grabber() {
        robot.grabber.setPosition(Provider.GRABBER_OPEN);
        telemetry.addData("Action", "opened grabber");
        telemetry.update();
    }

    public void close_grabber() {
        robot.grabber.setPosition(Provider.GRABBER_CLOSE);
        telemetry.addData("Action", "closed grabber");
        telemetry.update();

    }

    public void drive_time(long time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < time)) {
            telemetry.addData("Time", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void stop_motors() {
        robot.driveLF.setPower(0);
        robot.driveRF.setPower(0);
        robot.driveLB.setPower(0);
        robot.driveRB.setPower(0);
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
}
