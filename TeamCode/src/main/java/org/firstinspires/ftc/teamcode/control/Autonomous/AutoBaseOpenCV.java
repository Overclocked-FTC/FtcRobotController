package org.firstinspires.ftc.teamcode.control.Autonomous;

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
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

public abstract class AutoBaseOpenCV extends LinearOpMode {
    // Declare OpMode members.
    Provider robot = new Provider();

    // Variables
    boolean isSignalDetected = false;
    String signalZone = "";
//    Vector2d targetZone = vSignalZone3;
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

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;


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

