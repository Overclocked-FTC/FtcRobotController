package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class Provider {
    /*
    The Provider Class provides all the necessary information to run different mechanisms.
    This class declares all the motors, servos, and sensors that we will use.
    It hardware maps the hardware to the software and sets all the basic information for each piece.
    */

    //Public OpMode Members
    public DcMotor driveLF = null; //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public DcMotor driveRF = null;
    public DcMotor driveLB = null;
    public DcMotor driveRB = null;
    public DcMotor armMotor = null;
    public CRServo duckSpinner = null; //https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/
    public Servo grabber = null;

    //Motor encoder variables
    static final double MOTOR_TICK_COUNT = 537.7;
    double motorDegree = MOTOR_TICK_COUNT / 360;
    double wormGearRatio = 28;
    public double armPos0 = 0; //Arm is down all the way
    public double armPos1 = motorDegree * wormGearRatio * 50; //Last number is degrees of rotation. Shipping hub 1st layer
    public double armPos2 = motorDegree * wormGearRatio * 82; //Shipping hub 2nd layer
    public double armPos3 = motorDegree * wormGearRatio * 115; //Shipping hub 3rd layer
    //double pos4 =
    int slowMo = 1;

    //Servo variables
    public final static double GRABBER_OPEN = 0.0; //Sets the starting positions of the servo
    public final static double GRABBER_CLOSE = 0.12; //Sets the closed position of the servo

    //local OpMode members
    HardwareMap hwMap           = null;
    public ElapsedTime runtime  = new ElapsedTime();

    //Constructor
    public Provider(){
    }

    //Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        //Save reference to Hardware map
        hwMap = ahwMap;

        //Define and Initialize Motors
        driveLF = hwMap.get(DcMotor.class, "left_front_drive");
        driveRF = hwMap.get(DcMotor.class, "right_front_drive");
        driveLB = hwMap.get(DcMotor.class, "left_back_drive");
        driveRB = hwMap.get(DcMotor.class, "right_back_drive");
        armMotor = hwMap.get(DcMotor.class, "arm_motor");

        //Reverse the motor that runs backwards when connected directly to the battery
        driveLF.setDirection(DcMotor.Direction.FORWARD);
        driveRF.setDirection(DcMotor.Direction.REVERSE);
        driveLB.setDirection(DcMotor.Direction.FORWARD);
        driveRB.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        //Set all motors to zero power
        driveLF.setPower(0);
        driveRF.setPower(0);
        driveLB.setPower(0);
        driveRB.setPower(0);
        armMotor.setPower(0);

        //Whether to run with encoder or not
        driveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Define and initialize servos
        grabber = hwMap.servo.get("grabber");
        duckSpinner = hwMap.crservo.get("duck_spinner");

        //Define and initialize sensors
//        webcamName = hwMap.get(WebcamName.class, "Webcam 1");

    }

    public void arm_move(double armPos) {
        armMotor.setTargetPosition(((int)armPos));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);

        if (!armMotor.isBusy()) {
            armMotor.setPower(0);
        }
    }
}
