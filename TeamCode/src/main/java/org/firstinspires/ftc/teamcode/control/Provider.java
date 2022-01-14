package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.tensorflow.lite.annotations.UsedByReflection;

public class Provider {

    //Public OpMode Members
    public DcMotor driveLF = null;
    public DcMotor driveRF = null;
    public DcMotor driveLB = null;
    public DcMotor driveRB = null;
    public DcMotor armMotor = null;
    public CRServo duckSpinner = null;
    public Servo grabber = null;

    public final static double GRABBER_OPEN = 0.0; //Sets the starting positions of the servo
    public final static double GRABBER_CLOSE = 0.1;


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
        armMotor.setDirection(DcMotor.Direction.FORWARD);

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

    }
}
