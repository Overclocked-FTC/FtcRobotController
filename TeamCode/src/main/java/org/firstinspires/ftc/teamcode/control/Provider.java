package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum;
import org.firstinspires.ftc.teamcode.hardware.manipulators.Arm;
import org.firstinspires.ftc.teamcode.hardware.manipulators.Carousel_Spinner;
import org.firstinspires.ftc.teamcode.hardware.manipulators.Grabber_3000;

public class Provider {
    /*
    The Provider Class provides all the necessary information to run different mechanisms.
    This class declares all the motors, servos, and sensors that we will use.
    It hardware maps the hardware to the software and sets all the basic information for each piece.
    */

    //Hardware Components
    public Arm arm = new Arm();
    public Carousel_Spinner carousel = new Carousel_Spinner();
    public Drive_Mecanum drive = new Drive_Mecanum();
    public Grabber_3000 claw = new Grabber_3000();

    //local OpMode members
    public ElapsedTime runtime  = new ElapsedTime();

    //Constructor
    public Provider(){
    }

    //Initialize standard Hardware interfaces
    public void init(HardwareMap hwMap) {
        arm.init(hwMap);
        carousel.init(hwMap);
        claw.init(hwMap);
        drive.init(hwMap);
    }
}
