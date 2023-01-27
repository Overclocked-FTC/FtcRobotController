package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum;
import org.firstinspires.ftc.teamcode.hardware.manipulators.Twin_Towers;
import org.firstinspires.ftc.teamcode.hardware.manipulators.Grabber_911;

public class Provider {
    /*
    The Provider Class provides all the necessary information to run different mechanisms.
    This class declares all the motors, servos, and sensors that we will use.
    It hardware maps the hardware to the software and sets all the basic information for each piece.
    */

    // Hardware Components
    public Twin_Towers towers = new Twin_Towers();
    public Grabber_911 claw = new Grabber_911();
    public Drive_Mecanum drive = new Drive_Mecanum();

    // local OpMode members
    public ElapsedTime runtime  = new ElapsedTime();

    // Constructor
    public Provider(){
    }

    // Initialize standard Hardware interfaces
    public void init(HardwareMap hwMap) {
        towers.init(hwMap);
        claw.init(hwMap);
        drive.init(hwMap);
    }
}
