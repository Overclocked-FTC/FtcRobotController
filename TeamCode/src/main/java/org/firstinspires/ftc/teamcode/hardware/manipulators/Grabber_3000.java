package org.firstinspires.ftc.teamcode.hardware.manipulators;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber_3000 {
    //Hardware Components
    public Servo grabber = null;

    //Variables
    public final static double GRABBER_OPEN = 0.0; //Sets the starting positions of the servo
    public final static double GRABBER_CLOSE = 0.12; //Sets the closed position of the servo

    //Constructor
    public void Grabber_3000() {
    }

    //Initialization
    public void init(HardwareMap hwMap) {
        //Define and initialize servo
        grabber = hwMap.servo.get("grabber");
        grabber.setPosition(GRABBER_OPEN);
    }
}
