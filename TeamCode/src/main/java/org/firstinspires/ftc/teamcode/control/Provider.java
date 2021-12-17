package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Provider {

    //Public variables
    public DcMotor driveLF = null;
    public DcMotor driveRF = null;
    public DcMotor driveLB = null;
    public DcMotor driveRB = null;

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

        //Reverse the motor that runs backwards when connected directly to the battery
        driveLF.setDirection(DcMotor.Direction.FORWARD);
        driveRF.setDirection(DcMotor.Direction.REVERSE);
        driveLB.setDirection(DcMotor.Direction.FORWARD);
        driveRB.setDirection(DcMotor.Direction.REVERSE);

        //Set all motors to zero power
        driveLF.setPower(0);
        driveRF.setPower(0);
        driveLB.setPower(0);
        driveRB.setPower(0);
    }
}
