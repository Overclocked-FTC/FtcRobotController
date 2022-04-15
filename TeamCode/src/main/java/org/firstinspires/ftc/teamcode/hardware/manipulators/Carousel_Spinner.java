package org.firstinspires.ftc.teamcode.hardware.manipulators;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.Provider;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

public class Carousel_Spinner {
    //Hardware Components
    public CRServo duckSpinner = null; //https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/
    public DcMotor spinnerDuck = null;

    //Variables

    //Constructor
    public void Carousel_Spinner() {
    }

    //Initialization
    public void init(HardwareMap hwMap) {
        //Define and initialize servo
        duckSpinner = hwMap.crservo.get("duck_spinner");
        spinnerDuck = hwMap.get(DcMotor.class,"duck_spinner_motor");
        spinnerDuck.setDirection(DcMotor.Direction.FORWARD);
        spinnerDuck.setPower(0);
        spinnerDuck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
