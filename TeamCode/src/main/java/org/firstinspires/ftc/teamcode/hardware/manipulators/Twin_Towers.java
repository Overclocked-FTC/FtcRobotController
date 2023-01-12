package org.firstinspires.ftc.teamcode.hardware.manipulators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.Provider;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

public class Twin_Towers {
    //Hardware Components
    public DcMotor liftMotor = null;

    //Variables
    static final double MOTOR_TICK_COUNT = 537.7;
//    public double motorDegree = MOTOR_TICK_COUNT / 360;
//    public double GearRatio = 1;
    public double pulleyCircumference = 112; //Circumference in mm of pulley
    public double motorTickPerMillimeter = MOTOR_TICK_COUNT / pulleyCircumference;
    public double liftPos0 = 0; //Arm is down all the way
    public double liftPos1 = motorTickPerMillimeter * -343; //Last number is height in millimeters. Low Junction
    public double liftPos2 = motorTickPerMillimeter * -597; //Medium Junction
    public double liftPos3 = motorTickPerMillimeter * -850; //High Junction
    public double liftPosTest = motorTickPerMillimeter * -100;

    //Constructor
    public Twin_Towers() {
    }

    //Initialization
    public void init(HardwareMap hwMap) {
        //Define and Initialize Motor
        liftMotor = hwMap.get(DcMotor.class, "lift_motor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Initialize arm encoder
    }

    //Method to move the arm
    public void arm_move(double liftPos) {
        liftMotor.setTargetPosition(((int)liftPos));
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(.25);

        if (!liftMotor.isBusy()) {
            liftMotor.setPower(0);
        }
    }
}
