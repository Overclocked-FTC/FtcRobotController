package org.firstinspires.ftc.teamcode.hardware.manipulators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.Provider;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

public class Twin_Towers {
    // Hardware Components
    public DcMotor liftMotor = null;

    // Variables
    static final double MOTOR_TICK_COUNT = 537.7;
//    public double motorDegree = MOTOR_TICK_COUNT / 360;
    public double GearRatio = 1;
    public double pulleyCircumference = 112; // Circumference in mm of pulley
    public double motorTickPerMillimeter = MOTOR_TICK_COUNT / pulleyCircumference;
    public double liftPos0 = 0; // Arm is down all the way
    public double liftPos1 = motorTickPerMillimeter * 360 ; // Last number is height in millimeters. Low Junction
    public double liftPos2 = motorTickPerMillimeter * 600; // Medium Junction
    public double liftPos3 = motorTickPerMillimeter * 840; // High Junction
    public double liftPosConeStack2 = motorTickPerMillimeter * 30; // Lifts to the 2nd cone in the cone stack
    public double liftPosConeStack3 = motorTickPerMillimeter * 60; // Lifts to the 3rd cone in the cone stack
    public double liftPosConeStack4 = motorTickPerMillimeter * 85; // Lifts to the 4th cone in the cone stack
    public double liftPosConeStack5 = motorTickPerMillimeter * 125; // Lifts to the 5th cone in the cone stack
    public double liftPosTest = motorTickPerMillimeter * 100;

    // Constructor
    public Twin_Towers() {
    }

    // Initialization
    public void init(HardwareMap hwMap) {
        // Define and Initialize Motor
        liftMotor = hwMap.get(DcMotor.class, "lift_motor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Initialize arm encoder
    }

    // Method to move the arm
    public void towers_lift(double liftPos) {
        liftMotor.setTargetPosition(((int)liftPos));
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(.75);

        if (!liftMotor.isBusy()) {
            liftMotor.setPower(0);
        }
    }
}
