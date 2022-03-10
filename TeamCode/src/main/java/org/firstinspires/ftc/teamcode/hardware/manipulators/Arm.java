package org.firstinspires.ftc.teamcode.hardware.manipulators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.Provider;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

public class Arm {
    //Hardware Components
    public DcMotor armMotor = null;

    //Variables
    static final double MOTOR_TICK_COUNT = 537.7;
    public double motorDegree = MOTOR_TICK_COUNT / 360;
    public double wormGearRatio = 28;
    public double armPos0 = 0; //Arm is down all the way
    public double armPos1 = motorDegree * wormGearRatio * 50; //Last number is degrees of rotation. Shipping hub 1st layer
    public double armPos2 = motorDegree * wormGearRatio * 82; //Shipping hub 2nd layer
    public double armPos3 = motorDegree * wormGearRatio * 115; //Shipping hub 3rd layer

    //Constructor
    public Arm() {
    }

    //Initialization
    public void init(HardwareMap hwMap) {
        //Define and Initialize Motor
        armMotor = hwMap.get(DcMotor.class, "arm_motor");
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Initialize arm encoder
    }

    //Method to move the arm
    public void arm_move(double armPos) {
        armMotor.setTargetPosition(((int)armPos));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);

        if (!armMotor.isBusy()) {
            armMotor.setPower(0);
        }
    }
}
