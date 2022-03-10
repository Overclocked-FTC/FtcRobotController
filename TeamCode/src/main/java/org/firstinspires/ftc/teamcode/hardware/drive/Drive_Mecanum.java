package org.firstinspires.ftc.teamcode.hardware.drive;

import org.firstinspires.ftc.teamcode.control.Provider;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Drive_Mecanum {
    //Hardware Components
    public DcMotor driveLF = null; //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public DcMotor driveRF = null;
    public DcMotor driveLB = null;
    public DcMotor driveRB = null;

    //Variables
    public int slowMo = 1;

    //Constructor
    public Drive_Mecanum() {
    }

    public void init(HardwareMap hwMap) {
        //Define and Initialize Motors
        driveLF = hwMap.get(DcMotor.class, "left_front_drive");
        driveRF = hwMap.get(DcMotor.class, "right_front_drive");
        driveLB = hwMap.get(DcMotor.class, "left_back_drive");
        driveRB = hwMap.get(DcMotor.class, "right_back_drive");
        driveLF.setDirection(DcMotor.Direction.FORWARD);
        driveRF.setDirection(DcMotor.Direction.REVERSE);
        driveLB.setDirection(DcMotor.Direction.FORWARD);
        driveRB.setDirection(DcMotor.Direction.REVERSE);
        driveLF.setPower(0);
        driveRF.setPower(0);
        driveLB.setPower(0);
        driveRB.setPower(0);
        driveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
