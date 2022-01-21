/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

//import org.firstinspires.ftc.teamcode.hardware.manipulators.Arm;

@TeleOp(name="TeleOp_Iterative", group="Iterative Opmode")

public class TeleOp_Iterative extends OpMode {

    //Declare OpMode members
    Provider robot = new Provider();

    //Class variables
    boolean changed = false;

    //Code to run once when the driver hits INIT
    @Override
    public void init() {
        //Initialize the robot
        robot.init(hardwareMap);

        //Initialize encoders
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Initialize arm encoder

        //Initialize servos
        robot.grabber.setPosition(robot.GRABBER_OPEN);

        //Tell the driver that initialization is complete
        telemetry.addData("Status", "Initialized");
    }

    //Code to run repeatedly after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        //robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Code to run once when the driver hits PLAY
    @Override
    public void start() {
        //Resets the runtime value to 0
        robot.runtime.reset();
    }

    //Code to run repeatedly after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        //Show the elapsed game time and wheel power
        telemetry.addData("Status", "Run Time: " + robot.runtime.toString());

        //DRIVE CODE
        //This uses basic math to combine motions and is easier to drive straight
        double drive  = +gamepad1.left_stick_y / 1.5 / robot.slowMo;
        double strafe = +gamepad1.left_stick_x / 1.5 / robot.slowMo;
        double turn   = -gamepad1.right_stick_x / 1.5 / robot.slowMo;

        //Send calculated power to wheels
        robot.driveLF.setPower( - drive + strafe - turn );
        robot.driveRF.setPower( - drive - strafe + turn );
        robot.driveLB.setPower( - drive - strafe - turn );
        robot.driveRB.setPower( - drive + strafe + turn );

        //Go Slow/Fast code

        boolean slowMoButton = gamepad1.y;


        if (slowMoButton && !changed) {
            if (robot.slowMo == 1) {
                robot.slowMo = 2;
            } else {
                robot.slowMo = 1;
            }
            changed = true;
        } else if (!slowMoButton) {
            changed = false;
        }

        //ARM CODE (No encoder)
        //Variables for arm
        boolean armUp = gamepad1.right_bumper;
        boolean armDown = gamepad1.left_bumper;

        if (armUp && !armDown) {
            robot.armMotor.setPower(1);
        }

        if (armDown && !armUp) {
            robot.armMotor.setPower(-1);
        }

        if (!armUp && !armDown) {
            robot.armMotor.setPower(0);
        }

        //ARM CODE(With encoder)
        //Variables for arm
        boolean setArmPos0 = gamepad2.a;
        boolean setArmPos1 = gamepad2.b;
        boolean setArmPos2 = gamepad2.x;
        boolean setArmPos3 = gamepad2.y;

        //Code to move arm
        if (setArmPos0) {
            arm_move(robot.armPos0);
        }

        if (setArmPos1) {
            arm_move(robot.armPos1);
        }

        if (setArmPos2) {
            arm_move(robot.armPos2);
        }

        if (setArmPos3) {
            arm_move(robot.armPos3);
        }


        //SERVO GRABBER CODE
        //Variable for grabber
        boolean closeGrabber = gamepad2.dpad_up;
        boolean openGrabber = gamepad2.dpad_down;
        double grabberOpen = robot.GRABBER_OPEN;
        double grabberClose = robot.GRABBER_CLOSE;

        //Code to move servo for grabber
        if (closeGrabber && !openGrabber) {
            robot.grabber.setPosition(grabberClose);
        }

        if (openGrabber && !closeGrabber) {
            robot.grabber.setPosition(grabberOpen);
        }

        //DUCK SPINNER CODE
        //Variables for duck spinner
        boolean spin = gamepad1.a;
        boolean spinBack = gamepad1.b;

        //Code to move servo
        if (spin) {
            robot.duckSpinner.setPower(1);
        } else {
            robot.duckSpinner.setPower(0);
        }
    }

    public void arm_move(int armPos) {
        robot.armMotor.setTargetPosition(armPos);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(1);

        while (robot.armMotor.isBusy()) {

        }

        robot.armMotor.setPower(0);
    }

    //Code to run once after the driver hits STOP
    @Override
    public void stop() {
        //Just cause
        telemetry.addData("BOT Status", "I AM A BOT!");
    }
}
