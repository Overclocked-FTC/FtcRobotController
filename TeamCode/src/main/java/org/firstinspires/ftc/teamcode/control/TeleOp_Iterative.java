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

//import org.firstinspires.ftc.teamcode.hardware.manipulators.Arm;

@TeleOp(name="TeleOp_Iterative", group="Iterative Opmode")

public class TeleOp_Iterative extends OpMode {

    //Declare OpMode members
    Provider robot = new Provider();
    double grabberPosition = robot.GRABBER_HOME;
    final double GRABBER_SPEED = 0.8;

    //Code to run once when the driver hits INIT
    @Override
    public void init() {
        //Initialize the robot
        robot.init(hardwareMap);

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
        double drive  = +gamepad1.left_stick_y;
        double strafe = +gamepad1.left_stick_x ;
        double turn   = -gamepad1.right_stick_x;

        //Send calculated power to wheels
        //robot.driveLF.setPower( - drive + strafe - turn );
        robot.driveRF.setPower( - drive - strafe + turn );
        robot.driveLB.setPower( - drive - strafe - turn );
        robot.driveRB.setPower( - drive + strafe + turn );

        //ARM CODE
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

        //NEW ARM CODE
        if (armUp && !armDown) {
        }

        //SERVO GRABBER CODE
        //Variable for grabber
        boolean closeGrabber = gamepad1.dpad_up;
        boolean openGrabber = gamepad1.dpad_down;

        //Code to move servo for grabber
        if (closeGrabber) {
            grabberPosition += GRABBER_SPEED;
            robot.grabber.setPosition(grabberPosition);
        } else if (openGrabber) {
            grabberPosition -= GRABBER_SPEED;
            robot.grabber.setPosition(grabberPosition);
        }

        //DUCK SPINNER CODE
        //Variables for duck spinner
        boolean spin = gamepad1.a;
        boolean spinBack = gamepad1.b;

        //Code to move servo
        if (spin) {
            robot.duckSpinner.setPower(1);
        }// else {
//            robot.duckSpinner.setPower(0.5);
//        }




    }

    //Code to run once after the driver hits STOP
    @Override
    public void stop() {
        //Just cause
        telemetry.addData("BOT Status", "I AM A BOT!");
    }
}
