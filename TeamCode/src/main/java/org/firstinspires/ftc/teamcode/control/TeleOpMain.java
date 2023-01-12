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

import org.firstinspires.ftc.teamcode.hardware.manipulators.Grabber_911;

//import org.firstinspires.ftc.teamcode.hardware.manipulators.Arm;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp_Iterative", group="Iterative Opmode")

public class TeleOpMain extends OpMode {

    //Declare OpMode members
    Provider robot = new Provider();

    //Class variables
    boolean changed = false;

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
        double max;
        double drive  = -gamepad1.left_stick_y / 1.2 / robot.drive.slowMo;
        double strafe =  gamepad1.left_stick_x / 1.2 / robot.drive.slowMo;
        double turn   =  gamepad1.right_stick_x / 1.2 / robot.drive.slowMo;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower   = drive - strafe + turn;
        double rightBackPower  = drive + strafe - turn;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        //Send calculated power to wheels
        robot.drive.driveLF.setPower(leftFrontPower);
        robot.drive.driveRF.setPower(rightFrontPower);
        robot.drive.driveLB.setPower(leftBackPower);
        robot.drive.driveRB.setPower(rightBackPower);

        //Go Slow/Fast code
        boolean slowMoButton = gamepad1.y;

        if (slowMoButton && !changed) {
            if (robot.drive.slowMo == 1) {
                robot.drive.slowMo = 2;
            } else {
                robot.drive.slowMo = 1;
            }
            changed = true;
        } else if (!slowMoButton) {
            changed = false;
        }

        //ARM CODE (with encoder)
        //Variables for arm
        boolean setArmPos0 = gamepad2.a;
        boolean setArmPos1 = gamepad2.b;
        boolean setArmPos2 = gamepad2.x;
        boolean setArmPos3 = gamepad2.y;
        boolean setArmPosTest = gamepad2.right_stick_button;

        //Code to move arm
        if (setArmPos0) {
            robot.towers.arm_move(robot.towers.liftPos0);
        }

        if (setArmPos1) {
            robot.towers.arm_move(robot.towers.liftPos1);
        }

        if (setArmPos2) {
            robot.towers.arm_move(robot.towers.liftPos2);
        }

        if (setArmPos3) {
            robot.towers.arm_move(robot.towers.liftPos3);
        }

        if (setArmPosTest) {
            robot.towers.arm_move(robot.towers.liftPosTest);
        }


        //ARM CODE (without encoder)
        //Variables for arm
        boolean moveArmUp = gamepad2.dpad_right;
        boolean moveArmDown = gamepad2.dpad_left;

        //Code to move arm
        if (moveArmUp && !moveArmDown) {
            robot.towers.arm_move(robot.towers.liftMotor.getCurrentPosition() + robot.towers.motorTickPerMillimeter * 30);
        } else if (moveArmDown && !moveArmUp) {
            robot.towers.arm_move(robot.towers.liftMotor.getCurrentPosition() - robot.towers.motorTickPerMillimeter* 30);
        }

        //SERVO GRABBER CODE
        //Variable for grabber
        boolean closeGrabber = gamepad2.dpad_up;
        boolean openGrabber = gamepad2.dpad_down;
        double grabberOpen = Grabber_911.GRABBER_OPEN;
        double grabberClose = Grabber_911.GRABBER_CLOSE;

        //Code to move servo for grabber
        if (closeGrabber && !openGrabber) {
            robot.claw.grabber.setPosition(grabberClose);
        }

        if (openGrabber && !closeGrabber) {
            robot.claw.grabber.setPosition(grabberOpen);
        }
    }

    //Code to run once after the driver hits STOP
    @Override
    public void stop() {
        //Just cause
        telemetry.addData("BOT Status", "I AM A BOT!");
    }
}
