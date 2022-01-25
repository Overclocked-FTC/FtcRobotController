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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
Autonomous Objective
Objective: Find which level of the shipping hub the crate needs to get on and place the crate on the
hub. Drive over to the warehouse and park inside of it.
Steps to complete the objective:
1. Use camera to scan where the team shipping element is
2. Drive forward plowing through the team shipping element until in line with the shipping hub
3. Turn 90 degrees to the left to face shipping hub
4. Raise arm to correct position to put the crate on the shipping hub
5. Drive forward until the grabber is above the shipping hub
6. Drop the crate
7. Back up away from the shipping hub
8. Lower the arm back to pos0
9. Strafe back to the side of the wall, hitting wall to align with it
10. drive into the warehouse
11. Strafe into the corner of the warehouse to get out of the way of alliance team's autonomous
12. Done
 */

@Autonomous(name = "Auto Linear")

public class AutoOp_Linear extends LinearOpMode {

    // Declare OpMode members.
    Provider robot = new Provider();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Initialize the robot
        robot.init(hardwareMap);

        //Initialize encoders
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Initialize arm encoder

        //Initialize servos
        robot.grabber.setPosition(robot.GRABBER_CLOSE);

        //Tell that everything has been initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Code that makes the robot move
        drive_forward_time(0.25, 500);
        drive_forward_time(0.5, 1200);
        move_arm(robot.armPos3);
        turn_right_time(0.5, 600);
        open_grabber();
        drive_backward_time(0.5, 250);
        move_arm(robot.armPos0);
        strafe_right_time(0.75, 1500);
        strafe_right_time(0.25, 250);
        sleep(500);
        drive_backward_time(0.5, 1000);
        strafe_left_time(0.75, 1000);
        drive_backward_time(0.5, 500);

        //Done with Autonomous
        sleep(5000);
    }

    //Here are all the methods used to drive the robot in auto
    public void drive_forward(double power) {
        robot.driveLF.setPower(power);
        robot.driveRF.setPower(power);
        robot.driveLB.setPower(power);
        robot.driveRB.setPower(power);
    }

    public void drive_forward_time(double power, long time) {
        drive_forward(power);
        drive_time(time);
        stop_motors();
    }

    public void drive_backward(double power) {
        robot.driveLF.setPower(-power);
        robot.driveRF.setPower(-power);
        robot.driveLB.setPower(-power);
        robot.driveRB.setPower(-power);
    }

    public void drive_backward_time(double power, long time) {
        drive_backward(power);
        drive_time(time);
        stop_motors();
    }

    public void strafe_left(double power) {
        robot.driveLF.setPower(-power);
        robot.driveRF.setPower(power);
        robot.driveLB.setPower(power);
        robot.driveRB.setPower(-power);
    }

    public void strafe_left_time(double power, long time) {
        strafe_left(power);
        drive_time(time);
        stop_motors();
    }

    public void strafe_right(double power) {
        robot.driveLF.setPower(power);
        robot.driveRF.setPower(-power);
        robot.driveLB.setPower(-power);
        robot.driveRB.setPower(power);
    }

    public void strafe_right_time(double power, long time) {
        strafe_right(power);
        drive_time(time);
        stop_motors();
    }

    public void turn_left(double power) {
        robot.driveLF.setPower(-power);
        robot.driveRF.setPower(power);
        robot.driveLB.setPower(-power);
        robot.driveRB.setPower(power);
    }

    public void turn_left_time(double power, long time) {
        turn_left(power);
        drive_time(time);
        stop_motors();
    }

    public void turn_right(double power) {
        robot.driveLF.setPower(power);
        robot.driveRF.setPower(-power);
        robot.driveLB.setPower(power);
        robot.driveRB.setPower(-power);
    }

    public void turn_right_time(double power, long time) {
        turn_right(power);
        drive_time(time);
        stop_motors();
    }

    public void move_arm(double position) {
        robot.arm_move(position);
        while (opModeIsActive() && robot.armMotor.isBusy()){
            telemetry.addData("Action", "moving arm");
            telemetry.update();
        }
        robot.armMotor.setPower(0);
    }

    public void open_grabber() {
        robot.grabber.setPosition(Provider.GRABBER_OPEN);
        telemetry.addData("Action", "opened grabber");
        telemetry.update();
    }

    public void close_grabber() {
        robot.grabber.setPosition(Provider.GRABBER_CLOSE);
        telemetry.addData("Action", "closed grabber");
        telemetry.update();

    }

    public void drive_time(long time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < time)) {
            telemetry.addData("Time", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void stop_motors() {
        robot.driveLF.setPower(0);
        robot.driveRF.setPower(0);
        robot.driveLB.setPower(0);
        robot.driveRB.setPower(0);
    }
}
