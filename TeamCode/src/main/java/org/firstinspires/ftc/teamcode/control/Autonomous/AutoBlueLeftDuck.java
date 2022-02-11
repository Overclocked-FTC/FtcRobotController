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

package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue Left Duck", preselectTeleOp = "TeleOp_Iterative")
public class AutoBlueLeftDuck extends AutoBase {

    @Override
    public void runOpMode() {
        //Initialize the robot
        auto_init();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Code that finds which barcode the duck/shipping element is on
        detect_barcode_pos();

        //Code that makes the robot move
        //Code that goes to alliance shipping hub, drops off pre-loaded freight, and comes back
        drive_forward_time(0.25, 500);
        drive_forward_time(0.5, 850);
        move_arm(targetLevel);
        turn_right_time(0.4, 650);
        drive_forward_time(0.25, 175);
        open_grabber();
        sleep(750);
        drive_backward_time(0.5, 250);
        move_arm(robot.armPos0);
        strafe_right_time(0.75, 1100);
        strafe_right_time(0.25, 400);
        sleep(200);
        //Code that goes to the duck carousel, delivers duck, and comes back
        strafe_left_time(0.25, 500);
        sleep(200);
        drive_forward_time(0.25, 500);
        drive_forward_time(0.5, 1400);
        strafe_right_time(0.25, 1000);
        sleep(200);
        strafe_left_time(0.25, 500);
        drive_forward_time(0.25, 1000);
        spin_duck(1,4000);
        drive_backward_time(0.25, 500);
        drive_backward_time(0.5, 2000);
        strafe_right_time(0.25, 1000);
        sleep(200);
        //Code that goes into the warehouse and turns to face freight
        drive_backward_time(0.5, 700);
        strafe_left_time(0.75, 250);
        turn_left_time(0.4, 1300);
        //Code that goes into the warehouse and parks in the corner
//        drive_backward_time(0.5, 1000);
//        strafe_left_time(0.75, 1000);
//        drive_backward_time(0.5, 500);

        //Done with Autonomous
        sleep(5000);
    }
}
