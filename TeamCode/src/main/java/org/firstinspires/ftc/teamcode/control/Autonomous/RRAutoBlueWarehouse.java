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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

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

@Autonomous(name = "RR Auto Blue Warehouse", preselectTeleOp = "TeleOp_Iterative")
public class RRAutoBlueWarehouse extends AutoBase {

    @Override
    public void runOpMode() {
        //Initialize the robot
        auto_init();
        SampleMecanumDrive rr_drive = new SampleMecanumDrive(hardwareMap);

        //Set positions
        Pose2d startPose = new Pose2d(11.811, 64.85827, Math.toRadians(-90));
        Pose2d allyHubPose = new Pose2d(10.878, 23.622, Math.toRadians(180));
        Vector2d warehouseEntrancePose = new Vector2d(11.811, 64.85827);
        Vector2d frontWarehousePose = new Vector2d(42.0, 64.85827);

        rr_drive.setPoseEstimate(startPose);

        //Build all trajectories
        Trajectory trajHub = rr_drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(allyHubPose)
                .build();

        Trajectory trajWarehouseEntrance = rr_drive.trajectoryBuilder(trajHub.end(), true)
                .lineToConstantHeading(warehouseEntrancePose)
                .addDisplacementMarker(3, () -> {
                    robot.arm.arm_move(robot.arm.armPos0);
                })
                .build();

        Trajectory trajWarehouse = rr_drive.trajectoryBuilder(trajWarehouseEntrance.end(), true)
                .lineToConstantHeading(frontWarehousePose)
                .build();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Code that finds which barcode the duck/shipping element is on
        detect_barcode_pos();

        //Code that makes the robot move
        // Code that goes to alliance shipping hub, drops off pre-loaded freight, and comes back
        move_arm(targetLevel);
        rr_drive.followTrajectory(trajHub);
        open_grabber();
        sleep(750);
        rr_drive.followTrajectory(trajWarehouseEntrance);
        rr_drive.followTrajectory(trajWarehouse);

        //Done with Autonomous
        sleep(2000);
    }
}
