package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Autonomous(name = "RR Auto Red Warehouse", preselectTeleOp = "TeleOp_Iterative")
public class RRAutoRedWarehouse extends AutoBase {

    @Override
    public void runOpMode() {
//Initialize the robot
        auto_init();
        SampleMecanumDrive rr_drive = new SampleMecanumDrive(hardwareMap);

        //Set positions
        Pose2d startPose = new Pose2d(11.811, -64.85827, Math.toRadians(90));
        Pose2d allyHubPose = new Pose2d(10.878, -23.622, Math.toRadians(180));
        Pose2d warehouseEntrancePose = new Pose2d(11.811, -64.85827, Math.toRadians(0));
        Pose2d frontWarehousePose = new Pose2d(35.433, -64.85827, Math.toRadians(0));
        Vector2d sideWarehousePose = new Vector2d(35.433, -41.23627);
        Vector2d backWarehousePose = new Vector2d(59.055, -41.23627);


        rr_drive.setPoseEstimate(startPose);

        //Build all trajectories
        Trajectory trajHub = rr_drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(allyHubPose)
                .build();

        Trajectory trajWarehouseEntrance = rr_drive.trajectoryBuilder(trajHub.end(), true)
                .lineToLinearHeading(warehouseEntrancePose)
                .addDisplacementMarker(.5, () -> {
                    robot.arm.arm_move(robot.arm.armPos0);
                })
                .build();

        Trajectory trajWarehouse = rr_drive.trajectoryBuilder(trajWarehouseEntrance.end())
                .lineToLinearHeading(frontWarehousePose)
                .splineTo(sideWarehousePose, Math.toRadians(90))
                .splineTo(backWarehousePose, Math.toRadians(180))
                .build();

//        Trajectory trajStorage = rr_drive.trajectoryBuilder(trajCarousel.end())
//                .lineToLinearHeading(storagePose)
//                .build();

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
