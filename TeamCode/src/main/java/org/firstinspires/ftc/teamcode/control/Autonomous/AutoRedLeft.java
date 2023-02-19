package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Autonomous(name = "Auto RED Left", preselectTeleOp = "TeleOp_Iterative")
public class AutoRedLeft extends AutoBase {

    @Override
    public void runOpMode() {
        // Initialize the robot
        auto_init();
        SampleMecanumDrive rr_drive = new SampleMecanumDrive(hardwareMap);

        // Set positions
        // Coordinates are taken from center of bot
        // Junction naming starts from top left (from audience pov) and moves left and down. example: L2
        // Center to center distance of a tile: 23.5 in
        Pose2d startPose = new Pose2d(-35.125, -64.00, Math.toRadians(90)); // -36.375
        Pose2d junctionM4 = new Pose2d(-23.5-9.00, -23.5+0.5, Math.toRadians(0));
        Pose2d junctionM4V2 = new Pose2d(-23.5-6.00, -23.5+3.50, Math.toRadians(315));
        Pose2d coneStack = new Pose2d(-70.0+6.75, -12.70, Math.toRadians(180));
        Pose2d coneStackC2 = new Pose2d(-70.00+7.00, -14.75, Math.toRadians(180));
        Pose2d signalZone1 = new Pose2d(-60.00, -14.50, Math.toRadians(270));
        Pose2d signalZone2 = new Pose2d(-35.25-2, -14.00, Math.toRadians(270));
        Pose2d signalZone3 = new Pose2d(-13.50, -12.50, Math.toRadians(270));

        rr_drive.setPoseEstimate(startPose);

        // Build trajectories
        Trajectory trajJunctionM4 = rr_drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(junctionM4)
                .addDisplacementMarker(0, () -> {
                    robot.towers.towers_lift(robot.towers.liftPos2);
                })
                .build();

        Trajectory trajConeStack5P1 = rr_drive.trajectoryBuilder(junctionM4)
                .lineToLinearHeading(signalZone2)
                .addDisplacementMarker(4, () -> {
                    robot.towers.towers_lift(robot.towers.liftPosConeStack5);
                })
                .build();

        Trajectory trajConeStack5P2 = rr_drive.trajectoryBuilder(signalZone2)
                .lineToLinearHeading(coneStack)
                .build();

        Trajectory trajConeStack4 = rr_drive.trajectoryBuilder(junctionM4V2)
                .lineToLinearHeading(coneStackC2) // This may need to be coneStackC2
                .addDisplacementMarker(4, () -> {
                    robot.towers.towers_lift(robot.towers.liftPosConeStack4);
                })
                .build();

        Trajectory trajJunctionM4FromStack = rr_drive.trajectoryBuilder(coneStack)
                .lineToLinearHeading(junctionM4V2)
                .build();

        Trajectory trajJunctionM4C2FromStack = rr_drive.trajectoryBuilder(coneStackC2) // This may need to be coneStackC2
                .lineToLinearHeading(new Pose2d(-23.5-6.60, -23.5+0.50, Math.toRadians(310)))
                .build();

        // Wait for the game to start (driver presses PLAY)
        init_detection();

        // Code that finds which barcode the duck/shipping element is on
        detect_april_tag();

        // Set the target zone
        Pose2d parkingZone = null;
        if (targetZone == "Zone 1") {
            parkingZone = signalZone1;
        } else if (targetZone == "Zone 2") {
            parkingZone = signalZone2;
        } else if (targetZone == "Zone 3") {
            parkingZone = signalZone3;
        }

        // Build the last trajectory
        // This trajectory has to be built after the play button has been pressed
        Trajectory trajSignalZone = rr_drive.trajectoryBuilder(junctionM4V2)
                .lineToLinearHeading(parkingZone)
                .build();


        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Code that makes the robot move
        // Scores pre-loaded cone, picks up one more cone, scores second cone, parks
        // Cycle pre-load
        rr_drive.followTrajectory(trajJunctionM4);
        open_grabber();
        sleep(500);
        // Begin cycle 1
        rr_drive.followTrajectory(trajConeStack5P1);
        rr_drive.followTrajectory(trajConeStack5P2);
        close_grabber();
        sleep(500);
        lift_towers(robot.towers.liftPos2);
        rr_drive.followTrajectory(trajJunctionM4FromStack);
        open_grabber();
        sleep(500);
        // Begin cycle 2
        rr_drive.followTrajectory(trajConeStack4);
        close_grabber();
        sleep(500);
        lift_towers(robot.towers.liftPos2);
        rr_drive.followTrajectory(trajJunctionM4C2FromStack);
        open_grabber();
        sleep(500);
        // Add more cycles here
        // Drive to signal zone
        rr_drive.followTrajectory(trajSignalZone);
        lift_towers(robot.towers.liftPos0);

        // Done with Autonomous
        sleep(2000);
    }
}
