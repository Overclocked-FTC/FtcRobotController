package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Autonomous(name = "Auto RED Right", preselectTeleOp = "TeleOp_Iterative")
public class AutoRedRight extends AutoBase {

    @Override
    public void runOpMode() {

        // Initialize the robot
        auto_init();
        SampleMecanumDrive rr_drive = new SampleMecanumDrive(hardwareMap);

        // Set positions
        // Coordinates are taken from center of bot
        // Junction naming starts from top left (from audience pov) and moves left and down. example: L2
        // Center to center distance of a tile: 23.5 in
        Pose2d startPose = new Pose2d(38.000, -63.00, Math.toRadians(90));
        Pose2d junctionL4 = new Pose2d(23.5+7.00, -46, Math.toRadians(180)); // Junction L4 is at (23.5,-47). Robot will have to be 6.65 in off of that point
        Pose2d junctionL2 = new Pose2d(49, -23+6.00, Math.toRadians(270));
        Pose2d junctionM2 = new Pose2d(23+7.75, -21, Math.toRadians(180));
        Pose2d junctionM2V2 = new Pose2d(23+5.20, -22+4.00, Math.toRadians(225));
        Pose2d coneStack = new Pose2d(70.00-7.50, -12.50, Math.toRadians(0));
        Pose2d coneStackC2 = new Pose2d(71.00-7.75, -14.75, Math.toRadians(0));
        Pose2d signalZone1 = new Pose2d(11.75, -12.00, Math.toRadians(270));
        Pose2d signalZone2 = new Pose2d(35.25+2, -12.50, Math.toRadians(270));
        Pose2d signalZone3 = new Pose2d(63, -12.50, Math.toRadians(270));

        rr_drive.setPoseEstimate(startPose);

        // Build trajectories
        Trajectory trajJunctionM2 = rr_drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(junctionM2)
                .addDisplacementMarker(0, () -> {
                    robot.towers.towers_lift(robot.towers.liftPos2);
                })
                .build();

        Trajectory trajConeStack5P1 = rr_drive.trajectoryBuilder(junctionM2)
                .lineToLinearHeading(signalZone2)
                .addDisplacementMarker(4, () -> {
                    robot.towers.towers_lift(robot.towers.liftPosConeStack5);
                })
                .build();

        Trajectory trajConeStack5P2 = rr_drive.trajectoryBuilder(signalZone2)
                .lineToLinearHeading(coneStack)
                .build();

        Trajectory trajConeStack4 = rr_drive.trajectoryBuilder(junctionM2V2)
                .lineToLinearHeading(coneStackC2)
                .addDisplacementMarker(4, () -> {
                    robot.towers.towers_lift(robot.towers.liftPosConeStack4);
                })
                .build();

        Trajectory trajJunctionM2FromStack = rr_drive.trajectoryBuilder(coneStack)
                .lineToLinearHeading(junctionM2V2)
                .build();

        Trajectory trajJunctionM2C2FromStack = rr_drive.trajectoryBuilder(coneStackC2)
                .lineToLinearHeading(new Pose2d(23+5.20, -25+4.50, Math.toRadians(225)))
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
        Trajectory trajSignalZone = rr_drive.trajectoryBuilder(junctionM2V2)
                .lineToLinearHeading(parkingZone)
                .build();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Code that makes the robot move
        // Scores pre-loaded cone, picks up one more cone, scores second cone, parks
        // Cycle pre-load
        rr_drive.followTrajectory(trajJunctionM2);
        open_grabber();
        sleep(500);
        // Begin cycle 1
        rr_drive.followTrajectory(trajConeStack5P1);
        rr_drive.followTrajectory(trajConeStack5P2);
        close_grabber();
        sleep(500);
        lift_towers(robot.towers.liftPos2);
        rr_drive.followTrajectory(trajJunctionM2FromStack);
        open_grabber();
        sleep(500);
        // Begin cycle 2
        rr_drive.followTrajectory(trajConeStack4);
        close_grabber();
        sleep(500);
        lift_towers(robot.towers.liftPos2);
        rr_drive.followTrajectory(trajJunctionM2C2FromStack);
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
