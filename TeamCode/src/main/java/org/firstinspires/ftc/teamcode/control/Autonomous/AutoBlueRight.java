package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Autonomous(name = "Auto BLUE Right", preselectTeleOp = "TeleOp_Iterative")
public class AutoBlueRight extends AutoBase {

    @Override
    public void runOpMode() {
        // Initialize the robot
        auto_init();
        SampleMecanumDrive rr_drive = new SampleMecanumDrive(hardwareMap);

        // Set positions
        // Coordinates are taken from center of bot
        // Junction naming starts from top left (from audience pov) and moves left and down. example: L2
        // Center to center distance of a tile: 23.5 in
        Pose2d startPose = new Pose2d(-38.000, 64.00, Math.toRadians(270));
        Pose2d junctionL5 = new Pose2d(-23.5-7.00, 47.5, Math.toRadians(0)); // Junction L4 is at (23.5,-47). Robot will have to be 6.65 in off of that point
        Pose2d junctionL7 = new Pose2d(-48, 23-6.00, Math.toRadians(90));
        Pose2d junctionM3 = new Pose2d(-23-8.25, 22, Math.toRadians(0));
        Pose2d junctionM3V2 = new Pose2d(-23-7.00, 23.50-4.75, Math.toRadians(45));
        Pose2d coneStack = new Pose2d(-70.00+7.25, 11.00, Math.toRadians(178));
        Pose2d coneStackC2 = new Pose2d(-70.00+6.25, 13.50, Math.toRadians(180));
        Pose2d signalZone1 = new Pose2d(-11.75, 11.75, Math.toRadians(90));
        Pose2d signalZone2 = new Pose2d(-35.50-3, 11.75, Math.toRadians(90));
        Pose2d signalZone3 = new Pose2d(-61.00, 12.50, Math.toRadians(90));

        rr_drive.setPoseEstimate(startPose);

        // Build trajectories
        Trajectory trajJunctionM3 = rr_drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(junctionM3)
                .addDisplacementMarker(0, () -> {
                    robot.towers.towers_lift(robot.towers.liftPos2);
                })
                .build();

        Trajectory trajConeStack5P1 = rr_drive.trajectoryBuilder(junctionM3)
                .lineToLinearHeading(signalZone2)
                .addDisplacementMarker(4, () -> {
                    robot.towers.towers_lift(robot.towers.liftPosConeStack5);
                })
                .build();

        Trajectory trajConeStack5P2 = rr_drive.trajectoryBuilder(signalZone2)
                .lineToLinearHeading(coneStack)
                .build();

        Trajectory trajConeStack4 = rr_drive.trajectoryBuilder(junctionM3V2)
                .lineToLinearHeading(coneStackC2)
                .addDisplacementMarker(4, () -> {
                    robot.towers.towers_lift(robot.towers.liftPosConeStack4);
                })
                .build();

        Trajectory trajJunctionM3FromStack = rr_drive.trajectoryBuilder(coneStack)
                .lineToLinearHeading(junctionM3V2)
                .build();

        Trajectory trajJunctionM3C2FromStack = rr_drive.trajectoryBuilder(coneStackC2)
                .lineToLinearHeading(new Pose2d(-23-8.00, 23.50-3.75, Math.toRadians(45)))
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
        Trajectory trajSignalZone = rr_drive.trajectoryBuilder(junctionM3V2)
                .lineToLinearHeading(parkingZone)
                .build();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Code that makes the robot move
        // Scores pre-loaded cone, picks up one more cone, scores second cone, parks
        // Cycle pre-load
        rr_drive.followTrajectory(trajJunctionM3);
        open_grabber();
        sleep(500);
        // Begin cycle 1
        rr_drive.followTrajectory(trajConeStack5P1);
        rr_drive.followTrajectory(trajConeStack5P2);
        close_grabber();
        sleep(500);
        lift_towers(robot.towers.liftPos2);
        rr_drive.followTrajectory(trajJunctionM3FromStack);
        open_grabber();
        sleep(500);
        // Begin cycle 2
        rr_drive.followTrajectory(trajConeStack4);
        close_grabber();
        sleep(500);
        lift_towers(robot.towers.liftPos2);
        rr_drive.followTrajectory(trajJunctionM3C2FromStack);
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
