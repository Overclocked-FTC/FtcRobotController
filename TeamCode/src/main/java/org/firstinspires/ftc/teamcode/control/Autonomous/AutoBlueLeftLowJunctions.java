package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
@Disabled
@Autonomous(name = "Auto Blue Left Low Junctions", preselectTeleOp = "TeleOp_Iterative")
public class AutoBlueLeftLowJunctions extends AutoBase {

    // TODO: CURRENTLY SCUFFED

    @Override
    public void runOpMode() {
        // Initialize the robot
        auto_init();
        SampleMecanumDrive rr_drive = new SampleMecanumDrive(hardwareMap);

        // Set positions
        // Coordinates are taken from center of bot
        // Junction naming starts from top left (from audience pov) and moves left and down. example: L2
        // Center to center distance of a tile: 23.5 in
        Pose2d startPose = new Pose2d(40.000, 64.00, Math.toRadians(270));
        Pose2d junctionL3 = new Pose2d(23.5+7.00, 47, Math.toRadians(180)); // Junction L4 is at (23.5,-47). Robot will have to be 6.65 in off of that point
        Pose2d junctionL1 = new Pose2d(49, 23-6.00, Math.toRadians(90));
        Pose2d coneStack = new Pose2d(69.5-7.00, 12, Math.toRadians(0));
        Pose2d signalZone1 = new Pose2d(11.75, -11.75, Math.toRadians(0)); // Isn't used so isn't updated
        Pose2d signalZone2 = new Pose2d(35.25+2, 11.75, Math.toRadians(0));
        Pose2d signalZone3 = new Pose2d(58.75, -11.75, Math.toRadians(0)); // Isn't used so isn't updated
        Vector2d vConeStack = new Vector2d(68.5-7.00, -11.75); // Isn't used so isn't updated

        rr_drive.setPoseEstimate(startPose);

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

        // Build trajectories
        Trajectory trajJunctionL3 = rr_drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(junctionL3)
                .build();

        Trajectory trajConeStackP1 = rr_drive.trajectoryBuilder(trajJunctionL3.end(), true)
                .lineToLinearHeading(signalZone2)
                .addDisplacementMarker(4, () -> {
                    robot.towers.towers_lift(robot.towers.liftPosConeStack5);
                })
                .build();

        Trajectory trajConeStackP2 = rr_drive.trajectoryBuilder(trajConeStackP1.end(), true)
                .lineToLinearHeading(coneStack)
                .build();

        Trajectory trajJunctionL1P1 = rr_drive.trajectoryBuilder(trajConeStackP2.end(), true)
                .lineToLinearHeading(new Pose2d(60, 11.75, Math.toRadians(0)))
                .build();

        Trajectory trajJunctionL1P2 = rr_drive.trajectoryBuilder(trajJunctionL1P1.end(), true)
                .splineToLinearHeading(junctionL1, Math.toRadians(0))
                .build();

        Trajectory trajSignalZone = rr_drive.trajectoryBuilder(trajJunctionL1P2.end(), true)
                .lineToLinearHeading(parkingZone)
                .build();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Code that makes the robot move
        // Scores pre-loaded cone, picks up one more cone, scores second cone, parks
        lift_towers(robot.towers.liftPos1);
        rr_drive.followTrajectory(trajJunctionL3);
        open_grabber();
        sleep(500);
        rr_drive.followTrajectory(trajConeStackP1);
        rr_drive.followTrajectory(trajConeStackP2);
        close_grabber();
        sleep(500);
        lift_towers(robot.towers.liftPos1);
        rr_drive.followTrajectory(trajJunctionL1P1);
        rr_drive.followTrajectory(trajJunctionL1P2);
        open_grabber();
        rr_drive.followTrajectory(trajSignalZone);
        lift_towers(robot.towers.liftPos0);

        // Done with Autonomous
        sleep(2000);
    }
}
