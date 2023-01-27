package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Autonomous(name = "Auto Red Right Vuforia", preselectTeleOp = "TeleOp_Iterative")
public class AutoRedRightVuforia extends AutoBaseVuforia {

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
        Pose2d coneStack = new Pose2d(70.5-7.00, -11, Math.toRadians(0));
        Pose2d signalZone1 = new Pose2d(11.75, -11.75, Math.toRadians(270));
        Pose2d signalZone2 = new Pose2d(35.25+2, -11.75, Math.toRadians(270));
        Pose2d signalZone3 = new Pose2d(60, -11.75, Math.toRadians(270));
        Vector2d vConeStack = new Vector2d(68.5-7.00, -11.75); // Isn't used so isn't updated

        // Potential alt auto cycle positions
        Pose2d backupPose = new Pose2d(23, -11.75, Math.toRadians(0));
        Pose2d altJunctionL2 = new Pose2d(49-5, -23+5, Math.toRadians(315));

        rr_drive.setPoseEstimate(startPose);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Code that finds which barcode the duck/shipping element is on
        detect_zone_pos();

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
        Trajectory trajJunctionL4 = rr_drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(junctionL4)
                .build();

        Trajectory trajConeStackP1 = rr_drive.trajectoryBuilder(trajJunctionL4.end(), true)
                .lineToLinearHeading(signalZone2)
                .addDisplacementMarker(4, () -> {
                    robot.towers.towers_lift(robot.towers.liftPosConeStack5);
                })
                .build();

        Trajectory trajConeStackP2 = rr_drive.trajectoryBuilder(trajConeStackP1.end(), true)
                .lineToLinearHeading(coneStack)
                .build();

        Trajectory trajJunctionL2P1 = rr_drive.trajectoryBuilder(trajConeStackP2.end(), true)
                .lineToLinearHeading(new Pose2d(60, -12.00, Math.toRadians(0)))
                .build();

        Trajectory trajJunctionL2P2 = rr_drive.trajectoryBuilder(trajJunctionL2P1.end(), true)
                .splineToLinearHeading(junctionL2, Math.toRadians(0))
                .build();

        Trajectory trajSignalZone = rr_drive.trajectoryBuilder(trajJunctionL2P2.end(), true)
                .lineToLinearHeading(parkingZone)
                .build();

        // Potential alt auto cycle trajectories
        Trajectory trajAltJunctionL2P1 = rr_drive.trajectoryBuilder(trajConeStackP2.end(), true)
                .lineToLinearHeading(backupPose)
                .build();

        Trajectory trajAltJunctionL2P2 = rr_drive.trajectoryBuilder(trajAltJunctionL2P1.end(), true)
                .lineToLinearHeading(altJunctionL2)
                .build();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Code that makes the robot move
        // Scores pre-loaded cone, picks up one more cone, scores second cone, parks
        lift_towers(robot.towers.liftPos1);
        rr_drive.followTrajectory(trajJunctionL4);
        open_grabber();
        sleep(500);
        rr_drive.followTrajectory(trajConeStackP1);
        rr_drive.followTrajectory(trajConeStackP2);
        close_grabber();
        sleep(500);
        lift_towers(robot.towers.liftPos1);
        rr_drive.followTrajectory(trajJunctionL2P1); // To try the alt path for this ad "Alt" to the trajectory
        rr_drive.followTrajectory(trajJunctionL2P2); // To try the alt path for this ad "Alt" to the trajectory
        open_grabber();
        rr_drive.followTrajectory(trajSignalZone);
        lift_towers(robot.towers.liftPos0);

        // Done with Autonomous
        sleep(2000);
    }
}
