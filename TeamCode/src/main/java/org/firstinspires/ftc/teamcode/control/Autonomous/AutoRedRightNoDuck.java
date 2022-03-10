package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto Red Right No Duck", preselectTeleOp = "TeleOp_Iterative")
public class AutoRedRightNoDuck extends AutoBase {

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
        turn_left_time(0.4, 650);
        drive_forward_time(0.25, 175);
        open_grabber();
        sleep(750);
        drive_backward_time(0.4, 200);
        drive_backward_time(0.25, 200);
        strafe_right_time(0.25, 400);
        move_arm(robot.arm.armPos0);
        drive_forward_time(0.25, 100);
        strafe_left_time(0.75, 1200);
        strafe_left_time(0.25, 550);
        sleep(200);
        //Code that goes into the warehouse and parks in the corner
        drive_backward_time(0.4, 1100);
        strafe_right_time(0.75, 800);
        drive_backward_time(0.4, 500);

        //Done with Autonomous
        sleep(2000);
    }
}
