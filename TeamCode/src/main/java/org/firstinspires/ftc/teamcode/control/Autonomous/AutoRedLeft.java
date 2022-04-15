package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto Red Left", preselectTeleOp = "TeleOp_Iterative")
public class AutoRedLeft extends AutoBase {

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
        // Code that goes to alliance shipping hub, drops off pre-loaded freight, and comes back
        drive_forward_time(0.25, 500);
        drive_forward_time(0.5, 850);
        move_arm(targetLevel);
        turn_right_time(0.4, 650);
        drive_forward_time(0.25, 175);
        open_grabber();
        sleep(750);
        drive_backward_time(0.5, 600);
        move_arm(robot.arm.armPos0);
        turn_right_time(0.4, 525);
        sleep(100);
        strafe_right_time(0.60, 500);
        strafe_right_time(0.25, 300);
        sleep(200);
        //Code that goes to the duck carousel and delivers the duck
        strafe_left_time(0.25, 200);
        drive_forward_time(0.5, 650);
        drive_forward_time(0.25, 1100);
        drive_forward_time(0.1, 400);
        spin_duck(-0.4,3500);
        //Code that parks in the storage
        drive_backward_time(0.4, 700);

        //Done with Autonomous
        sleep(2000);
    }
}
