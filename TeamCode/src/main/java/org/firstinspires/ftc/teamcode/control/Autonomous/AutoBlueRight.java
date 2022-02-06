package org.firstinspires.ftc.teamcode.control.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Auto Blue Right", preselectTeleOp = "TeleOp_Iterative")
public class AutoBlueRight extends AutoBase {

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

        //Done with Autonomous
        sleep(5000);
    }
}
