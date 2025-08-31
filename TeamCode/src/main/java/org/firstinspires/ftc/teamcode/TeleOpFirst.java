package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.PIDFParams;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOpFirst extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            drive.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        }
    }
}
