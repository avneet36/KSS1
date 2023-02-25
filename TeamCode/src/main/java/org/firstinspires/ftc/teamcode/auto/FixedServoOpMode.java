package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class FixedServoOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        ServoArm arm = new ServoArm(hardwareMap) {
            @Override
            public void runOpMode() throws InterruptedException {

            }
        };

        waitForStart();

        while (opModeIsActive()) {
            arm.setPosition(-gamepad1.left_stick_y);
        }
    }
}
