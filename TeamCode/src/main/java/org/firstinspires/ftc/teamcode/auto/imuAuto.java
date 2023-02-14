package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;



@TeleOp
public class imuAuto extends LinearOpMode {
    //IMU
    private IMU imu_IMU;

    //Motors
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;

    int encoderTicks;

    //Speed
    double speed = 0.6;

    int activate;
    int reset;


    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        imu_IMU = hardwareMap.get(IMU.class, "imu");

        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        // Prompt user to press start button.
        telemetry.addData("IMU ", "Press start to continue...");
        telemetry.update();


        //Direction
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();


        telemetry.addLine("Init.... Press play");
        telemetry.update();

        while (opModeIsActive()) {

            moveForward(1000);

            telemetry.addData("Yaw", "Press Circle or B on Gamepad to reset.");
            // Check to see if reset yaw is requested.
            if (gamepad2.b) {
                imu_IMU.resetYaw();
            }
            orientation = imu_IMU.getRobotYawPitchRollAngles();
            angularVelocity = imu_IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
            // Display yaw, pitch, and roll.
            telemetry.addData("Yaw (Z)", JavaUtil.formatNumber(orientation.getYaw(AngleUnit.DEGREES), 2));
            telemetry.addData("Pitch (X)", JavaUtil.formatNumber(orientation.getPitch(AngleUnit.DEGREES), 2));
            telemetry.addData("Roll (Y)", JavaUtil.formatNumber(orientation.getRoll(AngleUnit.DEGREES), 2));
            // Display angular velocity.
            telemetry.addData("Yaw (Z) velocity", JavaUtil.formatNumber(angularVelocity.zRotationRate, 2));
            telemetry.addData("Pitch (X) velocity", JavaUtil.formatNumber(angularVelocity.xRotationRate, 2));
            telemetry.addData("Roll (Y) velocity", JavaUtil.formatNumber(angularVelocity.yRotationRate, 2));
            telemetry.update();


        private void moveForward(int encoder_Ticks) {

            motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

                encoderTicks = encoder_Ticks;

                bl = motorBackLeft.getCurrentPosition() + encoderTicks;
                br = motorBackRight.getCurrentPosition() + encoderTicks;
                fl = motorFrontLeft.getCurrentPosition() + encoderTicks;
                fr = motorFrontRight.getCurrentPosition() + encoderTicks;

                telemetry.addData(" Target Position of Front Left", fl);

                motorFrontLeft.setTargetPosition(fl);
                motorFrontRight.setTargetPosition(fr);
                motorBackLeft.setTargetPosition(bl);
                motorBackRight.setTargetPosition(br);

                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorBackLeft.setPower(speed);
                motorBackRight.setPower(speed);
                motorFrontLeft.setPower(speed);
                motorFrontRight.setPower(speed);
                sleep(1000);
            }
        }
    }
}