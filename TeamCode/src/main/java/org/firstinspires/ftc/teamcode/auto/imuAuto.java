package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.easyopencv.OpenCvCamera;


@Autonomous(name = "IMU Auto 3")
public class imuAuto extends LinearOpMode {

    //IMU
    private IMU imu_IMU;
    private BNO055IMU imu;
    float yaw;
    YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;

    //Drivetrain
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    int bl;
    int br;
    int fl;
    int fr;
    int encoderTicks;

    //private DcMotorEx slideMotor;
    private Servo claw;
    private Servo rotation;

   //Limit Switch
    private DigitalChannel slideLimitSwitch;

    int target_Position;
    double slidePower = 0.53;
    double speed;





    //CAMERA
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


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

        //Telemetry
        orientation = imu_IMU.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw (Z)", JavaUtil.formatNumber(orientation.getYaw(AngleUnit.DEGREES), 2));
        telemetry.update();



        waitForStart();


        telemetry.addLine("Init.... Press play");
        telemetry.update();

        while (opModeIsActive()) {

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
            sleep(5000);
            moveForward(1000);
        }
    }

    private void moveForward(int encoderTicks) {
        orientation = imu_IMU.getRobotYawPitchRollAngles();
        speed = 0.6;
        if (yaw < -5) {
            motorBackLeft.setPower(speed-0.1);
            motorBackRight.setPower(speed+0.1);
            motorFrontLeft.setPower(speed-0.1);
            motorFrontRight.setPower(speed+0.1);

        } else if (yaw > 5) {
            motorBackLeft.setPower(speed+0.1);
            motorBackRight.setPower(speed-0.1);
            motorFrontLeft.setPower(speed+0.1);
            motorFrontRight.setPower(speed-0.1);
        } else {
            motorBackLeft.setPower(speed);
            motorBackRight.setPower(speed);
            motorFrontLeft.setPower(speed);
            motorFrontRight.setPower(speed);
        }
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        bl = motorBackLeft.getCurrentPosition();
        br = motorBackRight.getCurrentPosition();
        fl = motorFrontLeft.getCurrentPosition();
        fr = motorFrontRight.getCurrentPosition();

        telemetry.addData("Current Positon ", fl);
        telemetry.addData("Target", fl+encoderTicks);
        telemetry.update();

        motorFrontRight.setTargetPosition(br+encoderTicks);
        motorFrontLeft.setTargetPosition(fl+encoderTicks);
        motorBackLeft.setTargetPosition(bl+encoderTicks);
        motorBackRight.setTargetPosition(br+encoderTicks);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}