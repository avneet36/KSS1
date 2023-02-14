package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


import java.util.ArrayList;

@Autonomous(name = "IMU Test Auto ")
public class imuTestAuto extends LinearOpMode {

    private IMU imu_IMU;
    private BNO055IMU imu;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;


    //private DcMotorEx slideMotor;
    private Servo claw;
    private Servo rotation;

    private DigitalChannel slideLimitSwitch;


    int encoderTicks;
    int target_Position;
    double slidePower = 0.53;
    double speed;
    int bl;
    int br;
    int fl;
    int fr;

    float yaw;
    YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;


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

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int RIGHT = 3;
    int MIDDLE = 2;

    AprilTagDetection tagOfInterest = null;




    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");

        claw = hardwareMap.get(Servo.class, "claw");
        rotation = hardwareMap.get(Servo.class, "rotation");
        //slideMotor = hardwareMap.get(DcMotorEx.class, "slide");

        slideLimitSwitch = hardwareMap.get(DigitalChannel.class, "slideLimitSwitch");

        //slideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        // Put initialization blocks here.
        speed = 0.6;
        double slideMotorVelocity = 0;

        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        yaw = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;


        imu_IMU = hardwareMap.get(IMU.class, "imu");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        // Prompt user to press start button.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();



        //slideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) ;
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */

        waitForStart();





        //clawServo.setPosition(0);

        if (opModeIsActive()) {

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

            //limitSwitch();

            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            /* Actually do something useful */
            /*if (tagOfInterest == null || tagOfInterest.id == LEFT) {
                claw.setPosition(0.3);
                sleep(1500);
                slide(2830);
                sleep (2000);
                moveForward2(1670);
                sleep(1200);
                rotation.setPosition(0.138);

                /*sleep(100);
                claw.setPosition(0.7);
                sleep (600);
                moveForward2(-500);
                sleep(600);
                Strafe_Left(1370);
                sleep(1000);
                slide(0);

                 */

            } else if (tagOfInterest.id == MIDDLE) {

                moveForward2(1600);



            } else if (tagOfInterest.id == RIGHT)  {

                Strafe_Right(1200);
                sleep (1200);
                moveForward2(400);

            }

            while (opModeIsActive()) {
                // Put loop blocks here.
                bl = motorBackLeft.getCurrentPosition();
                br = motorBackRight.getCurrentPosition();
                fl = motorFrontLeft.getCurrentPosition();
                fr = motorFrontRight.getCurrentPosition();

                telemetry.addData("Current Encoder Ticks of Back Left", motorBackLeft.getCurrentPosition());
                telemetry.addData("Current Encoder Ticks of Front Right", motorFrontRight.getCurrentPosition());

                telemetry.addData("Front Left Speed: ", motorFrontLeft.getPower());
                telemetry.addData("Back Left Speed", motorBackLeft.getPower());
                telemetry.addData("Front Right Speed", motorFrontRight.getPower());
                //telemetry.addData("Arm Ticks: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
        }


    /**
     * Describe this function...
     */

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void moveForward2(int encoder_Ticks) {

        yaw = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;
        if (yaw < -5) {

        } else if (yaw > 5) {

        } else {
            motorBackLeft.setPower(speed);
        }

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

    /**
     * Describe this function...
     */
    private void Strafe_Right(int encoder_Ticks) {


        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

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

        telemetry.addData("BL TICKS: ", motorBackLeft.getCurrentPosition());
        telemetry.addData("FR TICKS: ", motorFrontRight.getCurrentPosition());
        telemetry.update();

        sleep(1000);
    }

    private void Strafe_Left(int encoder_Ticks) {
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);

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

    private void limitSwitch(){
        if (slideLimitSwitch.getState() == false) {
            //slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            sleep(100);
            //slideMotor.setTargetPosition(150);
            //slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //slideMotor.setPower(0.4);

        }

    }

    private void slide(int target_Position){
        //slideMotor.setTargetPosition(target_Position);
        //slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //slideMotor.setPower(0.53);

    }

    private void turnLeft(int encoder_Ticks) {
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
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

    /*private void arm(int arm_encoderTicks, int slide_encoderTicks) {

        armTicks = arm_encoderTicks;
        slideTicks = slide_encoderTicks;

        armMotor.setTargetPosition(armTicks);
        slideMotor.setTargetPosition(slideTicks);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.35);
        slideMotor.setVelocity(slideSpeed);

        telemetry.addData("Arm encoder Ticks", armMotor.getCurrentPosition());
        telemetry.update();

        //armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //tester line to see if this builds

     */

}

