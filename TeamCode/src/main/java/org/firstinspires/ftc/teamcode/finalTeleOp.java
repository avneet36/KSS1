package org.firstinspires.ftc.teamcode;

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
//Tst for git


@TeleOp
public class finalTeleOp extends LinearOpMode {
    //drive
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;

    //arm
    private Servo claw = null;
    private Servo rotation = null;
    private DcMotorEx slide1 = null;
    private DcMotorEx slide2 = null;

    //limit Switch
    private DigitalChannel slideLimitSwitch = null;
    private final ElapsedTime timeSinceToggleClaw = new ElapsedTime();
    private final ElapsedTime timeSinceToggleSlide = new ElapsedTime();


    //variables
    //double slideMotorPower = 0.50;
    int maxSlideEncoderTicks = 3980;
    double slideVelocity = 4000;
    double chassisSpeed = 0.5;

    //boolean Override = true;
    boolean clawBoolean = true;
    boolean manualAuto = true;

    float y;
    float x;
    double rx;
    double denominator;
    String limitSwitchOutput;
    String rotationOutput;
    String clawOutput;
    String isPressed;
    int targetPosition;
    int activate;
    int reset;

    /* 5/8" - tile size
        encoder value to travel half tile =
     */


    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        claw = hardwareMap.get(Servo.class, "claw");
        rotation = hardwareMap.get(Servo.class, "rotation");
        slide1= hardwareMap.get(DcMotorEx.class, "slide1");
        slide2= hardwareMap.get(DcMotorEx.class, "slide2");

        slideLimitSwitch = hardwareMap.get(DigitalChannel.class, "slideLimitSwitch");




        //Direction
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slide1.setDirection(DcMotorEx.Direction.REVERSE);
        slide2.setDirection(DcMotorEx.Direction.FORWARD);



        slideLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        //Arm Encoders
        slide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();



        claw.setPosition(0.7);

        timeSinceToggleClaw.reset();
        timeSinceToggleSlide.reset();
        telemetry.addLine("Init.... Press play");
        telemetry.addData("Slide Current Ticks: ", slide1.getCurrentPosition());
        telemetry.addData("Claw Position: ", claw.getPosition() +  clawOutput);
        telemetry.addData("Rotation Servo Position: ", rotationOutput);
        telemetry.update();

        while (opModeIsActive()) {

            mechanumDrive();
            claw();
            limitSwitch();


            telemetry.addData("Slide1 Speed: ", slide1.getVelocity());
            telemetry.addData("Slide2 Speed: ", slide2.getVelocity());
            telemetry.addData("Claw Position: ", claw.getPosition());
            telemetry.addData("Claw: ", clawOutput);
            telemetry.addData("claw Toggle: ", clawBoolean);
            telemetry.addData("Rotation Servo Position: ", rotation.getPosition());
            telemetry.addData("Limit switch not pressed: ",  slideLimitSwitch.getState());
            telemetry.addData("Target Position: ", targetPosition);
            telemetry.addData("Slide1 Motor Current Ticks: ", slide1.getCurrentPosition());
            telemetry.addData("Slide2 Motor Current Ticks: ", slide2.getCurrentPosition());
            telemetry.addData("Slide Auto Toggle: ", manualAuto);
            telemetry.addData("Chassis Speed", motorBackLeft.getPower());
            telemetry.addData("BL ", motorBackLeft.getCurrentPosition());
            telemetry.addData("FR", motorFrontRight.getCurrentPosition());
            telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
            telemetry.addData("joystick x ", gamepad2.left_stick_x);
            telemetry.addData("joystick y ", gamepad2.left_stick_y);


            telemetry.update();

            if ((slide1.getCurrentPosition() >= maxSlideEncoderTicks) || (slide2.getCurrentPosition() >= maxSlideEncoderTicks)){
                slide1.setTargetPosition(3970);
                slide2.setTargetPosition(3970);
                slide1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slide2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slide1.setVelocity(slideVelocity);
                slide2.setVelocity(slideVelocity);
            }

            if (manualAuto) {
                slideAuto();
            }
            else if (!manualAuto){
                slideManual();
            }

            if ((gamepad1.left_bumper) && (manualAuto) && (timeSinceToggleSlide.milliseconds() > 300)) {
                manualAuto = false;
                timeSinceToggleSlide.reset();

            }
            else if ((gamepad1.left_bumper) && (!manualAuto) && (timeSinceToggleSlide.milliseconds() > 300)) {
                manualAuto = true;
                timeSinceToggleSlide.reset();

            }

        }
    }

    private void mechanumDrive() {
        y = -gamepad2.left_stick_y;
        x = gamepad2.left_stick_x;
        // Counteract imperfect strafing
        rx = gamepad2.right_stick_x * 1.1;
        // Denominator is the largest motor power
        // (absolute value) or 1.
        // This ensures all the powers maintain
        // the same ratio, but only when at least one is
        // out of the range [-1, 1].

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        // Make sure your ID's match your configuration
        motorFrontLeft.setPower((((y + x) + rx) / denominator) * ((0.5*(gamepad2.left_trigger-1)*(gamepad2.left_trigger-1)+0.2) * (0.4*(gamepad2.right_trigger)*(gamepad2.right_trigger)+0.6)));
        motorBackLeft.setPower((((y - x) + rx) / denominator)* ((0.5*(gamepad2.left_trigger-1)*(gamepad2.left_trigger-1)+0.2) * (0.4*(gamepad2.right_trigger)*(gamepad2.right_trigger)+0.6)));
        motorFrontRight.setPower((((y - x) - rx) / denominator)* ((0.5*(gamepad2.left_trigger-1)*(gamepad2.left_trigger-1)+0.2) * (0.4*(gamepad2.right_trigger)*(gamepad2.right_trigger)+0.6)));
        motorBackRight.setPower((((y + x) - rx) / denominator)* ((0.5*(gamepad2.left_trigger-1)*(gamepad2.left_trigger-1)+0.2) * (0.4*(gamepad2.right_trigger)*(gamepad2.right_trigger)+0.6)));



        // ((0.5*(gamepad2.left_trigger-1)*(gamepad2.left_trigger-1)+0.2) * (0.3*(gamepad2.right_trigger)*(gamepad2.right_trigger)+0.6))
        /*if (gamepad2.b){
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

         */




    }

    private void slideManual() {
        //slide
        if (gamepad2.left_bumper && ((slide1.getCurrentPosition() <= maxSlideEncoderTicks) && (slide2.getCurrentPosition() <= maxSlideEncoderTicks))) {
            slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //slideMotor.setVelocity(slideMotorVelocity);
            slide1.setVelocity(slideVelocity);
            slide2.setVelocity(slideVelocity);
        } else if (gamepad2.right_bumper && (slideLimitSwitch.getState() == true)) {
            slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //slideMotor.setVelocity(-slideMotorVelocity);
            slide1.setVelocity(-slideVelocity);
            slide2.setVelocity(-slideVelocity);
        } else {
            slide1.setVelocity(0);
            slide2.setVelocity(0);
            slide2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            slide1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        /*if ((gamepad2.a) && (clawBoolean && timeSinceToggleClaw.milliseconds() > 300)) {
            claw.setPosition(.33);
            clawBoolean = false;
            timeSinceToggleClaw.reset();
            clawOutput = "close";
        }
        else if ((gamepad2.a) && (!clawBoolean && timeSinceToggleClaw.milliseconds() > 300)) {
            claw.setPosition(0.7);
            clawBoolean = true;
            timeSinceToggleClaw.reset();
            clawOutput = "open";
        }

         */


    }

    private void slideAuto(){

        if (gamepad1.dpad_up) {
            // high junction
            targetPosition = 3950;
            isPressed = "dpad_up ";


        } else if (gamepad1.dpad_right || gamepad1.dpad_left) {
            //medium junction
            targetPosition = 2830;
            isPressed = "dpad_right";


        } else if (gamepad1.dpad_down) {
            //low junction
            targetPosition = 1750;
            isPressed = "dpad_down";

            //slide fix position and then drop to ensure getting hte cone on the junction
        } else if (gamepad1.back){
            targetPosition = 0;
        }

        else if (gamepad1.ps && (targetPosition >= 3930) && (targetPosition <= 3970)) {
            targetPosition = 3800;
            //activate = 1;

        }
        else if (gamepad1.ps && (targetPosition >= 2750) && (targetPosition <= 2900)) {
            targetPosition = 2730;
            //activate = 1;

        }
        else if (gamepad1.ps && (targetPosition >= 1700) && (targetPosition <= 1820)) {
            targetPosition = 1650;

            //activate = 1;


        }



        slide1.setTargetPosition(targetPosition);
        slide2.setTargetPosition(targetPosition);
        slide1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slide1.setVelocity(slideVelocity);
        slide2.setVelocity(slideVelocity);

        //so that claw runs after the fix position drop off
        /*if (activate == 1) {
            sleep(700);
            claw.setPosition(0.7);
            clawBoolean = true;
            activate = 0;
        }

         */

    }

   /* private void arm() {
        //BOOLEAN
        if (gamepad1.a && Override && timeSinceToggleArm.milliseconds() > 300){
            Override = false;
            timeSinceToggleArm.reset();
        }else if (gamepad1.a && !Override && timeSinceToggleArm.milliseconds() > 300) {
            Override = true;
            timeSinceToggleArm.reset();

        }if (gamepad1.dpad_up && Override && (armLimitSwitch.getState() == true)) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(0.35);
        }else if (gamepad1.dpad_down && Override && (armMotor.getCurrentPosition() >= minArmEncoderTicks)){
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(-0.35);
        }
        else if (!Override) {
            armMotorCurrentPosition = armMotor.getCurrentPosition();
            Override = true;
            /*armMotor.setTargetPosition(armMotorCurrentPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.15);
    */
        /*else if (gamepad1.y && Override){
            armMotor.setTargetPosition(-400);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.3);
            sleep(1500);
            slideMotor.setTargetPosition(13000);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(0.3);
        }



        }else {
            armMotor.setTargetPosition(armMotorCurrentPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.15);
        }

    }

    */


    private void claw(){
        if ((gamepad1.b || gamepad2.a) && (clawBoolean && timeSinceToggleClaw.milliseconds() > 300)) {
            claw.setPosition(.33);
            clawBoolean = false;
            timeSinceToggleClaw.reset();
            clawOutput = "close";
        }
        else if ((gamepad1.b || gamepad2.a) && !clawBoolean && timeSinceToggleClaw.milliseconds() > 300) {
            claw.setPosition(0.7);
            clawBoolean = true;
            timeSinceToggleClaw.reset();
            clawOutput = "open";
        }

        else if (gamepad1.y && ((slide1.getCurrentPosition() > 200) && (slide2.getCurrentPosition() > 200))){
            // claw forward (approx 0 degrees)
            rotation.setPosition(0.138);
            rotationOutput = "0 degrees";

        }else if (gamepad1.x  && ((slide1.getCurrentPosition() > 200) && (slide2.getCurrentPosition() > 200))){
            // claw forward ( 90 degrees)
            rotation.setPosition(0.46);
            rotationOutput = "90 degrees";

        }else if (gamepad1.a  && ((slide1.getCurrentPosition() > 200) && (slide2.getCurrentPosition() > 200))){
            // claw forward ( 180 degrees)
            rotation.setPosition(0.78);
            rotationOutput = "180 degrees";
        }
    }

    private void limitSwitch(){
        if (slideLimitSwitch.getState() == false) {
            slide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slide2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            sleep(100);
            targetPosition = 50;
            slide1.setTargetPosition(targetPosition);
            slide2.setTargetPosition(targetPosition);
            slide1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slide2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slide1.setVelocity(slideVelocity);
            slide2.setVelocity(slideVelocity);
            String limitSwitchOutput = "yes";
        }
        else if (slideLimitSwitch.getState() == true){
            String limitSwitchOutput = "no";
        }
    }

}