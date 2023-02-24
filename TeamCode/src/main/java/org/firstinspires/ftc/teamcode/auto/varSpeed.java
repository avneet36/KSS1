package org.firstinspires.ftc.teamcode.auto;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Autonomous(name = "varSpeed Test")
@Config
public class varSpeed extends LinearOpMode {

    private DcMotorEx motorBackLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    double error;
    double power;

    FtcDashboard dashboard;
    public static double targetPosition = 1500;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");


        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        dashboard = FtcDashboard.getInstance();

        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Error", error);
            telemetry.addData("varSpeed", varspeed());
            telemetry.update();


            motorFrontRight.setPower(varspeed());
            motorFrontLeft.setPower(varspeed());
            motorBackRight.setPower(varspeed());
            motorBackLeft.setPower(varspeed());
        }
    }

    private double varspeed() {
        error = targetPosition - motorBackLeft.getCurrentPosition();
        if (error > 500){

            power = 0.8;
            return power;
        } else if (error == 0) {
            power = 0;
            return power;
        }
        else {
            power = 0.8*(Math.pow((error/500),2));
            // (((-0.8) / Math.pow(1500, 2)) * (Math.pow((error - 1500), 2)) + 0.8);
            return power;

        }
    }

}