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
@Autonomous(name = "PID Test")
@Config
public class pidTest extends LinearOpMode {
    double integralSum = 0;
    public static double Kp = 0.002;
    public static double Ki = 0.00005;
    public static double Kd = 0.0005;

    private DcMotorEx motorBackLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;

    FtcDashboard dashboard;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    public static double targetPosition = 2264;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
            double power = PIDControl(targetPosition, motorFrontRight.getCurrentPosition());

            dashboardTelemetry.addData("Motor Position ", motorFrontRight.getCurrentPosition());
            dashboardTelemetry.addData("Power", power);
            dashboardTelemetry.addData("Target Position", targetPosition);
            dashboardTelemetry.addData("Timer", timer);
            dashboardTelemetry.update();

            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackRight.setPower(power);
            motorBackLeft.setPower(power);
        }
    }

    private double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;



        timer.reset();

        return Range.clip((error * Kp) + (derivative * Kd) + (integralSum * Ki),-0.6,0.6);
    }
}