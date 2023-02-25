/* Copyright (c) 2017-2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp

public class colorSensor3 extends LinearOpMode {
    ColorSensor colorSensor;
    DcMotor motor;
    ColorSensor colorSensor1;
    DcMotor motor1;
    ColorSensor colorSensor2;
    DcMotor motor2;


    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor1 = hardwareMap.get(ColorSensor.class, "sensorcolor");
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "sensor");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");


        boolean color;
        waitForStart();
        while (opModeIsActive()) {
            // Wait for the Play button to be pressed




            // Sen 1
            if (colorSensor.red() >= 200) {


                motor.setPower(0);

            } else {

                motor.setPower(0.5);

            }

            if (colorSensor.blue() >= 200) {


                motor.setPower(0);

            } else {

                motor.setPower(0.5);

            }


            // Sen 2
            if (colorSensor1.red() >= 200) {


                motor1.setPower(0);

            } else {

                motor1.setPower(0.5);

            }

            if (colorSensor1.blue() >= 200) {


                motor1.setPower(0);

            } else {

                motor1.setPower(0.5);

            }



            if (colorSensor2 instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM));
            }

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();

            // yellow pole
            motor2.setPower(0.5);
            if (colorSensor2.red() >= (104) && colorSensor2.green() >= (104)) { // numbers inside the brackets are the intensity level of the rbg color


                motor2.setPower(0);

            }



        }
    }
}